#!/usr/bin/env python3
"""
RoboMaster S1 Control ROS Node
================================
Autonomous control using 15-DOF EKF state estimates
Implements waypoint navigation and path following

Author: RoboMaster EKF ROS Integration
Date: 2025
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger, TriggerResponse, Empty
from actionlib import SimpleActionServer, SimpleActionClient
from actionlib_msgs.msg import GoalStatus
import tf2_ros
import tf2_geometry_msgs
from typing import List, Optional

from iphone_ekf_fusion.msg import EKFState


class RoboMasterController:
    """
    ROS-based autonomous controller for RoboMaster S1
    Uses EKF state estimates for navigation
    """
    
    def __init__(self):
        """Initialize RoboMaster control node"""
        rospy.init_node('robomaster_control', anonymous=False)
        
        # Control parameters
        self.max_linear_vel = rospy.get_param('~max_linear_velocity', 1.0)  # m/s
        self.max_angular_vel = rospy.get_param('~max_angular_velocity', 1.57)  # rad/s
        self.position_tolerance = rospy.get_param('~position_tolerance', 0.2)  # m
        self.yaw_tolerance = rospy.get_param('~yaw_tolerance', 0.1)  # rad
        
        # PID gains
        self.kp_linear = rospy.get_param('~kp_linear', 1.0)
        self.ki_linear = rospy.get_param('~ki_linear', 0.1)
        self.kd_linear = rospy.get_param('~kd_linear', 0.2)
        
        self.kp_angular = rospy.get_param('~kp_angular', 2.0)
        self.ki_angular = rospy.get_param('~ki_angular', 0.1)
        self.kd_angular = rospy.get_param('~kd_angular', 0.3)
        
        # State variables
        self.current_state = None
        self.target_pose = None
        self.waypoints = []
        self.current_waypoint_index = 0
        self.control_mode = "idle"  # idle, waypoint, position_hold, emergency_stop
        
        # PID error tracking
        self.linear_error_integral = 0.0
        self.angular_error_integral = 0.0
        self.last_linear_error = 0.0
        self.last_angular_error = 0.0
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher(
            '/robomaster/cmd_vel', Twist, queue_size=1
        )
        self.path_pub = rospy.Publisher(
            '~planned_path', Path, queue_size=1
        )
        self.status_pub = rospy.Publisher(
            '~status', String, queue_size=1
        )
        
        # Subscribers
        self.ekf_sub = rospy.Subscriber(
            '/ekf_15dof/ekf_state', EKFState, self.ekf_callback
        )
        self.odom_sub = rospy.Subscriber(
            '/ekf_15dof/odom', Odometry, self.odom_callback
        )
        self.goal_sub = rospy.Subscriber(
            '~goal', PoseStamped, self.goal_callback
        )
        
        # Services
        self.start_srv = rospy.Service(
            '~start', Trigger, self.start_service
        )
        self.stop_srv = rospy.Service(
            '~stop', Trigger, self.stop_service
        )
        self.emergency_stop_srv = rospy.Service(
            '~emergency_stop', Empty, self.emergency_stop_service
        )
        self.clear_waypoints_srv = rospy.Service(
            '~clear_waypoints', Empty, self.clear_waypoints_service
        )
        
        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Control timer
        self.control_rate = rospy.get_param('~control_rate', 20.0)
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate),
            self.control_callback
        )
        
        rospy.loginfo("RoboMaster control node initialized")
    
    def ekf_callback(self, msg: EKFState):
        """Process EKF state estimate"""
        self.current_state = msg
    
    def odom_callback(self, msg: Odometry):
        """Process odometry (alternative to EKF state)"""
        # Can be used as backup if EKF state is not available
        pass
    
    def goal_callback(self, msg: PoseStamped):
        """Receive navigation goal"""
        self.target_pose = msg
        self.control_mode = "waypoint"
        
        # Clear previous waypoints and set single goal
        self.waypoints = [msg]
        self.current_waypoint_index = 0
        
        rospy.loginfo(f"New goal received: ({msg.pose.position.x:.2f}, "
                     f"{msg.pose.position.y:.2f})")
    
    def control_callback(self, event):
        """Main control loop"""
        if self.current_state is None:
            return
        
        # Generate control command based on mode
        cmd = Twist()
        
        if self.control_mode == "idle":
            # Zero velocity
            pass
        
        elif self.control_mode == "waypoint":
            cmd = self.waypoint_control()
        
        elif self.control_mode == "position_hold":
            cmd = self.position_hold_control()
        
        elif self.control_mode == "emergency_stop":
            # Zero velocity with mode lock
            pass
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
        
        # Publish status
        status_msg = String()
        status_msg.data = f"Mode: {self.control_mode}, Waypoint: {self.current_waypoint_index}/{len(self.waypoints)}"
        self.status_pub.publish(status_msg)
    
    def waypoint_control(self) -> Twist:
        """Generate control for waypoint navigation"""
        cmd = Twist()
        
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            self.control_mode = "idle"
            return cmd
        
        # Get current waypoint
        target = self.waypoints[self.current_waypoint_index]
        
        # Calculate errors
        dx = target.pose.position.x - self.current_state.position.x
        dy = target.pose.position.y - self.current_state.position.y
        distance = np.sqrt(dx**2 + dy**2)
        
        # Check if waypoint reached
        if distance < self.position_tolerance:
            rospy.loginfo(f"Waypoint {self.current_waypoint_index} reached")
            self.current_waypoint_index += 1
            
            if self.current_waypoint_index >= len(self.waypoints):
                rospy.loginfo("All waypoints completed")
                self.control_mode = "idle"
            
            return cmd
        
        # Calculate desired heading
        desired_yaw = np.arctan2(dy, dx)
        current_yaw = self.current_state.euler_angles.z
        
        # Calculate heading error
        yaw_error = self.normalize_angle(desired_yaw - current_yaw)
        
        # PID control for linear velocity
        linear_error = distance
        self.linear_error_integral += linear_error / self.control_rate
        linear_error_derivative = (linear_error - self.last_linear_error) * self.control_rate
        self.last_linear_error = linear_error
        
        linear_vel = (self.kp_linear * linear_error +
                     self.ki_linear * self.linear_error_integral +
                     self.kd_linear * linear_error_derivative)
        
        # PID control for angular velocity
        self.angular_error_integral += yaw_error / self.control_rate
        angular_error_derivative = (yaw_error - self.last_angular_error) * self.control_rate
        self.last_angular_error = yaw_error
        
        angular_vel = (self.kp_angular * yaw_error +
                      self.ki_angular * self.angular_error_integral +
                      self.kd_angular * angular_error_derivative)
        
        # Apply limits
        cmd.linear.x = np.clip(linear_vel, -self.max_linear_vel, self.max_linear_vel)
        cmd.angular.z = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)
        
        # Reduce linear velocity when turning
        if abs(yaw_error) > 0.5:
            cmd.linear.x *= 0.3
        
        return cmd
    
    def position_hold_control(self) -> Twist:
        """Generate control to hold current position"""
        cmd = Twist()
        
        if self.target_pose is None:
            # Use current position as target
            self.target_pose = PoseStamped()
            self.target_pose.pose.position = self.current_state.position
            self.target_pose.pose.orientation = self.current_state.orientation
        
        # Same as waypoint control but with tighter tolerances
        saved_tolerance = self.position_tolerance
        self.position_tolerance = 0.05  # Tighter tolerance for holding
        
        cmd = self.waypoint_control()
        
        self.position_tolerance = saved_tolerance
        
        return cmd
    
    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-π, π]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def start_service(self, req):
        """Start autonomous control"""
        if self.control_mode == "emergency_stop":
            return TriggerResponse(
                success=False,
                message="Cannot start: Emergency stop active"
            )
        
        self.control_mode = "waypoint" if self.waypoints else "idle"
        self.reset_pid()
        
        return TriggerResponse(
            success=True,
            message=f"Control started in {self.control_mode} mode"
        )
    
    def stop_service(self, req):
        """Stop control"""
        self.control_mode = "idle"
        self.reset_pid()
        
        # Send zero velocity
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        return TriggerResponse(
            success=True,
            message="Control stopped"
        )
    
    def emergency_stop_service(self, req):
        """Emergency stop"""
        self.control_mode = "emergency_stop"
        self.reset_pid()
        
        # Send zero velocity
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        rospy.logwarn("EMERGENCY STOP ACTIVATED")
    
    def clear_waypoints_service(self, req):
        """Clear all waypoints"""
        self.waypoints = []
        self.current_waypoint_index = 0
        self.target_pose = None
        
        rospy.loginfo("Waypoints cleared")
    
    def reset_pid(self):
        """Reset PID controllers"""
        self.linear_error_integral = 0.0
        self.angular_error_integral = 0.0
        self.last_linear_error = 0.0
        self.last_angular_error = 0.0
    
    def add_waypoint(self, x: float, y: float, z: float = 0.0):
        """Add a waypoint to the path"""
        waypoint = PoseStamped()
        waypoint.header.frame_id = "odom"
        waypoint.header.stamp = rospy.Time.now()
        waypoint.pose.position.x = x
        waypoint.pose.position.y = y
        waypoint.pose.position.z = z
        waypoint.pose.orientation.w = 1.0
        
        self.waypoints.append(waypoint)
        self.publish_path()
    
    def publish_path(self):
        """Publish planned path for visualization"""
        path = Path()
        path.header.frame_id = "odom"
        path.header.stamp = rospy.Time.now()
        path.poses = self.waypoints
        
        self.path_pub.publish(path)
    
    def create_square_mission(self, size: float = 2.0):
        """Create a square path mission"""
        self.waypoints = []
        
        # Square corners
        corners = [
            (size/2, size/2),
            (size/2, -size/2),
            (-size/2, -size/2),
            (-size/2, size/2),
            (size/2, size/2)  # Return to start
        ]
        
        for x, y in corners:
            self.add_waypoint(x, y)
        
        rospy.loginfo(f"Created square mission with {len(self.waypoints)} waypoints")
    
    def create_circle_mission(self, radius: float = 1.5, points: int = 16):
        """Create a circular path mission"""
        self.waypoints = []
        
        for i in range(points + 1):
            angle = 2 * np.pi * i / points
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            self.add_waypoint(x, y)
        
        rospy.loginfo(f"Created circle mission with {len(self.waypoints)} waypoints")


def main():
    """Main function"""
    try:
        controller = RoboMasterController()
        
        # Create example mission
        rospy.sleep(2.0)  # Wait for connections
        controller.create_square_mission(size=3.0)
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
