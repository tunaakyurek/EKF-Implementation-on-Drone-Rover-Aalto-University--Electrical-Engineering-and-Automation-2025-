#!/usr/bin/env python3
"""
EKF Visualization Node
======================
Real-time visualization of EKF state and covariance

Author: RoboMaster EKF ROS Integration
Date: 2025
"""

import rospy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA, Header
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

from iphone_ekf_fusion.msg import EKFState


class EKFVisualizer:
    """
    Visualizes EKF state, covariance, and trajectory
    """
    
    def __init__(self):
        """Initialize visualizer"""
        rospy.init_node('ekf_visualizer', anonymous=False)
        
        # Parameters
        self.update_rate = rospy.get_param('~update_rate', 10.0)
        self.trajectory_length = rospy.get_param('~trajectory_length', 500)
        self.show_covariance = rospy.get_param('~show_covariance', True)
        self.show_bias = rospy.get_param('~show_bias', True)
        
        # Publishers
        self.marker_pub = rospy.Publisher(
            '~markers', MarkerArray, queue_size=1
        )
        self.trajectory_pub = rospy.Publisher(
            '~trajectory', Path, queue_size=1
        )
        self.covariance_pub = rospy.Publisher(
            '~covariance_cloud', PointCloud2, queue_size=1
        )
        
        # Subscribers
        self.ekf_sub = rospy.Subscriber(
            '/ekf_15dof/ekf_state', EKFState, self.ekf_callback
        )
        
        # State storage
        self.trajectory = []
        self.current_state = None
        
        # Visualization timer
        self.vis_timer = rospy.Timer(
            rospy.Duration(1.0 / self.update_rate),
            self.visualization_callback
        )
        
        rospy.loginfo("EKF visualizer started")
    
    def ekf_callback(self, msg: EKFState):
        """Process EKF state"""
        self.current_state = msg
        
        # Add to trajectory
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = msg.position
        pose.pose.orientation = msg.orientation
        
        self.trajectory.append(pose)
        
        # Limit trajectory length
        if len(self.trajectory) > self.trajectory_length:
            self.trajectory.pop(0)
    
    def visualization_callback(self, event):
        """Update visualizations"""
        if self.current_state is None:
            return
        
        # Create marker array
        markers = MarkerArray()
        
        # Add position marker
        markers.markers.append(self.create_position_marker())
        
        # Add velocity vector
        markers.markers.append(self.create_velocity_marker())
        
        # Add orientation axes
        markers.markers.extend(self.create_orientation_markers())
        
        # Add covariance ellipsoid
        if self.show_covariance:
            markers.markers.append(self.create_covariance_marker())
        
        # Add bias indicators
        if self.show_bias:
            markers.markers.extend(self.create_bias_markers())
        
        # Publish markers
        self.marker_pub.publish(markers)
        
        # Publish trajectory
        self.publish_trajectory()
        
        # Publish covariance cloud
        self.publish_covariance_cloud()
    
    def create_position_marker(self) -> Marker:
        """Create position marker"""
        marker = Marker()
        marker.header = self.current_state.header
        marker.ns = "position"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position = self.current_state.position
        marker.pose.orientation = self.current_state.orientation
        
        marker.scale = Vector3(0.2, 0.2, 0.2)
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
        
        return marker
    
    def create_velocity_marker(self) -> Marker:
        """Create velocity vector marker"""
        marker = Marker()
        marker.header = self.current_state.header
        marker.ns = "velocity"
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Arrow from position to position + velocity
        marker.points = [
            self.current_state.position,
            Point(
                x=self.current_state.position.x + self.current_state.velocity.x,
                y=self.current_state.position.y + self.current_state.velocity.y,
                z=self.current_state.position.z + self.current_state.velocity.z
            )
        ]
        
        marker.scale = Vector3(0.05, 0.1, 0.1)
        marker.color = ColorRGBA(1.0, 1.0, 0.0, 1.0)
        
        return marker
    
    def create_orientation_markers(self) -> List[Marker]:
        """Create orientation axes markers"""
        markers = []
        
        # Get rotation matrix from quaternion
        q = self.current_state.orientation
        R = self.quaternion_to_rotation_matrix(q)
        
        # Axes colors and directions
        axes = [
            ('x', [1, 0, 0], ColorRGBA(1.0, 0.0, 0.0, 1.0)),
            ('y', [0, 1, 0], ColorRGBA(0.0, 1.0, 0.0, 1.0)),
            ('z', [0, 0, 1], ColorRGBA(0.0, 0.0, 1.0, 1.0))
        ]
        
        for i, (name, direction, color) in enumerate(axes):
            marker = Marker()
            marker.header = self.current_state.header
            marker.ns = f"orientation_{name}"
            marker.id = i + 10
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # Rotate direction by orientation
            rotated_dir = R @ np.array(direction)
            
            marker.points = [
                self.current_state.position,
                Point(
                    x=self.current_state.position.x + rotated_dir[0] * 0.5,
                    y=self.current_state.position.y + rotated_dir[1] * 0.5,
                    z=self.current_state.position.z + rotated_dir[2] * 0.5
                )
            ]
            
            marker.scale = Vector3(0.02, 0.04, 0.04)
            marker.color = color
            
            markers.append(marker)
        
        return markers
    
    def create_covariance_marker(self) -> Marker:
        """Create covariance ellipsoid marker"""
        marker = Marker()
        marker.header = self.current_state.header
        marker.ns = "covariance"
        marker.id = 20
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position = self.current_state.position
        
        # Extract position covariance (3x3 submatrix)
        P = np.array(self.current_state.covariance).reshape(15, 15)
        P_pos = P[0:3, 0:3]
        
        # Compute eigenvalues for ellipsoid scale
        eigenvalues, _ = np.linalg.eig(P_pos)
        
        # 2-sigma ellipsoid
        marker.scale = Vector3(
            2 * np.sqrt(abs(eigenvalues[0])),
            2 * np.sqrt(abs(eigenvalues[1])),
            2 * np.sqrt(abs(eigenvalues[2]))
        )
        
        marker.color = ColorRGBA(1.0, 0.0, 1.0, 0.3)
        
        return marker
    
    def create_bias_markers(self) -> List[Marker]:
        """Create bias indicator markers"""
        markers = []
        
        # Accelerometer bias
        bias_marker = Marker()
        bias_marker.header = self.current_state.header
        bias_marker.ns = "accel_bias"
        bias_marker.id = 30
        bias_marker.type = Marker.TEXT_VIEW_FACING
        bias_marker.action = Marker.ADD
        
        bias_marker.pose.position.x = self.current_state.position.x + 1.0
        bias_marker.pose.position.y = self.current_state.position.y
        bias_marker.pose.position.z = self.current_state.position.z + 0.5
        
        bias_text = f"Bias: [{self.current_state.accelerometer_bias.x:.3f}, "
        bias_text += f"{self.current_state.accelerometer_bias.y:.3f}, "
        bias_text += f"{self.current_state.accelerometer_bias.z:.3f}]"
        bias_marker.text = bias_text
        
        bias_marker.scale.z = 0.1
        bias_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        
        markers.append(bias_marker)
        
        return markers
    
    def publish_trajectory(self):
        """Publish trajectory path"""
        if not self.trajectory:
            return
        
        path = Path()
        path.header = self.current_state.header
        path.poses = self.trajectory
        
        self.trajectory_pub.publish(path)
    
    def publish_covariance_cloud(self):
        """Publish covariance as point cloud"""
        # Sample points from covariance distribution
        P = np.array(self.current_state.covariance).reshape(15, 15)
        P_pos = P[0:3, 0:3]
        
        mean = np.array([
            self.current_state.position.x,
            self.current_state.position.y,
            self.current_state.position.z
        ])
        
        # Generate samples
        n_samples = 100
        samples = np.random.multivariate_normal(mean, P_pos, n_samples)
        
        # Create point cloud
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.current_state.header.frame_id
        
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1)
        ]
        
        points = []
        for sample in samples:
            intensity = np.exp(-0.5 * (sample - mean).T @ np.linalg.inv(P_pos) @ (sample - mean))
            points.append([sample[0], sample[1], sample[2], intensity])
        
        cloud = pc2.create_cloud(header, fields, points)
        self.covariance_pub.publish(cloud)
    
    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion to rotation matrix"""
        w, x, y, z = q.w, q.x, q.y, q.z
        
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
        ])


if __name__ == '__main__':
    try:
        visualizer = EKFVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
