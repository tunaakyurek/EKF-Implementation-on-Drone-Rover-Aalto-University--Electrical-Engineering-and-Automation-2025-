"""
RoboMaster S1 Autonomous Control Module - Lab Environment Compatible
==================================================================
Fixed version for RoboMaster app Lab environment (no numpy/external imports)
Following RoboMaster EKF Formulary specifications

This module provides autonomous control capabilities:
- Waypoint navigation
- Path following
- State-based control
- Pure Python implementation (no numpy)

COMPATIBILITY:
- RoboMaster app Lab environment
- Built-in APIs only (no external imports)
- Memory optimized for S1 (272MB RAM)
- ARM Cortex-A7 compatible

Author: RoboMaster EKF Integration System
Date: 2025
"""

import math
import time

# No external imports - use built-in APIs only
# RoboMaster app provides these automatically:
# - robot_ctrl: Robot control
# - sensor_imu: IMU sensor data
# - sensor_attitude: Attitude data
# - chassis_ctrl: Chassis control
# - rm_define: Constants and definitions

# Control modes as constants (no enum to avoid import issues)
MODE_IDLE = "idle"
MODE_MANUAL = "manual"
MODE_WAYPOINT = "waypoint"
MODE_PATH_FOLLOW = "path_follow"
MODE_HOVER = "hover"
MODE_EMERGENCY_STOP = "emergency_stop"


class Waypoint:
    """Waypoint definition for navigation"""
    
    def __init__(self, x, y, z, yaw=None, speed=0.5, tolerance=0.2, hold_time=0.0):
        self.x = x                      # X position in meters
        self.y = y                      # Y position in meters
        self.z = z                      # Z position in meters (altitude)
        self.yaw = yaw                  # Desired yaw angle in radians
        self.speed = speed              # Speed to reach waypoint (m/s)
        self.tolerance = tolerance      # Position tolerance in meters
        self.hold_time = hold_time      # Time to hold at waypoint in seconds


class ControlCommand:
    """Control command for RoboMaster"""
    
    def __init__(self, vx, vy, vz, yaw_rate, timestamp):
        self.vx = vx                # Forward velocity (m/s)
        self.vy = vy                # Lateral velocity (m/s)
        self.vz = vz                # Vertical velocity (m/s)
        self.yaw_rate = yaw_rate    # Yaw rate (rad/s)
        self.timestamp = timestamp  # Command timestamp


class PIDController:
    """
    PID controller for position and attitude control
    Pure Python implementation (no numpy)
    """
    
    def __init__(self, kp, ki, kd, output_limit=None):
        """
        Initialize PID controller
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_limit: Optional output limits (min, max)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        
        self.error_integral = 0.0
        self.last_error = 0.0
        self.last_time = None
    
    def reset(self):
        """Reset controller state"""
        self.error_integral = 0.0
        self.last_error = 0.0
        self.last_time = None
    
    def update(self, error, dt=None):
        """
        Update PID controller
        
        Args:
            error: Current error
            dt: Time step (if None, computed from system time)
            
        Returns:
            Control output
        """
        # Compute dt if not provided
        if dt is None:
            current_time = time.time()
            if self.last_time is not None:
                dt = current_time - self.last_time
            else:
                dt = 0.02  # Default 50 Hz
            self.last_time = current_time
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.error_integral += error * dt
        i_term = self.ki * self.error_integral
        
        # Derivative term
        if dt > 0:
            error_derivative = (error - self.last_error) / dt
        else:
            error_derivative = 0.0
        d_term = self.kd * error_derivative
        
        # Compute output
        output = p_term + i_term + d_term
        
        # Apply limits using pure Python
        if self.output_limit:
            output = max(self.output_limit[0], min(output, self.output_limit[1]))
            
            # Anti-windup
            if abs(output - (p_term + i_term + d_term)) > 0.001:
                self.error_integral -= error * dt
        
        self.last_error = error
        
        return output


class AutonomousController:
    """
    Main autonomous controller for RoboMaster S1
    Lab environment compatible (no numpy, uses built-in APIs)
    """
    
    def __init__(self, config=None):
        """
        Initialize autonomous controller
        
        Args:
            config: Optional configuration dictionary
        """
        # Load configuration
        self.config = config or {}
        
        # Control mode
        self.mode = MODE_IDLE
        
        # Current state (from sensors)
        self.current_state = None
        
        # Waypoint navigation
        self.waypoints = []
        self.current_waypoint_index = 0
        self.waypoint_reached_time = None
        
        # Path following
        self.path = []
        self.path_index = 0
        
        # Control limits
        self.max_velocity = self.config.get('max_velocity', 1.0)  # m/s
        self.max_yaw_rate = self.config.get('max_yaw_rate', math.pi/2)  # rad/s
        self.max_acceleration = self.config.get('max_acceleration', 0.5)  # m/s²
        
        # PID controllers
        self._init_controllers()
        
        # Control state
        self.is_running = False
        self.control_rate = self.config.get('control_rate', 10)  # Hz (reduced for S1)
        
        # Statistics
        self.stats = {
            'waypoints_reached': 0,
            'total_distance': 0.0,
            'control_updates': 0,
            'mode_changes': []
        }
        
        print("Autonomous controller initialized for RoboMaster Lab")
    
    def _init_controllers(self):
        """Initialize PID controllers"""
        # Position controllers
        self.x_controller = PIDController(
            kp=self.config.get('position_kp', 1.0),
            ki=self.config.get('position_ki', 0.1),
            kd=self.config.get('position_kd', 0.2),
            output_limit=(-self.max_velocity, self.max_velocity)
        )
        
        self.y_controller = PIDController(
            kp=self.config.get('position_kp', 1.0),
            ki=self.config.get('position_ki', 0.1),
            kd=self.config.get('position_kd', 0.2),
            output_limit=(-self.max_velocity, self.max_velocity)
        )
        
        # Yaw controller
        self.yaw_controller = PIDController(
            kp=self.config.get('yaw_kp', 2.0),
            ki=self.config.get('yaw_ki', 0.1),
            kd=self.config.get('yaw_kd', 0.3),
            output_limit=(-self.max_yaw_rate, self.max_yaw_rate)
        )
    
    def update_state_from_sensors(self):
        """
        Update current state from RoboMaster sensors
        Uses built-in sensor APIs
        """
        try:
            # Get IMU data using built-in APIs
            gyro_data = sensor_imu.get_gyroscope()
            accel_data = sensor_imu.get_accelerometer()
            attitude_data = sensor_attitude.get_attitude()
            
            # Extract sensor values (adjust based on actual API)
            # Note: This uses the built-in RoboMaster Lab API format
            self.current_state = {
                'x': 0.0,  # Position estimation would need integration
                'y': 0.0,  # Position estimation would need integration
                'z': 0.0,  # No altitude sensor on S1
                'yaw': attitude_data[2] if attitude_data else 0.0,  # Yaw from attitude
                'pitch': attitude_data[1] if attitude_data else 0.0,
                'roll': attitude_data[0] if attitude_data else 0.0,
                'gyro_x': gyro_data[0] if gyro_data else 0.0,
                'gyro_y': gyro_data[1] if gyro_data else 0.0,
                'gyro_z': gyro_data[2] if gyro_data else 0.0,
                'accel_x': accel_data[0] if accel_data else 0.0,
                'accel_y': accel_data[1] if accel_data else 0.0,
                'accel_z': accel_data[2] if accel_data else 0.0
            }
            
        except Exception as e:
            print("Sensor update error: {}".format(e))
            # Fallback to basic state
            self.current_state = {
                'x': 0.0, 'y': 0.0, 'z': 0.0,
                'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0,
                'gyro_x': 0.0, 'gyro_y': 0.0, 'gyro_z': 0.0,
                'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': 0.0
            }
    
    def set_mode(self, mode):
        """
        Set control mode
        
        Args:
            mode: New control mode
        """
        if mode != self.mode:
            print("Control mode changed: {} -> {}".format(self.mode, mode))
            self.mode = mode
            self.stats['mode_changes'].append({
                'time': time.time(),
                'from': self.mode,
                'to': mode
            })
            
            # Reset controllers on mode change
            self._reset_controllers()
    
    def _reset_controllers(self):
        """Reset all PID controllers"""
        self.x_controller.reset()
        self.y_controller.reset()
        self.yaw_controller.reset()
    
    def set_waypoints(self, waypoints):
        """
        Set waypoints for navigation
        
        Args:
            waypoints: List of waypoints
        """
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.waypoint_reached_time = None
        print("Set {} waypoints".format(len(waypoints)))
    
    def add_waypoint(self, waypoint):
        """
        Add a single waypoint
        
        Args:
            waypoint: Waypoint to add
        """
        self.waypoints.append(waypoint)
        print("Added waypoint: ({:.2f}, {:.2f}, {:.2f})".format(waypoint.x, waypoint.y, waypoint.z))
    
    def clear_waypoints(self):
        """Clear all waypoints"""
        self.waypoints = []
        self.current_waypoint_index = 0
        print("Waypoints cleared")
    
    def run_single_update(self):
        """
        Run a single control update cycle
        Designed for manual calling in Lab environment
        """
        if not self.is_running:
            return None
        
        try:
            # Update state from sensors
            self.update_state_from_sensors()
            
            if self.current_state is None:
                print("No state available")
                return None
            
            # Compute control command based on mode
            command = None
            
            if self.mode == MODE_IDLE:
                command = self._idle_control()
            
            elif self.mode == MODE_WAYPOINT:
                command = self._waypoint_control(self.current_state)
            
            elif self.mode == MODE_HOVER:
                command = self._hover_control(self.current_state)
            
            elif self.mode == MODE_EMERGENCY_STOP:
                command = self._emergency_stop()
            
            # Execute command using built-in APIs
            if command:
                self._execute_command(command)
                self.stats['control_updates'] += 1
            
            return command
            
        except Exception as e:
            print("Control update error: {}".format(e))
            return None
    
    def _execute_command(self, command):
        """
        Execute control command using RoboMaster built-in APIs
        
        Args:
            command: ControlCommand to execute
        """
        try:
            # Convert velocities to RoboMaster chassis commands
            # Scale to appropriate units (mm/s for chassis)
            vx_mm = int(command.vx * 1000)  # Convert m/s to mm/s
            vy_mm = int(command.vy * 1000)
            
            # Convert yaw rate to degrees/s
            yaw_deg = math.degrees(command.yaw_rate)
            
            # Send command to chassis using built-in API
            chassis_ctrl.set_trans_speed(vx_mm)  # Forward speed
            chassis_ctrl.set_rotate_speed(yaw_deg)  # Rotation speed
            
            # Note: RoboMaster S1 doesn't have vertical movement (vz ignored)
            
        except Exception as e:
            print("Command execution error: {}".format(e))
    
    def start(self):
        """Start autonomous control"""
        if self.is_running:
            print("Controller already running")
            return
        
        self.is_running = True
        
        # Set robot to appropriate mode
        try:
            robot_ctrl.set_mode(rm_define.robot_mode_chassis_follow)
        except Exception as e:
            print("Robot mode set error: {}".format(e))
        
        print("Autonomous control started")
    
    def stop(self):
        """Stop autonomous control"""
        self.is_running = False
        
        # Stop chassis movement
        try:
            chassis_ctrl.set_trans_speed(0)
            chassis_ctrl.set_rotate_speed(0)
        except Exception as e:
            print("Stop error: {}".format(e))
        
        print("Autonomous control stopped")
    
    def _idle_control(self):
        """Idle control - zero velocity"""
        return ControlCommand(
            vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0,
            timestamp=time.time()
        )
    
    def _waypoint_control(self, state):
        """
        Waypoint navigation control
        
        Args:
            state: Current state dictionary
            
        Returns:
            Control command or None
        """
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            # No more waypoints
            self.set_mode(MODE_IDLE)
            return None
        
        waypoint = self.waypoints[self.current_waypoint_index]
        
        # Extract current position (estimated)
        current_x = state.get('x', 0.0)
        current_y = state.get('y', 0.0)
        current_yaw = state.get('yaw', 0.0)
        
        # Compute errors
        error_x = waypoint.x - current_x
        error_y = waypoint.y - current_y
        
        # Distance to waypoint (2D only for S1)
        distance = math.sqrt(error_x**2 + error_y**2)
        
        # Check if waypoint reached
        if distance < waypoint.tolerance:
            if self.waypoint_reached_time is None:
                self.waypoint_reached_time = time.time()
                print("Waypoint {} reached".format(self.current_waypoint_index))
                self.stats['waypoints_reached'] += 1
            
            # Check hold time
            if time.time() - self.waypoint_reached_time >= waypoint.hold_time:
                # Move to next waypoint
                self.current_waypoint_index += 1
                self.waypoint_reached_time = None
                
                if self.current_waypoint_index >= len(self.waypoints):
                    print("All waypoints completed")
                    self.set_mode(MODE_IDLE)
                    return None
        
        # Compute control commands
        vx_body = self.x_controller.update(error_x)
        vy_body = self.y_controller.update(error_y)
        
        # Transform to body frame
        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        vx = vx_body * cos_yaw - vy_body * sin_yaw
        vy = vx_body * sin_yaw + vy_body * cos_yaw
        
        # Yaw control
        yaw_rate = 0.0
        if waypoint.yaw is not None:
            yaw_error = self._normalize_angle(waypoint.yaw - current_yaw)
            yaw_rate = self.yaw_controller.update(yaw_error)
        
        # Scale velocities based on waypoint speed
        velocity_scale = min(1.0, waypoint.speed / self.max_velocity)
        vx *= velocity_scale
        vy *= velocity_scale
        
        return ControlCommand(
            vx=vx, vy=vy, vz=0.0, yaw_rate=yaw_rate,
            timestamp=time.time()
        )
    
    def _hover_control(self, state):
        """
        Hover control - maintain current position
        
        Args:
            state: Current state dictionary
            
        Returns:
            Control command
        """
        # For hovering, we maintain current position
        # For simplicity, we'll just command zero velocity
        return ControlCommand(
            vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0,
            timestamp=time.time()
        )
    
    def _emergency_stop(self):
        """Emergency stop - immediate zero velocity"""
        return ControlCommand(
            vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0,
            timestamp=time.time()
        )
    
    def _normalize_angle(self, angle):
        """Normalize angle to [-π, π] using pure Python"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def get_statistics(self):
        """Get controller statistics"""
        return {
            'mode': self.mode,
            'waypoints_reached': self.stats['waypoints_reached'],
            'total_distance': self.stats['total_distance'],
            'control_updates': self.stats['control_updates'],
            'waypoint_progress': "{}/{}".format(self.current_waypoint_index, len(self.waypoints)),
            'current_state': self.current_state
        }


# Mission planning utilities (pure Python)
class MissionPlanner:
    """
    Mission planning utilities for autonomous navigation
    Pure Python implementation (no numpy)
    """
    
    @staticmethod
    def create_square_path(center, size, points_per_side=5):
        """
        Create a square path of waypoints
        
        Args:
            center: Center position (x, y)
            size: Size of square in meters
            points_per_side: Number of waypoints per side
            
        Returns:
            List of waypoints
        """
        waypoints = []
        half_size = size / 2
        
        # Define corners
        corners = [
            (center[0] - half_size, center[1] - half_size),  # Bottom-left
            (center[0] + half_size, center[1] - half_size),  # Bottom-right
            (center[0] + half_size, center[1] + half_size),  # Top-right
            (center[0] - half_size, center[1] + half_size),  # Top-left
            (center[0] - half_size, center[1] - half_size),  # Back to start
        ]
        
        # Create waypoints along edges
        for i in range(len(corners) - 1):
            start = corners[i]
            end = corners[i + 1]
            
            for j in range(points_per_side):
                t = j / points_per_side
                x = start[0] + t * (end[0] - start[0])
                y = start[1] + t * (end[1] - start[1])
                
                waypoints.append(Waypoint(
                    x=x, y=y, z=0.0,  # Z=0 for ground robot
                    speed=0.3, tolerance=0.2
                ))
        
        return waypoints
    
    @staticmethod
    def create_circle_path(center, radius, num_points=12):
        """
        Create a circular path of waypoints
        
        Args:
            center: Center position (x, y)
            radius: Radius in meters
            num_points: Number of waypoints
            
        Returns:
            List of waypoints
        """
        waypoints = []
        
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)
            
            # Face direction of motion
            next_angle = 2 * math.pi * ((i + 1) % num_points) / num_points
            yaw = math.atan2(
                math.sin(next_angle) - math.sin(angle),
                math.cos(next_angle) - math.cos(angle)
            )
            
            waypoints.append(Waypoint(
                x=x, y=y, z=0.0, yaw=yaw,
                speed=0.2, tolerance=0.15
            ))
        
        return waypoints


# Example usage for RoboMaster Lab
def run_autonomous_demo():
    """
    Simple autonomous demo for RoboMaster Lab
    This function can be called directly in the Lab environment
    """
    print("Starting RoboMaster S1 Autonomous Demo")
    
    # Create controller with conservative settings for S1
    config = {
        'max_velocity': 0.5,  # Reduced for safety
        'max_yaw_rate': math.pi/4,  # Reduced for stability
        'control_rate': 5,  # Reduced for S1 performance
        'position_kp': 0.8,
        'position_ki': 0.05,
        'position_kd': 0.2
    }
    
    controller = AutonomousController(config)
    
    # Create simple mission
    planner = MissionPlanner()
    
    # Small square path (1m x 1m)
    waypoints = planner.create_square_path(
        center=(0, 0), size=1.0, points_per_side=3
    )
    
    controller.set_waypoints(waypoints)
    controller.set_mode(MODE_WAYPOINT)
    
    # Start controller
    controller.start()
    
    # Run for limited time (30 seconds max)
    start_time = time.time()
    max_runtime = 30.0
    
    try:
        while time.time() - start_time < max_runtime:
            # Run single update
            command = controller.run_single_update()
            
            if command:
                print("Command: vx={:.2f}, vy={:.2f}, yaw_rate={:.2f}".format(command.vx, command.vy, command.yaw_rate))
            
            # Print statistics every 2 seconds
            if int((time.time() - start_time) % 2) == 0:
                stats = controller.get_statistics()
                print("Stats: {}".format(stats))
            
            # Sleep to maintain rate
            time.sleep(1.0 / controller.control_rate)
            
            # Check if mission completed
            if controller.mode == MODE_IDLE:
                print("Mission completed!")
                break
    
    except Exception as e:
        print("Demo error: {}".format(e))
    
    finally:
        controller.stop()
        print("Demo finished")


# For Lab environment - uncomment this line to run demo:
# run_autonomous_demo()
