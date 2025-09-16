"""
RoboMaster S1 Autonomous Control Module
========================================
Integrates EKF state estimates for autonomous navigation
Following RoboMaster EKF Formulary specifications

This module provides autonomous control capabilities:
- Waypoint navigation
- Path following
- Obstacle avoidance
- State-based control

Author: RoboMaster EKF Integration System
Date: 2025
"""

import numpy as np
import time
import logging
from dataclasses import dataclass
from typing import List, Optional, Tuple, Dict, Any
from enum import Enum
import threading
import queue

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class ControlMode(Enum):
    """Control modes for the autonomous controller"""
    IDLE = "idle"
    MANUAL = "manual"
    WAYPOINT = "waypoint"
    PATH_FOLLOW = "path_follow"
    HOVER = "hover"
    EMERGENCY_STOP = "emergency_stop"


@dataclass
class Waypoint:
    """Waypoint definition for navigation"""
    x: float                    # X position in meters
    y: float                    # Y position in meters
    z: float                    # Z position in meters (altitude)
    yaw: Optional[float] = None # Desired yaw angle in radians
    speed: float = 0.5          # Speed to reach waypoint (m/s)
    tolerance: float = 0.2      # Position tolerance in meters
    hold_time: float = 0.0      # Time to hold at waypoint in seconds


@dataclass
class ControlCommand:
    """Control command for RoboMaster"""
    vx: float          # Forward velocity (m/s)
    vy: float          # Lateral velocity (m/s)
    vz: float          # Vertical velocity (m/s)
    yaw_rate: float    # Yaw rate (rad/s)
    timestamp: float   # Command timestamp


class PIDController:
    """
    PID controller for position and attitude control
    """
    
    def __init__(self, kp: float, ki: float, kd: float, 
                 output_limit: Optional[Tuple[float, float]] = None):
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
    
    def update(self, error: float, dt: Optional[float] = None) -> float:
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
        
        # Apply limits
        if self.output_limit:
            output = np.clip(output, self.output_limit[0], self.output_limit[1])
            
            # Anti-windup
            if abs(output - (p_term + i_term + d_term)) > 0.001:
                self.error_integral -= error * dt
        
        self.last_error = error
        
        return output


class AutonomousController:
    """
    Main autonomous controller for RoboMaster S1
    Integrates EKF state estimates for navigation
    """
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize autonomous controller
        
        Args:
            config: Optional configuration dictionary
        """
        # Load configuration
        self.config = config or {}
        
        # Control mode
        self.mode = ControlMode.IDLE
        
        # Current state (from EKF)
        self.current_state = None
        self.state_lock = threading.Lock()
        
        # Waypoint navigation
        self.waypoints = []
        self.current_waypoint_index = 0
        self.waypoint_reached_time = None
        
        # Path following
        self.path = []
        self.path_index = 0
        
        # Control limits
        self.max_velocity = self.config.get('max_velocity', 1.0)  # m/s
        self.max_yaw_rate = self.config.get('max_yaw_rate', np.pi/2)  # rad/s
        self.max_acceleration = self.config.get('max_acceleration', 0.5)  # m/s²
        
        # PID controllers
        self._init_controllers()
        
        # Control thread
        self.control_thread = None
        self.is_running = False
        self.control_rate = self.config.get('control_rate', 20)  # Hz
        
        # Command queue
        self.command_queue = queue.Queue(maxsize=10)
        
        # Statistics
        self.stats = {
            'waypoints_reached': 0,
            'total_distance': 0.0,
            'control_updates': 0,
            'mode_changes': []
        }
        
        logger.info("Autonomous controller initialized")
    
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
        
        self.z_controller = PIDController(
            kp=self.config.get('altitude_kp', 1.5),
            ki=self.config.get('altitude_ki', 0.2),
            kd=self.config.get('altitude_kd', 0.3),
            output_limit=(-self.max_velocity * 0.5, self.max_velocity * 0.5)
        )
        
        # Yaw controller
        self.yaw_controller = PIDController(
            kp=self.config.get('yaw_kp', 2.0),
            ki=self.config.get('yaw_ki', 0.1),
            kd=self.config.get('yaw_kd', 0.3),
            output_limit=(-self.max_yaw_rate, self.max_yaw_rate)
        )
    
    def update_state(self, state):
        """
        Update current state from EKF
        
        Args:
            state: State dictionary or EKF8State object containing position, orientation, etc.
        """
        with self.state_lock:
            # Handle both dictionary and EKF8State object
            if hasattr(state, 'to_array'):  # EKF8State object
                self.current_state = {
                    'position': [state.x, state.y, state.z],
                    'velocity': [0.0, 0.0, state.vz],  # Only vz available in 8-DOF
                    'orientation': [state.roll, state.pitch, state.yaw],
                    'angular_velocity': [0.0, 0.0, state.yaw_rate]  # Only yaw_rate in 8-DOF
                }
            else:  # Dictionary
                self.current_state = state
    
    def set_mode(self, mode: ControlMode):
        """
        Set control mode
        
        Args:
            mode: New control mode
        """
        if mode != self.mode:
            logger.info(f"Control mode changed: {self.mode.value} -> {mode.value}")
            self.mode = mode
            self.stats['mode_changes'].append({
                'time': time.time(),
                'from': self.mode.value,
                'to': mode.value
            })
            
            # Reset controllers on mode change
            self._reset_controllers()
    
    def _reset_controllers(self):
        """Reset all PID controllers"""
        self.x_controller.reset()
        self.y_controller.reset()
        self.z_controller.reset()
        self.yaw_controller.reset()
    
    def set_waypoints(self, waypoints: List[Waypoint]):
        """
        Set waypoints for navigation
        
        Args:
            waypoints: List of waypoints
        """
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.waypoint_reached_time = None
        logger.info(f"Set {len(waypoints)} waypoints")
    
    def add_waypoint(self, waypoint: Waypoint):
        """
        Add a single waypoint
        
        Args:
            waypoint: Waypoint to add
        """
        self.waypoints.append(waypoint)
        logger.info(f"Added waypoint: ({waypoint.x:.2f}, {waypoint.y:.2f}, {waypoint.z:.2f})")
    
    def clear_waypoints(self):
        """Clear all waypoints"""
        self.waypoints = []
        self.current_waypoint_index = 0
        logger.info("Waypoints cleared")
    
    def set_path(self, path: List[Tuple[float, float, float]]):
        """
        Set path for following
        
        Args:
            path: List of (x, y, z) positions
        """
        self.path = path
        self.path_index = 0
        logger.info(f"Set path with {len(path)} points")
    
    def start(self):
        """Start autonomous control"""
        if self.is_running:
            logger.warning("Controller already running")
            return
        
        self.is_running = True
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        logger.info("Autonomous control started")
    
    def stop(self):
        """Stop autonomous control"""
        self.is_running = False
        if self.control_thread:
            self.control_thread.join(timeout=2)
        logger.info("Autonomous control stopped")
    
    def _control_loop(self):
        """Main control loop"""
        dt = 1.0 / self.control_rate
        
        while self.is_running:
            try:
                start_time = time.time()
                
                # Get current state
                with self.state_lock:
                    state = self.current_state
                
                if state is None:
                    logger.warning("No state available")
                    time.sleep(dt)
                    continue
                
                # Compute control command based on mode
                command = None
                
                if self.mode == ControlMode.IDLE:
                    command = self._idle_control()
                
                elif self.mode == ControlMode.WAYPOINT:
                    command = self._waypoint_control(state)
                
                elif self.mode == ControlMode.PATH_FOLLOW:
                    command = self._path_follow_control(state)
                
                elif self.mode == ControlMode.HOVER:
                    command = self._hover_control(state)
                
                elif self.mode == ControlMode.EMERGENCY_STOP:
                    command = self._emergency_stop()
                
                # Send command
                if command:
                    if not self.command_queue.full():
                        self.command_queue.put(command)
                    self.stats['control_updates'] += 1
                
                # Sleep to maintain rate
                elapsed = time.time() - start_time
                if elapsed < dt:
                    time.sleep(dt - elapsed)
            
            except Exception as e:
                logger.error(f"Control loop error: {e}")
    
    def _idle_control(self) -> ControlCommand:
        """Idle control - zero velocity"""
        return ControlCommand(
            vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0,
            timestamp=time.time()
        )
    
    def _waypoint_control(self, state: Dict) -> Optional[ControlCommand]:
        """
        Waypoint navigation control
        
        Args:
            state: Current state dictionary
            
        Returns:
            Control command or None
        """
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            # No more waypoints
            self.set_mode(ControlMode.IDLE)
            return None
        
        waypoint = self.waypoints[self.current_waypoint_index]
        
        # Extract current position
        current_x = state.get('x', 0.0)
        current_y = state.get('y', 0.0)
        current_z = state.get('z', 0.0)
        current_yaw = state.get('yaw', 0.0)
        
        # Compute errors
        error_x = waypoint.x - current_x
        error_y = waypoint.y - current_y
        error_z = waypoint.z - current_z
        
        # Distance to waypoint
        distance = np.sqrt(error_x**2 + error_y**2 + error_z**2)
        
        # Check if waypoint reached
        if distance < waypoint.tolerance:
            if self.waypoint_reached_time is None:
                self.waypoint_reached_time = time.time()
                logger.info(f"Waypoint {self.current_waypoint_index} reached")
                self.stats['waypoints_reached'] += 1
            
            # Check hold time
            if time.time() - self.waypoint_reached_time >= waypoint.hold_time:
                # Move to next waypoint
                self.current_waypoint_index += 1
                self.waypoint_reached_time = None
                
                if self.current_waypoint_index >= len(self.waypoints):
                    logger.info("All waypoints completed")
                    self.set_mode(ControlMode.IDLE)
                    return None
        
        # Compute control commands
        vx_body = self.x_controller.update(error_x)
        vy_body = self.y_controller.update(error_y)
        vz = self.z_controller.update(error_z)
        
        # Transform to body frame
        cos_yaw = np.cos(current_yaw)
        sin_yaw = np.sin(current_yaw)
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
            vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate,
            timestamp=time.time()
        )
    
    def _path_follow_control(self, state: Dict) -> Optional[ControlCommand]:
        """
        Path following control
        
        Args:
            state: Current state dictionary
            
        Returns:
            Control command or None
        """
        if not self.path or self.path_index >= len(self.path):
            self.set_mode(ControlMode.IDLE)
            return None
        
        # Get current and target positions
        current_pos = np.array([state.get('x', 0.0), state.get('y', 0.0), state.get('z', 0.0)])
        target_pos = np.array(self.path[self.path_index])
        
        # Look-ahead for smoother following
        lookahead_distance = 0.5  # meters
        lookahead_index = self.path_index
        
        while lookahead_index < len(self.path) - 1:
            next_pos = np.array(self.path[lookahead_index + 1])
            if np.linalg.norm(next_pos - current_pos) > lookahead_distance:
                break
            lookahead_index += 1
        
        target_pos = np.array(self.path[lookahead_index])
        
        # Compute errors
        error = target_pos - current_pos
        distance = np.linalg.norm(error)
        
        # Check if point reached
        if distance < 0.2:  # 20cm tolerance
            self.path_index = min(self.path_index + 1, len(self.path) - 1)
            self.stats['total_distance'] += distance
        
        # Compute velocities
        if distance > 0.01:
            direction = error / distance
            speed = min(self.max_velocity, distance)
            velocity = direction * speed
            
            vx = velocity[0]
            vy = velocity[1]
            vz = velocity[2]
        else:
            vx = vy = vz = 0.0
        
        # Simple yaw alignment with direction of motion
        if abs(vx) > 0.01 or abs(vy) > 0.01:
            desired_yaw = np.arctan2(vy, vx)
            current_yaw = state.get('yaw', 0.0)
            yaw_error = self._normalize_angle(desired_yaw - current_yaw)
            yaw_rate = self.yaw_controller.update(yaw_error)
        else:
            yaw_rate = 0.0
        
        return ControlCommand(
            vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate,
            timestamp=time.time()
        )
    
    def _hover_control(self, state: Dict) -> ControlCommand:
        """
        Hover control - maintain current position
        
        Args:
            state: Current state dictionary
            
        Returns:
            Control command
        """
        # For hovering, we maintain current position
        # This would typically use position hold from the first hover command
        # For simplicity, we'll just command zero velocity
        return ControlCommand(
            vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0,
            timestamp=time.time()
        )
    
    def _emergency_stop(self) -> ControlCommand:
        """Emergency stop - immediate zero velocity"""
        return ControlCommand(
            vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0,
            timestamp=time.time()
        )
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-π, π]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def get_command(self, timeout: float = 0.1) -> Optional[ControlCommand]:
        """
        Get latest control command
        
        Args:
            timeout: Queue timeout in seconds
            
        Returns:
            Control command or None
        """
        try:
            return self.command_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get controller statistics"""
        return {
            'mode': self.mode.value,
            'waypoints_reached': self.stats['waypoints_reached'],
            'total_distance': self.stats['total_distance'],
            'control_updates': self.stats['control_updates'],
            'waypoint_progress': f"{self.current_waypoint_index}/{len(self.waypoints)}",
            'path_progress': f"{self.path_index}/{len(self.path)}"
        }


# Mission planning utilities
class MissionPlanner:
    """
    Mission planning utilities for autonomous navigation
    """
    
    @staticmethod
    def create_square_path(center: Tuple[float, float], size: float, 
                          altitude: float, points_per_side: int = 10) -> List[Waypoint]:
        """
        Create a square path of waypoints
        
        Args:
            center: Center position (x, y)
            size: Size of square in meters
            altitude: Flight altitude
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
                    x=x, y=y, z=altitude,
                    speed=0.5, tolerance=0.2
                ))
        
        return waypoints
    
    @staticmethod
    def create_circle_path(center: Tuple[float, float], radius: float,
                          altitude: float, num_points: int = 36) -> List[Waypoint]:
        """
        Create a circular path of waypoints
        
        Args:
            center: Center position (x, y)
            radius: Radius in meters
            altitude: Flight altitude
            num_points: Number of waypoints
            
        Returns:
            List of waypoints
        """
        waypoints = []
        
        for i in range(num_points):
            angle = 2 * np.pi * i / num_points
            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            
            # Face direction of motion
            next_angle = 2 * np.pi * ((i + 1) % num_points) / num_points
            yaw = np.arctan2(
                np.sin(next_angle) - np.sin(angle),
                np.cos(next_angle) - np.cos(angle)
            )
            
            waypoints.append(Waypoint(
                x=x, y=y, z=altitude, yaw=yaw,
                speed=0.3, tolerance=0.15
            ))
        
        return waypoints
    
    @staticmethod
    def create_spiral_path(center: Tuple[float, float], max_radius: float,
                          min_altitude: float, max_altitude: float,
                          revolutions: int = 3, points_per_rev: int = 36) -> List[Waypoint]:
        """
        Create a spiral path (ascending helix)
        
        Args:
            center: Center position (x, y)
            max_radius: Maximum radius in meters
            min_altitude: Starting altitude
            max_altitude: Ending altitude
            revolutions: Number of revolutions
            points_per_rev: Points per revolution
            
        Returns:
            List of waypoints
        """
        waypoints = []
        total_points = revolutions * points_per_rev
        
        for i in range(total_points):
            t = i / total_points
            angle = 2 * np.pi * revolutions * t
            radius = max_radius * t
            altitude = min_altitude + t * (max_altitude - min_altitude)
            
            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            
            waypoints.append(Waypoint(
                x=x, y=y, z=altitude,
                speed=0.4, tolerance=0.2
            ))
        
        return waypoints


# Example usage
if __name__ == "__main__":
    # Create controller
    config = {
        'max_velocity': 1.0,
        'max_yaw_rate': np.pi/3,
        'control_rate': 20,
        'position_kp': 1.2,
        'position_ki': 0.1,
        'position_kd': 0.3
    }
    
    controller = AutonomousController(config)
    
    # Create mission
    planner = MissionPlanner()
    
    # Square path
    waypoints = planner.create_square_path(
        center=(0, 0), size=2.0, altitude=1.0, points_per_side=5
    )
    
    controller.set_waypoints(waypoints)
    controller.set_mode(ControlMode.WAYPOINT)
    
    # Start controller
    controller.start()
    
    # Simulate state updates
    try:
        for i in range(100):
            # Simulated state from EKF
            state = {
                'x': i * 0.02,
                'y': 0.0,
                'z': 1.0,
                'yaw': 0.0
            }
            controller.update_state(state)
            
            # Get command
            command = controller.get_command()
            if command:
                print(f"Command: vx={command.vx:.2f}, vy={command.vy:.2f}, "
                      f"vz={command.vz:.2f}, yaw_rate={command.yaw_rate:.2f}")
            
            time.sleep(0.05)
            
            # Print statistics
            if i % 20 == 0:
                stats = controller.get_statistics()
                print(f"Stats: {stats}")
    
    except KeyboardInterrupt:
        print("Stopping...")
    
    finally:
        controller.stop()
