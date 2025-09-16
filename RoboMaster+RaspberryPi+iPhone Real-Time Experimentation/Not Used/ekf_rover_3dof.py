"""
3-DOF Extended Kalman Filter for Ground Rover Navigation
========================================================
Simplified EKF for ground-based vehicles following RoboMaster conventions
Optimized for 2D navigation with minimal computational overhead

State Vector (3-DOF): [x, y, theta]
- x: position in global X (meters)
- y: position in global Y (meters) 
- theta: heading angle (radians)

Designed for iPhone sensor integration with RoboMaster S1 rover mode
Author: RoboMaster EKF Integration System
Date: 2025
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional, Dict, Any
import logging
import time

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class EKFRoverState:
    """3-DOF EKF state for ground rover navigation"""
    x: float        # meters (global X position)
    y: float        # meters (global Y position)
    theta: float    # radians (heading angle)
    
    def to_array(self) -> np.ndarray:
        """Convert to numpy array"""
        return np.array([self.x, self.y, self.theta])
    
    @classmethod
    def from_array(cls, arr: np.ndarray) -> 'EKFRoverState':
        """Create from numpy array"""
        return cls(x=arr[0], y=arr[1], theta=arr[2])
    
    def __repr__(self) -> str:
        return (f"RoverState(pos=[{self.x:.2f}, {self.y:.2f}], "
                f"heading={np.degrees(self.theta):.1f}°)")


class EKFRover3DOF:
    """
    3-DOF Extended Kalman Filter for ground rover navigation
    Optimized for computational efficiency and iPhone sensor integration
    """
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """Initialize 3-DOF rover EKF"""
        self.n_states = 3
        
        # State vector: [x, y, theta]
        self.x = np.zeros(self.n_states)
        
        # Initial covariance
        self.P = np.eye(self.n_states)
        self.P[0:2, 0:2] *= 1.0    # Position uncertainty (1m)
        self.P[2, 2] = 0.1         # Heading uncertainty (~6°)
        
        # Process noise (from config or defaults)
        config = config or {}
        self.Q = np.diag([
            config.get('q_position', 0.01),    # Position process noise
            config.get('q_position', 0.01),    # Position process noise  
            config.get('q_heading', 0.05)      # Heading process noise
        ])
        
        # Measurement noise
        self.R_compass = np.array([[config.get('r_compass', 0.1)]])  # Compass/magnetometer
        self.R_gps = np.eye(2) * config.get('r_gps', 1.0)           # GPS position
        
        # Motion model parameters
        self.wheel_base = config.get('wheel_base', 0.3)  # meters (RoboMaster S1)
        self.max_speed = config.get('max_speed', 2.0)    # m/s
        
        # Statistics
        self.update_count = 0
        self.prediction_count = 0
        
        logger.info("3-DOF Rover EKF initialized for ground navigation")
    
    def predict(self, dt: float, velocity: float = 0.0, steering_angle: float = 0.0):
        """
        Prediction step using bicycle/Ackermann model
        
        Args:
            dt: Time step (seconds)
            velocity: Forward velocity (m/s)
            steering_angle: Steering angle (radians)
        """
        if dt <= 0:
            logger.warning(f"Invalid dt: {dt}")
            return
        
        # Current state
        x, y, theta = self.x
        
        # Bicycle model kinematics
        if abs(velocity) > 1e-6:
            # Moving: use bicycle model
            angular_velocity = velocity * np.tan(steering_angle) / self.wheel_base
            
            if abs(angular_velocity) > 1e-6:
                # Turning motion
                radius = velocity / angular_velocity
                
                # State update (exact solution for constant curvature)
                dtheta = angular_velocity * dt
                dx = radius * (np.sin(theta + dtheta) - np.sin(theta))
                dy = radius * (-np.cos(theta + dtheta) + np.cos(theta))
                
                self.x[0] += dx
                self.x[1] += dy
                self.x[2] += dtheta
            else:
                # Straight line motion
                self.x[0] += velocity * np.cos(theta) * dt
                self.x[1] += velocity * np.sin(theta) * dt
        
        # Normalize heading angle
        self.x[2] = self._normalize_angle(self.x[2])
        
        # Jacobian of motion model
        F = self._compute_motion_jacobian(dt, velocity, steering_angle)
        
        # Predict covariance: P = F * P * F^T + Q
        self.P = F @ self.P @ F.T + self.Q * dt
        
        self.prediction_count += 1
        logger.debug(f"Rover prediction: v={velocity:.2f}, δ={np.degrees(steering_angle):.1f}°")
    
    def _compute_motion_jacobian(self, dt: float, velocity: float, steering_angle: float) -> np.ndarray:
        """Compute Jacobian of motion model for covariance prediction"""
        F = np.eye(self.n_states)
        
        if abs(velocity) > 1e-6:
            theta = self.x[2]
            angular_velocity = velocity * np.tan(steering_angle) / self.wheel_base
            
            if abs(angular_velocity) > 1e-6:
                # Turning motion Jacobian
                radius = velocity / angular_velocity
                dtheta = angular_velocity * dt
                
                # ∂x/∂θ
                F[0, 2] = radius * (np.cos(theta + dtheta) - np.cos(theta))
                # ∂y/∂θ  
                F[1, 2] = radius * (np.sin(theta + dtheta) - np.sin(theta))
            else:
                # Straight motion Jacobian
                F[0, 2] = -velocity * np.sin(theta) * dt
                F[1, 2] = velocity * np.cos(theta) * dt
        
        return F
    
    def update_compass(self, heading_measurement: float):
        """
        Update with compass/magnetometer heading measurement
        
        Args:
            heading_measurement: Measured heading in radians
        """
        # Measurement vector
        z = np.array([heading_measurement])
        
        # Expected measurement (current heading)
        h = np.array([self.x[2]])
        
        # Measurement Jacobian
        H = np.zeros((1, self.n_states))
        H[0, 2] = 1.0  # ∂heading/∂theta
        
        # Kalman update
        self._kalman_update(z, h, H, self.R_compass, angle_measurement=True)
        
        logger.debug(f"Compass update: measured={np.degrees(heading_measurement):.1f}°")
    
    def update_gps(self, x_gps: float, y_gps: float):
        """
        Update with GPS position measurement
        
        Args:
            x_gps: GPS X position (meters)
            y_gps: GPS Y position (meters)
        """
        # Measurement vector
        z = np.array([x_gps, y_gps])
        
        # Expected measurement (current position)
        h = self.x[0:2]
        
        # Measurement Jacobian
        H = np.zeros((2, self.n_states))
        H[0:2, 0:2] = np.eye(2)
        
        # Kalman update
        self._kalman_update(z, h, H, self.R_gps)
        
        logger.debug(f"GPS update: pos=({x_gps:.2f}, {y_gps:.2f})")
    
    def update_imu_heading(self, accel: np.ndarray, mag: np.ndarray):
        """
        Extract heading from accelerometer + magnetometer
        
        Args:
            accel: Accelerometer [ax, ay, az] in m/s²
            mag: Magnetometer [mx, my, mz] in μT
        """
        # Tilt compensation using accelerometer
        ax, ay, az = accel
        norm_a = np.linalg.norm(accel)
        
        if norm_a > 0:
            # Normalize accelerometer
            ax, ay, az = accel / norm_a
            
            # Calculate tilt angles
            roll = np.arctan2(ay, az)
            pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))
            
            # Tilt-compensated magnetometer
            mx, my, mz = mag
            
            # Rotate magnetometer readings to horizontal plane
            mx_h = mx * np.cos(pitch) + mz * np.sin(pitch)
            my_h = (mx * np.sin(roll) * np.sin(pitch) + 
                    my * np.cos(roll) - 
                    mz * np.sin(roll) * np.cos(pitch))
            
            # Calculate tilt-compensated heading
            heading = np.arctan2(my_h, mx_h)
            
            # Update with compass measurement
            self.update_compass(heading)
        else:
            logger.warning("Invalid accelerometer data for tilt compensation")
    
    def _kalman_update(self, z: np.ndarray, h: np.ndarray, H: np.ndarray, 
                       R: np.ndarray, angle_measurement: bool = False):
        """Standard Kalman update with angle wrapping support"""
        # Innovation
        y = z - h
        
        # Handle angle wrapping for heading measurements
        if angle_measurement and len(y) == 1:
            y[0] = self._normalize_angle(y[0])
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            logger.warning("Singular matrix in Kalman gain calculation")
            return
        
        # Update state
        self.x = self.x + K @ y
        
        # Normalize heading
        self.x[2] = self._normalize_angle(self.x[2])
        
        # Update covariance (Joseph form)
        I_KH = np.eye(self.n_states) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        
        self.update_count += 1
    
    def get_state(self) -> EKFRoverState:
        """Get current state as EKFRoverState object"""
        return EKFRoverState.from_array(self.x)
    
    def get_covariance(self) -> np.ndarray:
        """Get current covariance matrix"""
        return self.P.copy()
    
    def get_position_uncertainty(self) -> np.ndarray:
        """Get position uncertainty (standard deviation)"""
        return np.sqrt(np.diag(self.P[0:2, 0:2]))
    
    def get_heading_uncertainty(self) -> float:
        """Get heading uncertainty (standard deviation in radians)"""
        return np.sqrt(self.P[2, 2])
    
    def reset(self, initial_state: Optional[EKFRoverState] = None):
        """Reset filter to initial conditions"""
        if initial_state:
            self.x = initial_state.to_array()
        else:
            self.x = np.zeros(self.n_states)
        
        # Reset covariance
        self.P = np.eye(self.n_states)
        self.P[0:2, 0:2] *= 1.0
        self.P[2, 2] = 0.1
        
        # Reset statistics
        self.update_count = 0
        self.prediction_count = 0
        
        logger.info("Rover EKF reset")
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-π, π]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get filter statistics"""
        return {
            'update_count': self.update_count,
            'prediction_count': self.prediction_count,
            'covariance_trace': np.trace(self.P),
            'position': self.x[0:2].tolist(),
            'heading_deg': np.degrees(self.x[2]),
            'position_uncertainty': self.get_position_uncertainty().tolist(),
            'heading_uncertainty_deg': np.degrees(self.get_heading_uncertainty())
        }


# Example usage
if __name__ == "__main__":
    # Create rover EKF
    config = {
        'q_position': 0.01,
        'q_heading': 0.02,
        'r_compass': 0.1,
        'r_gps': 1.0,
        'wheel_base': 0.3
    }
    
    ekf = EKFRover3DOF(config)
    
    # Simulate rover motion
    dt = 0.1  # 10 Hz
    
    for i in range(50):
        # Prediction with motion
        velocity = 1.0  # 1 m/s forward
        steering = 0.1 * np.sin(i * 0.2)  # Sinusoidal steering
        
        ekf.predict(dt, velocity, steering)
        
        # Simulated measurements every 5 steps
        if i % 5 == 0:
            # Simulated GPS
            true_x = i * 0.1
            true_y = 0.5 * np.sin(i * 0.1)
            ekf.update_gps(true_x + np.random.normal(0, 0.1), 
                          true_y + np.random.normal(0, 0.1))
            
            # Simulated compass
            true_heading = 0.1 * np.cos(i * 0.1)
            ekf.update_compass(true_heading + np.random.normal(0, 0.05))
        
        # Print state every 10 steps
        if i % 10 == 0:
            state = ekf.get_state()
            print(f"Step {i}: {state}")
    
    # Final statistics
    stats = ekf.get_statistics()
    print(f"\nFinal statistics: {stats}")
