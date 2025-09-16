"""
RoboMaster 8-DOF Extended Kalman Filter Implementation
=====================================================
Following EXACT RoboMaster EKF Formulary specifications
As specified in RoboMaster_Formulary.pdf

State Vector (8-DOF):
[x, y, theta, vx, vy, bias_accel_x, bias_accel_y, bias_angular_velocity]

Where:
- x, y: Position in global frame (meters)
- theta: Yaw angle (radians)
- vx, vy: Velocity in global frame (m/s)
- bias_accel_x, bias_accel_y: Accelerometer biases (m/s²)
- bias_angular_velocity: Gyroscope bias (rad/s)

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
class RoboMasterState:
    """
    RoboMaster 8-DOF EKF state vector
    Following exact formulary specifications
    """
    # Position (2-DOF)
    x: float                    # meters (global X position)
    y: float                    # meters (global Y position)
    
    # Orientation (1-DOF)
    theta: float                # radians (yaw angle)
    
    # Velocity (2-DOF)
    vx: float                   # m/s (global X velocity)
    vy: float                   # m/s (global Y velocity)
    
    # Sensor biases (3-DOF)
    bias_accel_x: float         # m/s² (accelerometer X bias)
    bias_accel_y: float         # m/s² (accelerometer Y bias)
    bias_angular_velocity: float # rad/s (gyroscope Z bias)
    
    def to_array(self) -> np.ndarray:
        """Convert to numpy array following formulary order"""
        return np.array([
            self.x, self.y, self.theta,
            self.vx, self.vy,
            self.bias_accel_x, self.bias_accel_y, self.bias_angular_velocity
        ])
    
    @classmethod
    def from_array(cls, arr: np.ndarray) -> 'RoboMasterState':
        """Create from numpy array"""
        return cls(
            x=arr[0], y=arr[1], theta=arr[2],
            vx=arr[3], vy=arr[4],
            bias_accel_x=arr[5], bias_accel_y=arr[6], 
            bias_angular_velocity=arr[7]
        )
    
    def get_position(self) -> np.ndarray:
        """Get position vector [x, y]"""
        return np.array([self.x, self.y])
    
    def get_velocity(self) -> np.ndarray:
        """Get velocity vector [vx, vy]"""
        return np.array([self.vx, self.vy])
    
    def get_biases(self) -> np.ndarray:
        """Get bias vector [bias_accel_x, bias_accel_y, bias_angular_velocity]"""
        return np.array([self.bias_accel_x, self.bias_accel_y, self.bias_angular_velocity])
    
    def __repr__(self) -> str:
        return (f"RoboMasterState(pos=[{self.x:.2f}, {self.y:.2f}], "
                f"theta={np.degrees(self.theta):.1f}°, "
                f"vel=[{self.vx:.2f}, {self.vy:.2f}], "
                f"biases=[{self.bias_accel_x:.3f}, {self.bias_accel_y:.3f}, {self.bias_angular_velocity:.3f}])")


class RoboMasterEKF8DOF:
    """
    RoboMaster 8-DOF Extended Kalman Filter
    Implements exact formulary specifications from RoboMaster_Formulary.pdf
    
    State vector: [x, y, theta, vx, vy, bias_ax, bias_ay, bias_omega]
    """
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize RoboMaster 8-DOF EKF following formulary specifications
        
        Args:
            config: Configuration dictionary with noise parameters
        """
        self.n_states = 8
        config = config or {}
        
        # State vector initialization
        self.x = np.zeros(self.n_states)
        
        # Initial state covariance P (following formulary)
        self.P = np.eye(self.n_states)
        self.P[0:2, 0:2] *= config.get('init_pos_var', 0.5)      # Reduced from 1.0 - start with less position uncertainty
        self.P[2, 2] = config.get('init_theta_var', 0.05)         # Reduced from 0.1 - start with less orientation uncertainty  
        self.P[3:5, 3:5] *= config.get('init_vel_var', 0.2)      # Reduced from 0.5 - start with less velocity uncertainty
        self.P[5:7, 5:7] *= config.get('init_accel_bias_var', 0.005)  # Reduced from 0.01 - start with less accel bias uncertainty
        self.P[7, 7] = config.get('init_gyro_bias_var', 0.005)   # Reduced from 0.01 - start with less gyro bias uncertainty
        
        # Continuous-time process noise PSD parameters (following formulary)
        # q_accel: white acceleration PSD (per axis) [m^2/s^3]
        # q_gyro: white gyro noise PSD (yaw) [rad^2/s^3]
        # q_accel_bias: accel bias random-walk PSD [(m/s^2)^2/s]
        # q_gyro_bias: gyro bias random-walk PSD [(rad/s)^2/s] - increased for better observability
        self.q_accel = config.get('q_accel', 0.5)
        self.q_gyro = config.get('q_gyro', 0.01)
        self.q_accel_bias = config.get('q_accel_bias', 1e-6)
        self.q_gyro_bias = config.get('q_gyro_bias', 1e-5)  # Increased from 1e-8 for better bias learning
        
        # Measurement noise covariances R (following formulary)
        # Note: IMU is not used as a direct measurement update; kept for compatibility if needed externally
        self.R_accel = np.eye(2) * config.get('r_accel', 0.1)   # Accelerometer noise (2D)
        self.R_gyro = np.array([[config.get('r_gyro', 0.01)]])  # Gyroscope noise (1D)
        self.R_gps_pos = np.eye(2) * config.get('r_gps_pos', 1.0)     # GPS position noise
        self.R_gps_vel = np.eye(2) * config.get('r_gps_vel', 0.5)     # GPS velocity noise
        # Slightly reduce mag yaw influence (increase noise)
        self.R_yaw = np.array([[config.get('r_yaw', 1.0)]])           # Magnetometer-derived yaw noise
        self.R_nhc = np.array([[config.get('r_nhc', 0.1)]])           # Non-holonomic constraint noise
        self.R_zupt = np.eye(2) * config.get('r_zupt', 0.01)         # Zero-velocity update noise
        self.R_zaru = np.array([[config.get('r_zaru', 0.001)]])       # Zero-angular-rate update noise
        
        # Physical constants
        self.gravity = config.get('gravity', 9.81)  # m/s²
        
        # Statistics
        self.update_count = 0
        self.prediction_count = 0
        
        # Store last GPS velocity for course calculation
        self.last_gps_vel = None
        self.last_gps_time = None
        # Track last true yaw rate (omega_measured - bias) from predict
        self.last_omega_true = 0.0

        # GPS course yaw usage (configurable)
        self.enable_gps_course_yaw = bool(config.get('enable_gps_course_yaw', True))
        self.gps_course_yaw_speed_threshold = float(config.get('gps_course_yaw_speed_threshold', 1.0))
        self.gps_course_yaw_yaw_rate_threshold = float(config.get('gps_course_yaw_yaw_rate_threshold', 0.03))
        
        logger.info("RoboMaster 8-DOF EKF initialized following exact formulary specifications")
    
    def _validate_matrices(self, F: np.ndarray, P: np.ndarray, Qd: np.ndarray) -> bool:
        """
        Validate matrix dimensions and properties for consistency
        
        Args:
            F: State transition Jacobian
            P: State covariance matrix
            Qd: Discrete process noise matrix
            
        Returns:
            True if all matrices are valid, False otherwise
        """
        # Check dimensions
        expected_shape = (self.n_states, self.n_states)
        
        if F.shape != expected_shape:
            logger.error(f"Jacobian F has wrong dimensions: {F.shape}, expected {expected_shape}")
            return False
        
        if P.shape != expected_shape:
            logger.error(f"Covariance P has wrong dimensions: {P.shape}, expected {expected_shape}")
            return False
        
        if Qd.shape != expected_shape:
            logger.error(f"Process noise Qd has wrong dimensions: {Qd.shape}, expected {expected_shape}")
            return False
        
        # Check that P is symmetric
        if not np.allclose(P, P.T, atol=1e-10):
            logger.warning("Covariance P is not symmetric")
            return False
        
        # Check that Qd is symmetric
        if not np.allclose(Qd, Qd.T, atol=1e-10):
            logger.warning("Process noise Qd is not symmetric")
            return False
        
        # Check that diagonal elements are positive
        if np.any(np.diag(P) < 0):
            logger.error(f"Negative diagonal elements in P: {np.diag(P)[np.diag(P) < 0]}")
            return False
        
        if np.any(np.diag(Qd) < 0):
            logger.error(f"Negative diagonal elements in Qd: {np.diag(Qd)[np.diag(Qd) < 0]}")
            return False
        
        return True
    
    def _compute_discrete_process_noise(self, dt: float) -> np.ndarray:
        """
        Discretize continuous-time process noise (PSD) into discrete Qd for dt.
        State ordering: [x, y, theta, vx, vy, bias_ax, bias_ay, bias_omega]
        
        For constant-acceleration model with position-velocity coupling:
        Qd_posvel = [[(dt^3)/3*q_a, (dt^2)/2*q_a],
                      [(dt^2)/2*q_a, dt*q_a]]
        
        This follows the standard INS/GNSS process noise model.
        """
        Qd = np.zeros((self.n_states, self.n_states))
        dt2 = dt * dt
        dt3 = dt2 * dt
        q_a = self.q_accel
        
        # X-axis position-velocity coupling [pos x (0), vel x (3)]
        Qd[0, 0] = (dt3 / 3.0) * q_a      # Position variance
        Qd[0, 3] = (dt2 / 2.0) * q_a      # Position-velocity cross-covariance
        Qd[3, 0] = (dt2 / 2.0) * q_a      # Symmetric cross-covariance
        Qd[3, 3] = dt * q_a                # Velocity variance
        
        # Y-axis position-velocity coupling [pos y (1), vel y (4)]
        Qd[1, 1] = (dt3 / 3.0) * q_a      # Position variance
        Qd[1, 4] = (dt2 / 2.0) * q_a      # Position-velocity cross-covariance
        Qd[4, 1] = (dt2 / 2.0) * q_a      # Symmetric cross-covariance
        Qd[4, 4] = dt * q_a                # Velocity variance
        
        # Theta (yaw) driven by gyro white noise
        Qd[2, 2] = dt * self.q_gyro
        
        # Accelerometer bias random walk (independent)
        Qd[5, 5] = dt * self.q_accel_bias
        Qd[6, 6] = dt * self.q_accel_bias
        
        # Gyro bias random walk (independent)
        Qd[7, 7] = dt * self.q_gyro_bias
        
        # Validate matrix properties
        if not np.allclose(Qd, Qd.T):
            logger.warning("Process noise matrix Qd is not symmetric")
            Qd = (Qd + Qd.T) / 2  # Force symmetry
        
        # Check for negative diagonal elements (should not happen)
        diag_negative = np.diag(Qd) < 0
        if np.any(diag_negative):
            logger.error(f"Negative diagonal elements in Qd: {np.diag(Qd)[diag_negative]}")
            # Clip to small positive values
            Qd[diag_negative, diag_negative] = 1e-12
        
        return Qd
    
    def predict(self, dt: float, control_input: Optional[np.ndarray] = None):
        """
        Prediction step following RoboMaster formulary motion model
        
        Args:
            dt: Time step (seconds)
            control_input: Optional control inputs [ax_control, ay_control, omega_control]
        """
        if dt <= 0:
            logger.warning(f"Invalid dt: {dt}. Skipping prediction.")
            return
        # Clamp dt to avoid spikes
        dt = float(max(1e-3, min(dt, 0.1)))
        
        # Store current state for Jacobian computation
        x_prev, y_prev, theta_prev = self.x[0:3]
        vx_prev, vy_prev = self.x[3:5]
        bias_ax, bias_ay, bias_omega = self.x[5:8]
        
        # Compute state transition Jacobian BEFORE state update
        F = self._compute_state_jacobian(dt, control_input, x_prev, y_prev, theta_prev, vx_prev, vy_prev, bias_ax, bias_ay, bias_omega)
        
        # Apply state transition model: x_k+1 = F * x_k + B * u_k
        # For our case, we'll implement the motion model directly and then validate with F
        
        # Position update: p_k+1 = p_k + v_k * dt
        self.x[0] = x_prev + vx_prev * dt     # x += vx * dt
        self.x[1] = y_prev + vy_prev * dt     # y += vy * dt
        
        # Orientation update: theta_k+1 = theta_k + omega * dt
        if control_input is not None and len(control_input) >= 3:
            omega_measured = control_input[2]
            omega_true = omega_measured - bias_omega
            self.x[2] = theta_prev + omega_true * dt
            # Store last true yaw rate for constraints
            try:
                self.last_omega_true = float(omega_true)
            except Exception:
                self.last_omega_true = 0.0
        # If no control input, orientation remains constant
        
        # Velocity update with control input
        if control_input is not None and len(control_input) >= 2:
            ax_measured, ay_measured = control_input[0:2]
            # Remove bias and apply in global frame
            ax_true = ax_measured - bias_ax
            ay_true = ay_measured - bias_ay
            
            # Transform acceleration from body to global frame
            cos_theta = np.cos(theta_prev)
            sin_theta = np.sin(theta_prev)
            
            ax_global = ax_true * cos_theta - ay_true * sin_theta
            ay_global = ax_true * sin_theta + ay_true * cos_theta
            
            self.x[3] = vx_prev + ax_global * dt  # vx += ax_global * dt
            self.x[4] = vy_prev + ay_global * dt  # vy += ay_global * dt
        
        # Biases remain constant (random walk model)
        # bias_k+1 = bias_k (handled by process noise)
        
        # Normalize angle
        self.x[2] = self._normalize_angle(self.x[2])
        
        # Compute discrete process noise
        Qd = self._compute_discrete_process_noise(dt)
        
        # Validate all matrices before covariance prediction
        if not self._validate_matrices(F, self.P, Qd):
            logger.error("Matrix validation failed, skipping covariance prediction")
            return
        
        # Apply covariance prediction: P_k+1 = F * P_k * F^T + Qd
        self.P = F @ self.P @ F.T + Qd
        
        # Ensure covariance remains symmetric and positive definite
        self.P = (self.P + self.P.T) / 2  # Force symmetry
        
        # Final validation check
        if not self._validate_matrices(F, self.P, Qd):
            logger.error("Matrix validation failed after covariance prediction")
            return
        
        self.prediction_count += 1
        
        logger.debug(f"RoboMaster prediction completed (dt={dt:.3f}s)")
    
    def _compute_state_jacobian(self, dt: float, control_input: Optional[np.ndarray],
                                x: float, y: float, theta: float,
                                vx: float, vy: float,
                                bias_ax: float, bias_ay: float, bias_omega: float) -> np.ndarray:
        """
        Compute state transition Jacobian F following formulary
        
        State: [x, y, theta, vx, vy, bias_ax, bias_ay, bias_omega]
        
        Args:
            dt: Time step
            control_input: Control inputs [ax, ay, omega]
            x, y, theta: Current position and orientation
            vx, vy: Current velocity
            bias_ax, bias_ay, bias_omega: Current biases
        """
        F = np.eye(self.n_states)
        
        # Position derivatives: ∂x/∂vx, ∂y/∂vy
        F[0, 3] = dt  # ∂x/∂vx
        F[1, 4] = dt  # ∂y/∂vy
        
        # Orientation derivatives (if angular velocity provided)
        if control_input is not None and len(control_input) >= 3:
            F[2, 7] = -dt  # ∂theta/∂bias_omega (negative because omega_true = omega_meas - bias)
        
        # Velocity derivatives (if acceleration provided)
        if control_input is not None and len(control_input) >= 2:
            ax_meas, ay_meas = control_input[0:2]
            
            # Use passed biases
            ax_true = ax_meas - bias_ax
            ay_true = ay_meas - bias_ay
            
            cos_theta = np.cos(theta)
            sin_theta = np.sin(theta)
            
            # ∂vx/∂theta = dt * (-ax_true * sin_theta - ay_true * cos_theta)
            F[3, 2] = dt * (-ax_true * sin_theta - ay_true * cos_theta)
            
            # ∂vy/∂theta = dt * (ax_true * cos_theta - ay_true * sin_theta)
            F[4, 2] = dt * (ax_true * cos_theta - ay_true * sin_theta)
            
            # ∂vx/∂bias_ax = -dt * cos_theta
            F[3, 5] = -dt * cos_theta
            
            # ∂vx/∂bias_ay = dt * sin_theta
            F[3, 6] = dt * sin_theta
            
            # ∂vy/∂bias_ax = -dt * sin_theta
            F[4, 5] = -dt * sin_theta
            
            # ∂vy/∂bias_ay = -dt * cos_theta
            F[4, 6] = -dt * cos_theta
        
        return F
    
    def update_imu(self, accel_body: np.ndarray, gyro_z: float):
        """
        Deprecated: IMU should be used only in the prediction step, not as a measurement.
        Kept for backward compatibility but performs no update.
        """
        logger.debug("IMU measurement update is disabled to maintain independence from prediction inputs")

    def update_yaw(self, yaw_measurement: float):
        """
        Update orientation (theta) using an external yaw measurement (e.g., magnetometer-derived).
        Args:
            yaw_measurement: Measured yaw angle in radians (wrapped to [-pi, pi])
        """
        z = np.array([yaw_measurement])
        h = np.array([self.x[2]])
        H = np.zeros((1, self.n_states))
        H[0, 2] = 1.0
        self._kalman_update(z, h, H, self.R_yaw)
        logger.debug(f"Yaw update with measurement={yaw_measurement:.3f} rad")

    def update_non_holonomic_constraint(self, speed_threshold: float = 0.5, yaw_rate_threshold: float = 0.1):
        """
        Non-holonomic constraint update: lateral body velocity should be ~0 for ground vehicles.
        Makes theta observable from vx, vy when moving forward.
        Args:
            speed_threshold: Minimum speed to apply NHC (m/s)
            yaw_rate_threshold: Maximum yaw rate to apply NHC (rad/s)
        """
        vx, vy = self.x[3:5]
        theta = self.x[2]
        speed = np.sqrt(vx**2 + vy**2)
        
        # Only apply when moving forward and not turning sharply
        if speed > speed_threshold and abs(self.last_omega_true) < yaw_rate_threshold:
            # Lateral velocity in body frame: -sin(theta)*vx + cos(theta)*vy ≈ 0
            z = np.array([0.0])
            h = np.array([-np.sin(theta) * vx + np.cos(theta) * vy])
            
            # Jacobian: H = [0, 0, (-cos(theta)*vx - sin(theta)*vy), -sin(theta), cos(theta), 0, 0, 0]
            H = np.zeros((1, self.n_states))
            H[0, 2] = -np.cos(theta) * vx - np.sin(theta) * vy  # ∂h/∂theta
            H[0, 3] = -np.sin(theta)                            # ∂h/∂vx
            H[0, 4] = np.cos(theta)                             # ∂h/∂vy
            
            # Only apply if the theta derivative is significant
            theta_derivative = abs(-np.cos(theta) * vx - np.sin(theta) * vy)
            if theta_derivative > 0.1:  # Only apply if theta is observable
                self._kalman_update(z, h, H, self.R_nhc)
                logger.debug(f"NHC update applied: speed={speed:.2f} m/s, lateral_vel={h[0]:.3f} m/s, theta_deriv={theta_derivative:.3f}")
            else:
                logger.debug(f"NHC skipped: theta not observable (derivative={theta_derivative:.3f})")
        else:
            logger.debug(f"NHC skipped: speed={speed:.2f} m/s, yaw_rate={abs(self.x[7]):.3f} rad/s")

    def update_zero_velocity(self, speed_threshold: float = 0.1):
        """
        Zero-velocity update (ZUPT): when speed is very low, measure vx=vy=0.
        Helps stabilize velocity and accelerometer biases.
        Args:
            speed_threshold: Speed threshold to trigger ZUPT (m/s)
        """
        vx, vy = self.x[3:5]
        speed = np.sqrt(vx**2 + vy**2)
        
        if speed < speed_threshold:
            z = np.array([0.0, 0.0])  # [vx=0, vy=0]
            h = np.array([vx, vy])     # Current velocity estimate
            
            H = np.zeros((2, self.n_states))
            H[0, 3] = 1.0  # ∂vx/∂vx
            H[1, 4] = 1.0  # ∂vy/∂vy
            
            self._kalman_update(z, h, H, self.R_zupt)
            logger.debug(f"ZUPT applied: speed={speed:.3f} m/s")

    def update_zero_angular_rate(self, angular_rate_threshold: float = 0.05):
        """
        Zero-angular-rate update (ZARU): when angular rate is very low, measure omega=0.
        Directly tightens gyroscope bias estimate.
        Args:
            angular_rate_threshold: Angular rate threshold to trigger ZARU (rad/s)
        """
        # Use last true yaw rate (omega_measured - bias) estimated in predict
        omega_true = float(self.last_omega_true)
        if abs(omega_true) < angular_rate_threshold:
            # Measurement: omega_true ≈ 0
            z = np.array([0.0])
            h = np.array([omega_true])
            # h = omega_measured - bias, with omega_measured treated as constant; ∂h/∂bias = -1
            H = np.zeros((1, self.n_states))
            H[0, 7] = -1.0
            self._kalman_update(z, h, H, self.R_zaru)
            logger.debug(f"ZARU applied: omega_true={omega_true:.4f} rad/s")
    
    def update_gps_position(self, gps_pos: np.ndarray):
        """
        Update with GPS position measurements
        
        Args:
            gps_pos: GPS position [x, y] in global frame (meters)
        """
        # Measurement vector
        z = gps_pos
        
        # Expected measurement (current position)
        h = self.x[0:2]
        
        # Measurement Jacobian
        H = np.zeros((2, self.n_states))
        H[0:2, 0:2] = np.eye(2)  # ∂pos/∂pos = I
        
        # Kalman update
        self._kalman_update(z, h, H, self.R_gps_pos)
        
        logger.debug(f"GPS position update: {gps_pos}")
    
    def update_gps_velocity(self, gps_vel: np.ndarray):
        """
        Update with GPS velocity measurements
        
        Args:
            gps_vel: GPS velocity [vx, vy] in global frame (m/s)
        """
        # Measurement vector
        z = gps_vel
        
        # Expected measurement (current velocity)
        h = self.x[3:5]
        
        # Measurement Jacobian
        H = np.zeros((2, self.n_states))
        H[0:2, 3:5] = np.eye(2)  # ∂vel/∂vel = I
        
        # Kalman update
        self._kalman_update(z, h, H, self.R_gps_vel)
        
        # Store GPS velocity for course calculation
        self.last_gps_vel = gps_vel.copy()
        self.last_gps_time = time.time()
        
        logger.debug(f"GPS velocity update: {gps_vel}")
    
    def update_gps_course_yaw(self, speed_threshold: float = 1.0, yaw_rate_threshold: float = 0.03):
        """
        Update yaw using GPS course when speed is sufficient.
        This provides absolute heading measurements, solving the yaw observability issue.
        
        Args:
            speed_threshold: Minimum speed to use GPS course (m/s)
        """
        if self.last_gps_vel is None:
            return
        
        # Calculate speed from GPS velocity
        speed = np.linalg.norm(self.last_gps_vel)
        
        # Only use GPS course yaw when moving sufficiently fast AND not turning
        if speed > speed_threshold and abs(self.last_omega_true) < yaw_rate_threshold:
            # Calculate course angle from GPS velocity
            vx_gps, vy_gps = self.last_gps_vel
            yaw_course = np.arctan2(vy_gps, vx_gps)
            
            # Update yaw with GPS course measurement
            self.update_yaw(yaw_course)
            
            logger.debug(f"GPS course yaw update: {np.degrees(yaw_course):.1f}° (speed: {speed:.2f} m/s, |omega|={abs(self.last_omega_true):.3f})")
        else:
            logger.debug(f"GPS course yaw gated: speed={speed:.2f} m/s, |omega|={abs(self.last_omega_true):.3f}")
    
    def update_magnetometer_yaw(self, mag_vector: np.ndarray):
        """
        Update yaw using magnetometer measurements.
        This provides absolute heading measurements for yaw observability.
        
        Args:
            mag_vector: Magnetometer vector [mx, my, mz] in body frame
        """
        if len(mag_vector) < 2:
            return
        
        # Calculate yaw from magnetometer (assuming phone is level)
        # For a level phone, yaw = atan2(my, mx) in body frame
        # This gives heading relative to magnetic north
        mx, my = mag_vector[0], mag_vector[1]
        
        # Apply calibration if available (simplified for now)
        # In practice, you'd want to calibrate for hard/soft iron effects
        yaw_mag = np.arctan2(my, mx)
        
        # Update yaw with magnetometer measurement
        self.update_yaw(yaw_mag)
        
        logger.debug(f"Magnetometer yaw update: {np.degrees(yaw_mag):.1f}°")
    
    def apply_constraint_updates(self, speed_threshold: float = 0.1, yaw_rate_threshold: float = 0.05):
        """
        Apply all constraint-based updates automatically.
        This includes NHC, ZUPT, and ZARU updates for robust yaw estimation.
        
        Args:
            speed_threshold: Speed threshold for ZUPT (m/s)
            yaw_rate_threshold: Angular rate threshold for ZARU (rad/s)
        """
        # Apply non-holonomic constraint when moving
        self.update_non_holonomic_constraint()
        
        # Apply zero-velocity update when stopped
        self.update_zero_velocity(speed_threshold)
        
        # Apply zero-angular-rate update when not rotating
        self.update_zero_angular_rate(yaw_rate_threshold)
        
        # Apply GPS course yaw update if enabled and GPS velocity is available
        if self.enable_gps_course_yaw:
            self.update_gps_course_yaw(
                speed_threshold=self.gps_course_yaw_speed_threshold,
                yaw_rate_threshold=self.gps_course_yaw_yaw_rate_threshold,
            )
        
        logger.debug("Applied all constraint updates")
    
    def _kalman_update(self, z: np.ndarray, h: np.ndarray, H: np.ndarray, R: np.ndarray):
        """
        Standard Kalman update following formulary equations
        
        Args:
            z: Measurement vector
            h: Expected measurement (h(x))
            H: Measurement Jacobian
            R: Measurement noise covariance
        """
        # Validate input dimensions
        if z.shape[0] != h.shape[0]:
            logger.error(f"Measurement dimension mismatch: z={z.shape}, h={h.shape}")
            return
        
        if H.shape[0] != z.shape[0] or H.shape[1] != self.n_states:
            logger.error(f"Jacobian dimension mismatch: H={H.shape}, expected ({z.shape[0]}, {self.n_states})")
            return
        
        if R.shape[0] != z.shape[0] or R.shape[1] != z.shape[0]:
            logger.error(f"Measurement noise dimension mismatch: R={R.shape}, expected ({z.shape[0]}, {z.shape[0]})")
            return
        
        # Innovation
        y = z - h
        
        # Handle angle wrapping for orientation measurements
        if H.shape[0] == 1 and H[0, 2] != 0:  # Orientation measurement
            y[0] = self._normalize_angle(y[0])
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Validate innovation covariance
        if not np.allclose(S, S.T, atol=1e-10):
            logger.warning("Innovation covariance S is not symmetric")
            S = (S + S.T) / 2
        
        # Check for positive definiteness
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            logger.warning("Singular innovation covariance matrix S")
            return
        
        # Kalman gain
        K = self.P @ H.T @ S_inv
        
        # Update state
        self.x = self.x + K @ y
        
        # Normalize angle
        self.x[2] = self._normalize_angle(self.x[2])
        
        # Update covariance (Joseph form for numerical stability)
        I_KH = np.eye(self.n_states) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        
        # Ensure covariance remains symmetric and positive definite
        self.P = (self.P + self.P.T) / 2
        
        # Validate updated covariance
        if not self._validate_matrices(np.eye(self.n_states), self.P, np.zeros((self.n_states, self.n_states))):
            logger.warning("Covariance validation failed after update")
        
        self.update_count += 1
    
    def get_state(self) -> RoboMasterState:
        """Get current state as RoboMasterState object"""
        return RoboMasterState.from_array(self.x)
    
    def get_covariance(self) -> np.ndarray:
        """Get current covariance matrix"""
        return self.P.copy()
    
    def get_position_uncertainty(self) -> np.ndarray:
        """Get position uncertainty (standard deviation)"""
        return np.sqrt(np.diag(self.P[0:2, 0:2]))
    
    def get_velocity_uncertainty(self) -> np.ndarray:
        """Get velocity uncertainty (standard deviation)"""
        return np.sqrt(np.diag(self.P[3:5, 3:5]))
    
    def get_bias_uncertainty(self) -> np.ndarray:
        """Get bias uncertainty (standard deviation)"""
        return np.sqrt(np.diag(self.P[5:8, 5:8]))
    
    def reset(self, initial_state: Optional[RoboMasterState] = None):
        """
        Reset filter to initial conditions
        
        Args:
            initial_state: Optional initial state
        """
        if initial_state:
            self.x = initial_state.to_array()
        else:
            self.x = np.zeros(self.n_states)
        
        # Reset covariance
        self.P = np.eye(self.n_states)
        self.P[0:2, 0:2] *= 1.0    # Position uncertainty
        self.P[2, 2] = 0.1         # Orientation uncertainty
        self.P[3:5, 3:5] *= 0.5    # Velocity uncertainty
        self.P[5:7, 5:7] *= 0.01   # Accel bias uncertainty
        self.P[7, 7] = 0.01        # Gyro bias uncertainty - increased for better observability
        
        # Reset statistics
        self.update_count = 0
        self.prediction_count = 0
        
        logger.info("RoboMaster EKF reset")
    
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
            'velocity': self.x[3:5].tolist(),
            'orientation_deg': np.degrees(self.x[2]),
            'biases': self.x[5:8].tolist(),
            'position_uncertainty': self.get_position_uncertainty().tolist(),
            'velocity_uncertainty': self.get_velocity_uncertainty().tolist(),
            'bias_uncertainty': self.get_bias_uncertainty().tolist()
        }


# Example usage and testing
if __name__ == "__main__":
    # Create RoboMaster EKF with configuration
    config = {
        'q_accel': 0.5,
        'q_gyro': 0.01,
        'q_accel_bias': 1e-6,
        'q_gyro_bias': 1e-5,  # Increased for better bias learning
        'r_accel': 0.1,
        'r_gyro': 0.01,
        'r_gps_pos': 1.0,
        'r_gps_vel': 0.5,
        'r_yaw': 0.5,
        'r_nhc': 0.1,
        'r_zupt': 0.01,
        'r_zaru': 0.001
    }
    
    ekf = RoboMasterEKF8DOF(config)
    
    # Simulate robot motion
    dt = 0.02  # 50 Hz
    
    for i in range(200):
        # Simulated control input [ax, ay, omega]
        control = np.array([1.0, 0.1, 0.05])  # Accelerating forward with slight turn
        
        # Prediction
        ekf.predict(dt, control)
        
        # Apply NHC, ZUPT, ZARU updates
        ekf.apply_constraint_updates()
        
        # Simulated GPS updates every 10 steps
        if i % 10 == 0:
            # Simulated GPS position
            true_pos = np.array([i * 0.02, 0.5 * np.sin(i * 0.01)])
            gps_pos = true_pos + np.random.randn(2) * 0.1
            ekf.update_gps_position(gps_pos)
            
            # Simulated GPS velocity
            true_vel = np.array([1.0, 0.5 * 0.01 * np.cos(i * 0.01)])
            gps_vel = true_vel + np.random.randn(2) * 0.05
            ekf.update_gps_velocity(gps_vel)
        
        # Print state every 50 iterations
        if i % 50 == 0:
            state = ekf.get_state()
            print(f"Iteration {i}: {state}")
    
    # Final statistics
    stats = ekf.get_statistics()
    print(f"\nFinal statistics: {stats}")


