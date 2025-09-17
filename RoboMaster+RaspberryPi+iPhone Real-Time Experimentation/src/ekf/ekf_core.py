"""
Extended Kalman Filter for RoboMaster S1 state estimation.

State Vector: [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz] (12D)
Sensors: IMU (accelerometer, gyroscope), GPS, Barometer, Magnetometer
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple
import time

@dataclass
class SensorData:
    """Container for all sensor measurements"""
    timestamp: float
    # IMU data (body frame)
    accel: np.ndarray  # [ax, ay, az] in m/s²
    gyro: np.ndarray   # [wx, wy, wz] in rad/s
    # Magnetometer (body frame)
    mag: np.ndarray    # [mx, my, mz] in μT
    # Barometer (if available)
    pressure: Optional[float] = None  # Pa
    altitude: Optional[float] = None  # m
    # GPS (if available) - NED frame
    gps_lat: Optional[float] = None   # degrees
    gps_lon: Optional[float] = None   # degrees
    gps_alt: Optional[float] = None   # m
    # Chassis position from RoboMaster (relative)
    chassis_x: Optional[float] = None  # m
    chassis_y: Optional[float] = None  # m
    chassis_yaw: Optional[float] = None  # rad

@dataclass
class EKFState:
    """
    EKF state vector: [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
    All quantities in NED (North-East-Down) coordinate frame
    """
    position: np.ndarray     # [x, y, z] in meters (NED)
    velocity: np.ndarray     # [vx, vy, vz] in m/s (NED)
    orientation: np.ndarray  # [roll, pitch, yaw] in radians
    angular_velocity: np.ndarray  # [wx, wy, wz] in rad/s (body frame)
    
    def to_vector(self) -> np.ndarray:
        """Convert state to 12D vector"""
        return np.concatenate([
            self.position, 
            self.velocity, 
            self.orientation, 
            self.angular_velocity
        ])
    
    @classmethod
    def from_vector(cls, vec: np.ndarray) -> 'EKFState':
        """Create state from 12D vector"""
        return cls(
            position=vec[0:3],
            velocity=vec[3:6],
            orientation=vec[6:9],
            angular_velocity=vec[9:12]
        )

class ExtendedKalmanFilter:
    """
    Extended Kalman Filter for RoboMaster S1 state estimation
    State: [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz] (12D)
    Coordinate Frame: NED (North-East-Down)
    """
    
    def __init__(self):
        self.n_states = 12
        self.x = np.zeros(self.n_states)
        self.P = np.eye(self.n_states) * 1.0
        
        self.Q = self._create_process_noise_matrix()
        self.R_imu = np.eye(6) * 0.1  # IMU: [ax, ay, az, wx, wy, wz]
        self.R_mag = np.eye(3) * 0.5  # Magnetometer: [mx, my, mz]
        self.R_baro = np.array([[0.1]])  # Barometer: [altitude]
        self.R_gps = np.eye(3) * 1.0   # GPS: [x, y, z]
        self.R_chassis = np.eye(3) * 0.05  # Chassis: [x, y, yaw]
        
        # Physical constants as per formulary
        self.gravity = 9.81  # m/s²
        self.mag_declination = 0.0  # Magnetic declination (to be calibrated)
        self.yaw_mount_offset = 0.0  # Static mounting yaw offset (rad)
        
        # Sensor/frame alignment
        self.R_imu_to_body = np.eye(3)  # Rotation from IMU sensor frame to robot body frame
        self.last_accel_body = None
        self.last_gyro_body = None
        self.last_mag_body = None
        
        # Gating and detection parameters
        self.mag_heading_gate_deg = 30.0
        self.mag_norm_gate_rel = 0.25
        self.gyro_gate_rate = 0.5  # rad/s
        self.accel_gate_rel_g = 0.2  # fraction of g deviation
        
        # Zero motion update strength (lower is stronger pull to zero)
        self.R_zvu = np.eye(2) * 0.01  # measurement noise for [vx, vy] = [0,0]
        
        # Initialization flags
        self.yaw_initialized = False
        
        # Time tracking
        self.last_time = None
        
        # Reference position (for GPS conversion)
        self.ref_lat = None
        self.ref_lon = None
        self.ref_alt = None
    
    def _create_process_noise_matrix(self) -> np.ndarray:
        """
        Create process noise covariance matrix Q as per formulary
        Q represents the uncertainty in the process model
        """
        Q = np.zeros((self.n_states, self.n_states))
        
        # Position noise (low, as it's integrated from velocity)
        Q[0:3, 0:3] = np.eye(3) * 0.01
        
        # Velocity noise (moderate)
        Q[3:6, 3:6] = np.eye(3) * 0.1
        
        # Orientation noise (low to moderate)
        Q[6:9, 6:9] = np.eye(3) * 0.05
        
        # Angular velocity noise (moderate)
        Q[9:12, 9:12] = np.eye(3) * 0.1
        
        return Q
    
    def predict(self, dt: float):
        """
        Prediction step of EKF as per formulary
        x_k = F_k * x_{k-1}
        P_k = F_k * P_{k-1} * F_k^T + Q_k
        """
        # Clamp dt for stability in case of timestamp jitter
        if dt <= 0:
            return
        dt = float(max(1e-3, min(dt, 0.1)))
            
        # State transition model (kinematic model) as per formulary
        F = self._compute_state_transition_matrix(dt)
        
        # Predict state: x = F * x
        self.x = F @ self.x
        
        # Predict covariance: P = F * P * F^T + Q
        self.P = F @ self.P @ F.T + self.Q * dt
        
        # Normalize angles to [-π, π] as per formulary
        self.x[6:9] = self._normalize_angles(self.x[6:9])
    
    def _compute_state_transition_matrix(self, dt: float) -> np.ndarray:
        """
        Compute state transition matrix F as per formulary
        F represents the linearized system dynamics
        """
        F = np.eye(self.n_states)
        
        # Position updates from velocity (kinematic model)
        F[0:3, 3:6] = np.eye(3) * dt
        
        # Orientation updates from angular velocity (kinematic model)
        F[6:9, 9:12] = np.eye(3) * dt
        
        return F
    
    def _normalize_angles(self, angles: np.ndarray) -> np.ndarray:
        """
        Normalize angles to [-π, π] as per formulary
        Ensures consistent angle representation
        """
        return (angles + np.pi) % (2 * np.pi) - np.pi
    
    def update_imu(self, accel: np.ndarray, gyro: np.ndarray):
        """
        Update with IMU measurements as per formulary
        z_imu = [ax, ay, az, wx, wy, wz]
        """
        # Expected IMU measurements based on current state
        h_imu = self._compute_expected_imu(self.x)
        
        # Jacobian matrix for IMU as per formulary
        H_imu = self._compute_imu_jacobian(self.x)
        
        # Measurement vector
        z_imu = np.concatenate([accel, gyro])
        
        # Perform update
        self._kalman_update(z_imu, h_imu, H_imu, self.R_imu)
    
    def _compute_expected_imu(self, state: np.ndarray) -> np.ndarray:
        """
        Compute expected IMU measurements as per formulary
        h_imu = [expected_ax, expected_ay, expected_az, wx, wy, wz]
        """
        roll, pitch, yaw = state[6:9]
        wx, wy, wz = state[9:12]
        
        # Expected acceleration (gravity in body frame + motion) as per formulary
        # Gravity vector in body frame: g_body = R_b^n * [0, 0, g]
        g_body = np.array([
            -self.gravity * np.sin(pitch),
            self.gravity * np.sin(roll) * np.cos(pitch),
            self.gravity * np.cos(roll) * np.cos(pitch)
        ])
        
        # Expected gyroscope (direct measurement of angular velocity)
        expected_gyro = np.array([wx, wy, wz])
        
        return np.concatenate([g_body, expected_gyro])
    
    def _compute_imu_jacobian(self, state: np.ndarray) -> np.ndarray:
        """
        Compute Jacobian matrix for IMU measurements as per formulary
        H_imu = ∂h_imu/∂x
        """
        roll, pitch, yaw = state[6:9]
        
        H = np.zeros((6, self.n_states))
        
        # Accelerometer Jacobian (w.r.t. roll and pitch) as per formulary
        H[0, 7] = -self.gravity * np.cos(pitch)  # ∂ax/∂pitch
        H[1, 6] = self.gravity * np.cos(roll) * np.cos(pitch)   # ∂ay/∂roll
        H[1, 7] = -self.gravity * np.sin(roll) * np.sin(pitch)  # ∂ay/∂pitch
        H[2, 6] = -self.gravity * np.sin(roll) * np.cos(pitch)  # ∂az/∂roll
        H[2, 7] = -self.gravity * np.cos(roll) * np.sin(pitch)  # ∂az/∂pitch
        
        # Gyroscope Jacobian (direct measurement) as per formulary
        H[3:6, 9:12] = np.eye(3)
        
        return H
    
    def update_magnetometer(self, mag: np.ndarray):
        """
        Update with magnetometer measurements as per formulary
        z_mag = [mx, my, mz] (normalized)
        """
        # Normalize magnetometer reading as per formulary
        if mag is None or not np.all(np.isfinite(mag)):
            return
        if np.linalg.norm(mag) <= 0:
            return
        mag_norm = mag / np.linalg.norm(mag)
        
        # Gating based on dynamics and magnetic norm/heading residual
        if self.last_gyro_body is not None and np.linalg.norm(self.last_gyro_body) > self.gyro_gate_rate:
            return
        if self.last_accel_body is not None:
            accel_norm = np.linalg.norm(self.last_accel_body)
            if not (abs(accel_norm - self.gravity) <= self.accel_gate_rel_g * self.gravity):
                return
        
        # Compute heading residual (proxy) using xy plane
        meas_hdg = np.arctan2(mag_norm[1], mag_norm[0])
        expected_hdg = self.x[8] + self.mag_declination + self.yaw_mount_offset
        hdg_residual = (meas_hdg - expected_hdg + np.pi) % (2 * np.pi) - np.pi
        if abs(np.degrees(hdg_residual)) > self.mag_heading_gate_deg:
            return
        
        # Expected magnetometer reading
        h_mag = self._compute_expected_magnetometer(self.x)
        
        # Jacobian for magnetometer as per formulary
        H_mag = self._compute_magnetometer_jacobian(self.x)
        
        # Perform update
        self._kalman_update(mag_norm, h_mag, H_mag, self.R_mag)
    
    def _compute_expected_magnetometer(self, state: np.ndarray) -> np.ndarray:
        """
        Compute expected magnetometer measurements as per formulary
        h_mag = R_b^n * m_n (normalized)
        """
        roll, pitch, yaw = state[6:9]
        
        # Magnetic field vector in NED frame (assuming north-aligned) as per formulary
        mag_ned = np.array([np.cos(self.mag_declination), 
                           np.sin(self.mag_declination), 
                           0.0])
        
        # Rotation matrix from NED to body frame as per formulary
        R = self._rotation_matrix_ned_to_body(roll, pitch, yaw)
        
        # Expected magnetometer reading in body frame
        mag_body = R @ mag_ned
        
        return mag_body / np.linalg.norm(mag_body)
    
    def _rotation_matrix_ned_to_body(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """
        Compute rotation matrix from NED to body frame as per formulary
        R_b^n = R_z(yaw) * R_y(pitch) * R_x(roll)
        """
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        
        # Rotation matrix as per formulary
        R = np.array([
            [cp*cy, cp*sy, -sp],
            [sr*sp*cy - cr*sy, sr*sp*sy + cr*cy, sr*cp],
            [cr*sp*cy + sr*sy, cr*sp*sy - sr*cy, cr*cp]
        ])
        
        return R
    
    def _compute_magnetometer_jacobian(self, state: np.ndarray) -> np.ndarray:
        """
        Compute Jacobian for magnetometer measurements as per formulary
        H_mag = ∂h_mag/∂x
        """
        # This is a simplified version - full implementation would include
        # derivatives of the rotation matrix w.r.t. roll, pitch, yaw
        H = np.zeros((3, self.n_states))
        
        # For now, assume linear relationship with orientation as per formulary
        H[0:3, 6:9] = np.eye(3) * 0.1
        
        return H
    
    def update_gps(self, lat: float, lon: float, alt: float):
        """
        Update with GPS measurements as per formulary
        z_gps = [x, y, z] (local NED coordinates)
        """
        if self.ref_lat is None:
            # Set reference position on first GPS fix as per formulary
            self.ref_lat = lat
            self.ref_lon = lon
            self.ref_alt = alt
            return
        
        # Convert GPS to local coordinates as per formulary
        x, y, z = self._gps_to_local(lat, lon, alt)
        
        # GPS measurement vector
        z_gps = np.array([x, y, z])
        
        # Expected GPS measurement (just position) as per formulary
        h_gps = self.x[0:3]
        
        # Jacobian (direct measurement of position) as per formulary
        H_gps = np.zeros((3, self.n_states))
        H_gps[0:3, 0:3] = np.eye(3)
        
        # Perform update
        self._kalman_update(z_gps, h_gps, H_gps, self.R_gps)
    
    def _gps_to_local(self, lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
        """
        Convert GPS coordinates to local NED coordinates as per formulary
        Uses simplified conversion for small distances
        """
        # Simplified conversion (assumes small distances) as per formulary
        R_earth = 6371000  # Earth radius in meters
        
        dlat = np.radians(lat - self.ref_lat)
        dlon = np.radians(lon - self.ref_lon)
        
        x = R_earth * dlon * np.cos(np.radians(self.ref_lat))
        y = R_earth * dlat
        z = alt - self.ref_alt
        
        return x, y, z
    
    def update_chassis(self, x: float, y: float, yaw: float):
        """
        Update with chassis encoder measurements as per formulary
        z_chassis = [x, y, yaw] (local coordinates)
        """
        z_chassis = np.array([x, y, yaw])
        
        # Expected chassis measurement as per formulary
        h_chassis = np.array([self.x[0], self.x[1], self.x[8]])
        
        # Jacobian as per formulary
        H_chassis = np.zeros((3, self.n_states))
        H_chassis[0, 0] = 1  # x position
        H_chassis[1, 1] = 1  # y position
        H_chassis[2, 8] = 1  # yaw angle
        
        # Perform update
        self._kalman_update(z_chassis, h_chassis, H_chassis, self.R_chassis)
    
    def _kalman_update(self, z: np.ndarray, h: np.ndarray, H: np.ndarray, R: np.ndarray):
        """
        Perform Kalman update step as per formulary
        y = z - h (innovation)
        S = H * P * H^T + R (innovation covariance)
        K = P * H^T * S^(-1) (Kalman gain)
        x = x + K * y (state update)
        P = (I - K * H) * P * (I - K * H)^T + K * R * K^T (covariance update)
        """
        # Innovation
        y = z - h
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.x = self.x + K @ y
        
        # Update covariance (Joseph form for numerical stability) as per formulary
        I_KH = np.eye(self.n_states) - K @ H
        KRKt = K @ R @ K.T
        self.P = I_KH @ self.P @ I_KH.T + KRKt
        
        # Normalize angles as per formulary
        self.x[6:9] = self._normalize_angles(self.x[6:9])
    
    def get_state(self) -> EKFState:
        """Get current state as EKFState object as per formulary"""
        return EKFState.from_vector(self.x)
    
    def process_sensor_data(self, sensor_data: SensorData):
        """
        Process all available sensor data as per formulary
        Follows the sensor fusion strategy outlined in the formulary
        """
        # Compute time step
        current_time = sensor_data.timestamp
        if self.last_time is not None:
            dt = current_time - self.last_time
            if dt > 0:
                self.predict(dt)
        self.last_time = current_time
        
        # Transform sensors to body frame
        accel_body = None
        gyro_body = None
        mag_body = None
        if sensor_data.accel is not None:
            accel_body = self.R_imu_to_body @ sensor_data.accel
            self.last_accel_body = accel_body
        if sensor_data.gyro is not None:
            gyro_body = self.R_imu_to_body @ sensor_data.gyro
            self.last_gyro_body = gyro_body
        if sensor_data.mag is not None:
            mag_body = self.R_imu_to_body @ sensor_data.mag
            self.last_mag_body = mag_body

        # Simple yaw initialization from magnetometer during stationary dwell
        if not self.yaw_initialized and (mag_body is not None) and (accel_body is not None) and (gyro_body is not None):
            accel_norm = np.linalg.norm(accel_body)
            if (np.linalg.norm(gyro_body) < 0.05) and (abs(accel_norm - self.gravity) < 0.1 * self.gravity):
                yaw0 = np.arctan2(mag_body[1], mag_body[0]) - self.mag_declination - self.yaw_mount_offset
                self.x[8] = ((yaw0 + np.pi) % (2 * np.pi)) - np.pi
                self.yaw_initialized = True
        
        # Update with available sensors as per formulary
        if accel_body is not None and gyro_body is not None:
            self.update_imu(accel_body, gyro_body)
        
        if mag_body is not None:
            self.update_magnetometer(mag_body)
        
        if all(x is not None for x in [sensor_data.gps_lat, sensor_data.gps_lon, sensor_data.gps_alt]):
            self.update_gps(sensor_data.gps_lat, sensor_data.gps_lon, sensor_data.gps_alt)
        
        if all(x is not None for x in [sensor_data.chassis_x, sensor_data.chassis_y, sensor_data.chassis_yaw]):
            self.update_chassis(sensor_data.chassis_x, sensor_data.chassis_y, sensor_data.chassis_yaw)

        # Zero-Velocity Update (ZVU) during detected stops
        if self.last_accel_body is not None and self.last_gyro_body is not None:
            accel_norm = np.linalg.norm(self.last_accel_body)
            if (np.linalg.norm(self.last_gyro_body) < 0.05) and (abs(accel_norm - self.gravity) < 0.05 * self.gravity):
                # Measurement: vx = 0, vy = 0
                z = np.array([0.0, 0.0])
                h = self.x[3:5]
                H = np.zeros((2, self.n_states))
                H[0, 3] = 1.0
                H[1, 4] = 1.0
                self._kalman_update(z, h, H, self.R_zvu)