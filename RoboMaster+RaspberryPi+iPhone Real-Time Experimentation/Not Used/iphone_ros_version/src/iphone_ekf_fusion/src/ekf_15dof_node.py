#!/usr/bin/env python3
"""
15-DOF Extended Kalman Filter ROS Node
=======================================
Implements the 15-dimensional state EKF for iPhone-RoboMaster integration
Following RoboMaster EKF Formulary specifications

State Vector (15-DOF):
[x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz, bax, bay, baz]
- Position (3): x, y, z
- Velocity (3): vx, vy, vz  
- Orientation (3): roll, pitch, yaw
- Angular velocity (3): wx, wy, wz
- Accelerometer bias (3): bax, bay, baz

Author: RoboMaster EKF ROS Integration
Date: 2025
"""

import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Quaternion, Point
from sensor_msgs.msg import Imu, NavSatFix, MagneticField, FluidPressure
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import tf2_ros
import tf2_geometry_msgs
from typing import Optional, Tuple

# Import custom messages (will be generated from .msg files)
from iphone_ekf_fusion.msg import IPhoneSensorData, EKFState


class EKF15DOF:
    """
    15-DOF Extended Kalman Filter Implementation
    Full state estimation with IMU bias tracking
    """
    
    def __init__(self):
        """Initialize 15-DOF EKF with ROS parameters"""
        
        # State dimension
        self.n_states = 15
        
        # Initialize state vector [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz, bax, bay, baz]
        self.x = np.zeros(self.n_states)
        
        # Initialize covariance matrix
        self.P = np.eye(self.n_states)
        self._initialize_covariance()
        
        # Process noise covariance Q
        self.Q = self._create_process_noise_matrix()
        
        # Measurement noise covariances R
        self._initialize_measurement_noise()
        
        # Physical constants
        self.gravity = rospy.get_param('~gravity', 9.81)
        self.magnetic_declination = rospy.get_param('~magnetic_declination', 0.0)
        
        # GPS reference point
        self.gps_reference = None
        
        # Time tracking
        self.last_update_time = None
        self.dt_nominal = 0.02  # 50 Hz nominal rate
        
        # Statistics
        self.update_count = 0
        self.prediction_count = 0
        
        rospy.loginfo("15-DOF EKF initialized")
    
    def _initialize_covariance(self):
        """Initialize covariance matrix with ROS parameters"""
        # Position uncertainty
        self.P[0:3, 0:3] *= rospy.get_param('~initial_position_cov', 1.0)
        # Velocity uncertainty
        self.P[3:6, 3:6] *= rospy.get_param('~initial_velocity_cov', 0.5)
        # Orientation uncertainty
        self.P[6:9, 6:9] *= rospy.get_param('~initial_orientation_cov', 0.1)
        # Angular velocity uncertainty
        self.P[9:12, 9:12] *= rospy.get_param('~initial_angular_velocity_cov', 0.1)
        # Accelerometer bias uncertainty
        self.P[12:15, 12:15] *= rospy.get_param('~initial_accel_bias_cov', 0.01)
    
    def _create_process_noise_matrix(self) -> np.ndarray:
        """Create process noise covariance matrix Q"""
        Q = np.zeros((self.n_states, self.n_states))
        
        # Position process noise
        q_pos = rospy.get_param('~q_position', 0.01)
        Q[0:3, 0:3] = np.eye(3) * q_pos
        
        # Velocity process noise
        q_vel = rospy.get_param('~q_velocity', 0.1)
        Q[3:6, 3:6] = np.eye(3) * q_vel
        
        # Orientation process noise
        q_orient = rospy.get_param('~q_orientation', 0.05)
        Q[6:9, 6:9] = np.eye(3) * q_orient
        
        # Angular velocity process noise
        q_angvel = rospy.get_param('~q_angular_velocity', 0.1)
        Q[9:12, 9:12] = np.eye(3) * q_angvel
        
        # Accelerometer bias process noise (slow drift)
        q_bias = rospy.get_param('~q_accel_bias', 0.001)
        Q[12:15, 12:15] = np.eye(3) * q_bias
        
        return Q
    
    def _initialize_measurement_noise(self):
        """Initialize measurement noise covariances"""
        self.R_accel = np.eye(3) * rospy.get_param('~r_accelerometer', 0.5)
        self.R_gyro = np.eye(3) * rospy.get_param('~r_gyroscope', 0.1)
        self.R_mag = np.eye(3) * rospy.get_param('~r_magnetometer', 0.5)
        self.R_gps = np.eye(3) * rospy.get_param('~r_gps', 1.0)
        self.R_baro = np.array([[rospy.get_param('~r_barometer', 0.1)]])
    
    def predict(self, dt: float):
        """
        Prediction step with full 15-DOF dynamics
        x_k = f(x_{k-1}, u_k) + w_k
        P_k = F_k * P_{k-1} * F_k^T + Q_k
        """
        if dt <= 0:
            rospy.logwarn(f"Invalid dt: {dt}")
            return
        
        # State transition with nonlinear dynamics
        self.x = self._state_transition(self.x, dt)
        
        # Jacobian of state transition
        F = self._compute_state_jacobian(self.x, dt)
        
        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q * dt
        
        # Normalize angles
        self.x[6:9] = self._normalize_angles(self.x[6:9])
        
        self.prediction_count += 1
    
    def _state_transition(self, x: np.ndarray, dt: float) -> np.ndarray:
        """
        Nonlinear state transition for 15-DOF system
        Includes IMU bias in the dynamics
        """
        x_new = x.copy()
        
        # Extract states
        pos = x[0:3]
        vel = x[3:6]
        orient = x[6:9]
        angvel = x[9:12]
        accel_bias = x[12:15]
        
        roll, pitch, yaw = orient
        
        # Position update (simple integration)
        x_new[0:3] = pos + vel * dt
        
        # Velocity update (considering orientation for gravity compensation)
        # This is where bias-compensated acceleration would be integrated
        # For now, simple integration with gravity
        gravity_world = np.array([0, 0, self.gravity])
        R = self._rotation_matrix_body_to_world(roll, pitch, yaw)
        gravity_body = R.T @ gravity_world
        
        # Velocity stays approximately constant in prediction
        # (acceleration will be corrected in measurement update)
        x_new[3:6] = vel
        
        # Orientation update from angular velocity
        x_new[6:9] = orient + angvel * dt
        
        # Angular velocity stays constant in prediction
        x_new[9:12] = angvel
        
        # Bias states stay constant (random walk model)
        x_new[12:15] = accel_bias
        
        return x_new
    
    def _compute_state_jacobian(self, x: np.ndarray, dt: float) -> np.ndarray:
        """
        Compute Jacobian matrix for state transition
        F = ∂f/∂x for the 15-DOF system
        """
        F = np.eye(self.n_states)
        
        # Position derivative w.r.t velocity
        F[0:3, 3:6] = np.eye(3) * dt
        
        # Orientation derivative w.r.t angular velocity
        F[6:9, 9:12] = np.eye(3) * dt
        
        # More complex derivatives could be added here for
        # coupling between states (e.g., velocity w.r.t orientation)
        
        return F
    
    def update_imu(self, accel: np.ndarray, gyro: np.ndarray):
        """
        Update with IMU measurements
        Accounts for accelerometer bias in measurement model
        """
        # Measurement vector
        z = np.concatenate([accel, gyro])
        
        # Expected measurements (including bias)
        h = self._compute_expected_imu()
        
        # Measurement Jacobian
        H = self._compute_imu_jacobian()
        
        # Measurement noise
        R = np.block([
            [self.R_accel, np.zeros((3, 3))],
            [np.zeros((3, 3)), self.R_gyro]
        ])
        
        # Kalman update
        self._kalman_update(z, h, H, R)
    
    def _compute_expected_imu(self) -> np.ndarray:
        """
        Compute expected IMU measurements
        Includes accelerometer bias in the model
        """
        roll, pitch, yaw = self.x[6:9]
        wx, wy, wz = self.x[9:12]
        bax, bay, baz = self.x[12:15]  # Accelerometer biases
        
        # Expected acceleration (gravity in body frame + bias)
        expected_accel = np.array([
            -self.gravity * np.sin(pitch) + bax,
            self.gravity * np.sin(roll) * np.cos(pitch) + bay,
            self.gravity * np.cos(roll) * np.cos(pitch) + baz
        ])
        
        # Expected gyroscope (direct measurement of angular velocity)
        expected_gyro = np.array([wx, wy, wz])
        
        return np.concatenate([expected_accel, expected_gyro])
    
    def _compute_imu_jacobian(self) -> np.ndarray:
        """
        Compute Jacobian matrix for IMU measurements
        H = ∂h/∂x for 15-DOF system including bias terms
        """
        roll, pitch = self.x[6:8]
        
        H = np.zeros((6, self.n_states))
        
        # Accelerometer Jacobian w.r.t orientation
        H[0, 7] = -self.gravity * np.cos(pitch)  # ∂ax/∂pitch
        H[1, 6] = self.gravity * np.cos(roll) * np.cos(pitch)  # ∂ay/∂roll
        H[1, 7] = -self.gravity * np.sin(roll) * np.sin(pitch)  # ∂ay/∂pitch
        H[2, 6] = -self.gravity * np.sin(roll) * np.cos(pitch)  # ∂az/∂roll
        H[2, 7] = -self.gravity * np.cos(roll) * np.sin(pitch)  # ∂az/∂pitch
        
        # Accelerometer Jacobian w.r.t bias
        H[0:3, 12:15] = np.eye(3)  # Direct bias contribution
        
        # Gyroscope Jacobian
        H[3:6, 9:12] = np.eye(3)  # Direct measurement of angular velocity
        
        return H
    
    def update_magnetometer(self, mag: np.ndarray):
        """Update with magnetometer measurements"""
        # Simplified heading correction
        mag_heading = np.arctan2(mag[1], mag[0]) - self.magnetic_declination
        
        z = np.array([mag_heading])
        h = np.array([self.x[8]])  # Current yaw
        
        H = np.zeros((1, self.n_states))
        H[0, 8] = 1.0
        
        R = np.array([[0.1]])
        
        self._kalman_update(z, h, H, R)
    
    def update_gps(self, lat: float, lon: float, alt: float):
        """Update with GPS measurements"""
        if self.gps_reference is None:
            self.gps_reference = {'lat': lat, 'lon': lon}
            rospy.loginfo(f"GPS reference set: {self.gps_reference}")
            return
        
        # Convert to local coordinates
        lat_to_m = 111320.0
        lon_to_m = 111320.0 * np.cos(np.radians(self.gps_reference['lat']))
        
        x_gps = (lat - self.gps_reference['lat']) * lat_to_m
        y_gps = (lon - self.gps_reference['lon']) * lon_to_m
        z_gps = -alt  # NED convention
        
        z = np.array([x_gps, y_gps, z_gps])
        h = self.x[0:3]
        
        H = np.zeros((3, self.n_states))
        H[0:3, 0:3] = np.eye(3)
        
        self._kalman_update(z, h, H, self.R_gps)
    
    def update_barometer(self, pressure: float, temperature: float = 15.0):
        """Update with barometric pressure measurements"""
        # Convert pressure to altitude using barometric formula
        p0 = 101325.0  # Sea level pressure (Pa)
        altitude = 44330.0 * (1.0 - (pressure / p0) ** (1.0/5.255))
        
        z = np.array([-altitude])  # NED convention
        h = np.array([self.x[2]])
        
        H = np.zeros((1, self.n_states))
        H[0, 2] = 1.0
        
        self._kalman_update(z, h, H, self.R_baro)
    
    def _kalman_update(self, z: np.ndarray, h: np.ndarray, 
                       H: np.ndarray, R: np.ndarray):
        """
        Kalman update step
        Joseph form for numerical stability
        """
        # Innovation
        y = z - h
        
        # Handle angle wrapping
        if len(y) == 1 and H[0, 8] != 0:
            y[0] = self._normalize_angle(y[0])
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            rospy.logwarn("Singular matrix in Kalman gain")
            return
        
        # State update
        self.x = self.x + K @ y
        
        # Normalize angles
        self.x[6:9] = self._normalize_angles(self.x[6:9])
        
        # Covariance update (Joseph form)
        I_KH = np.eye(self.n_states) - K @ H
        KRKt = K @ R @ K.T
        self.P = I_KH @ self.P @ I_KH.T + KRKt
        
        self.update_count += 1
    
    def _rotation_matrix_body_to_world(self, roll: float, pitch: float, 
                                       yaw: float) -> np.ndarray:
        """Compute rotation matrix from body to world frame"""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        
        return np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-π, π]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def _normalize_angles(self, angles: np.ndarray) -> np.ndarray:
        """Normalize multiple angles"""
        return (angles + np.pi) % (2 * np.pi) - np.pi
    
    def get_state_msg(self) -> EKFState:
        """Convert current state to ROS message"""
        msg = EKFState()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        
        # Position
        msg.position.x = self.x[0]
        msg.position.y = self.x[1]
        msg.position.z = self.x[2]
        
        # Velocity
        msg.velocity.x = self.x[3]
        msg.velocity.y = self.x[4]
        msg.velocity.z = self.x[5]
        
        # Orientation (convert to quaternion)
        roll, pitch, yaw = self.x[6:9]
        msg.orientation = self._euler_to_quaternion(roll, pitch, yaw)
        msg.euler_angles.x = roll
        msg.euler_angles.y = pitch
        msg.euler_angles.z = yaw
        
        # Angular velocity
        msg.angular_velocity.x = self.x[9]
        msg.angular_velocity.y = self.x[10]
        msg.angular_velocity.z = self.x[11]
        
        # IMU biases
        msg.accelerometer_bias.x = self.x[12]
        msg.accelerometer_bias.y = self.x[13]
        msg.accelerometer_bias.z = self.x[14]
        
        # Covariance
        msg.covariance = self.P.flatten().tolist()
        msg.covariance_trace = np.trace(self.P)
        
        # Metadata
        msg.update_count = self.update_count
        msg.timestamp = rospy.Time.now().to_sec()
        
        return msg
    
    def _euler_to_quaternion(self, roll: float, pitch: float, 
                            yaw: float) -> Quaternion:
        """Convert Euler angles to quaternion"""
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        
        return q


class EKF15DOFNode:
    """
    ROS Node for 15-DOF EKF
    Manages subscriptions, publications, and timing
    """
    
    def __init__(self):
        """Initialize ROS node"""
        rospy.init_node('ekf_15dof_node', anonymous=False)
        
        # Create EKF instance
        self.ekf = EKF15DOF()
        
        # Publishers
        self.state_pub = rospy.Publisher(
            '~ekf_state', EKFState, queue_size=1
        )
        self.odom_pub = rospy.Publisher(
            '~odom', Odometry, queue_size=1
        )
        
        # Subscribers
        self.iphone_sub = rospy.Subscriber(
            '/iphone/sensor_data', IPhoneSensorData, 
            self.iphone_callback
        )
        self.imu_sub = rospy.Subscriber(
            '/imu/data', Imu, self.imu_callback
        )
        self.gps_sub = rospy.Subscriber(
            '/gps/fix', NavSatFix, self.gps_callback
        )
        self.mag_sub = rospy.Subscriber(
            '/imu/mag', MagneticField, self.mag_callback
        )
        self.pressure_sub = rospy.Subscriber(
            '/pressure', FluidPressure, self.pressure_callback
        )
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Timer for prediction step
        self.predict_rate = rospy.get_param('~predict_rate', 50.0)
        self.predict_timer = rospy.Timer(
            rospy.Duration(1.0 / self.predict_rate),
            self.predict_callback
        )
        
        # Time tracking
        self.last_predict_time = rospy.Time.now()
        
        rospy.loginfo("EKF 15-DOF Node initialized")
    
    def iphone_callback(self, msg: IPhoneSensorData):
        """Process iPhone sensor data"""
        # Extract IMU data
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # Update EKF with IMU
        self.ekf.update_imu(accel, gyro)
        
        # Update with magnetometer if available
        if msg.magnetic_field_accuracy > 0.5:
            mag = np.array([
                msg.magnetic_field.x,
                msg.magnetic_field.y,
                msg.magnetic_field.z
            ])
            self.ekf.update_magnetometer(mag)
        
        # Update with GPS if valid
        if msg.gps_valid:
            self.ekf.update_gps(
                msg.latitude,
                msg.longitude,
                msg.altitude
            )
        
        # Update with barometer
        if msg.pressure > 0:
            self.ekf.update_barometer(msg.pressure)
        
        # Publish updated state
        self.publish_state()
    
    def imu_callback(self, msg: Imu):
        """Process standard IMU message"""
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        self.ekf.update_imu(accel, gyro)
        self.publish_state()
    
    def gps_callback(self, msg: NavSatFix):
        """Process GPS fix"""
        if msg.status.status >= 0:  # Has fix
            self.ekf.update_gps(
                msg.latitude,
                msg.longitude,
                msg.altitude
            )
            self.publish_state()
    
    def mag_callback(self, msg: MagneticField):
        """Process magnetometer data"""
        mag = np.array([
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z
        ])
        self.ekf.update_magnetometer(mag)
        self.publish_state()
    
    def pressure_callback(self, msg: FluidPressure):
        """Process barometric pressure"""
        self.ekf.update_barometer(msg.fluid_pressure)
        self.publish_state()
    
    def predict_callback(self, event):
        """Periodic prediction step"""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_predict_time).to_sec()
        
        if dt > 0:
            self.ekf.predict(dt)
            self.last_predict_time = current_time
    
    def publish_state(self):
        """Publish EKF state and odometry"""
        # Publish EKF state message
        state_msg = self.ekf.get_state_msg()
        self.state_pub.publish(state_msg)
        
        # Publish odometry message
        odom_msg = self.create_odometry_msg(state_msg)
        self.odom_pub.publish(odom_msg)
        
        # Broadcast TF transform
        self.broadcast_transform(state_msg)
    
    def create_odometry_msg(self, state: EKFState) -> Odometry:
        """Create odometry message from EKF state"""
        odom = Odometry()
        odom.header = state.header
        odom.child_frame_id = "base_link"
        
        # Position
        odom.pose.pose.position = state.position
        odom.pose.pose.orientation = state.orientation
        
        # Velocity
        odom.twist.twist.linear = state.velocity
        odom.twist.twist.angular = state.angular_velocity
        
        # Covariance (simplified - using position and orientation covariance)
        pose_cov = np.zeros(36)
        pose_cov[0:3] = state.covariance[0:3]  # Position covariance
        pose_cov[21:24] = state.covariance[6:9]  # Orientation covariance
        odom.pose.covariance = pose_cov.tolist()
        
        return odom
    
    def broadcast_transform(self, state: EKFState):
        """Broadcast TF transform"""
        from geometry_msgs.msg import TransformStamped
        
        t = TransformStamped()
        t.header = state.header
        t.child_frame_id = "base_link"
        
        t.transform.translation.x = state.position.x
        t.transform.translation.y = state.position.y
        t.transform.translation.z = state.position.z
        t.transform.rotation = state.orientation
        
        self.tf_broadcaster.sendTransform(t)
    
    def run(self):
        """Main node loop"""
        rospy.spin()


if __name__ == '__main__':
    try:
        node = EKF15DOFNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
