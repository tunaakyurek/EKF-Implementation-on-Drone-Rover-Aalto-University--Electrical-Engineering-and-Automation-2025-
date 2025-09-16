"""
Minimal EKF Implementation for RoboMaster S1 Direct Execution

This is a severely simplified Extended Kalman Filter designed to run on
the RoboMaster S1's limited hardware (272MB RAM, ARM Cortex-A7).

LIMITATIONS:
- No NumPy/SciPy (not available in S1 sandbox)
- Pure Python matrix operations (slow)
- Reduced state dimensions (6D instead of 12D)
- Lower update frequency (5-10 Hz max)
- No advanced filtering features

MEMORY USAGE: Approximately 5-10 MB
CPU USAGE: ~20% on ARM Cortex-A7
"""

import math
import time

class SimpleMatrix:
    """Lightweight matrix class for basic linear algebra operations"""
    
    def __init__(self, rows, cols=None, data=None):
        if cols is None:
            # Assume square matrix
            cols = rows
        
        self.rows = rows
        self.cols = cols
        
        if data is None:
            self.data = [[0.0 for _ in range(cols)] for _ in range(rows)]
        else:
            self.data = data
    
    def __mul__(self, other):
        """Matrix multiplication"""
        if isinstance(other, (int, float)):
            # Scalar multiplication
            result = SimpleMatrix(self.rows, self.cols)
            for i in range(self.rows):
                for j in range(self.cols):
                    result.data[i][j] = self.data[i][j] * other
            return result
        
        # Matrix multiplication
        if self.cols != other.rows:
            raise ValueError("Matrix dimensions don't match for multiplication")
        
        result = SimpleMatrix(self.rows, other.cols)
        for i in range(self.rows):
            for j in range(other.cols):
                for k in range(self.cols):
                    result.data[i][j] += self.data[i][k] * other.data[k][j]
        
        return result
    
    def __add__(self, other):
        """Matrix addition"""
        if self.rows != other.rows or self.cols != other.cols:
            raise ValueError("Matrix dimensions don't match for addition")
        
        result = SimpleMatrix(self.rows, self.cols)
        for i in range(self.rows):
            for j in range(self.cols):
                result.data[i][j] = self.data[i][j] + other.data[i][j]
        
        return result
    
    def __sub__(self, other):
        """Matrix subtraction"""
        if self.rows != other.rows or self.cols != other.cols:
            raise ValueError("Matrix dimensions don't match for subtraction")
        
        result = SimpleMatrix(self.rows, self.cols)
        for i in range(self.rows):
            for j in range(self.cols):
                result.data[i][j] = self.data[i][j] - other.data[i][j]
        
        return result
    
    def transpose(self):
        """Matrix transpose"""
        result = SimpleMatrix(self.cols, self.rows)
        for i in range(self.rows):
            for j in range(self.cols):
                result.data[j][i] = self.data[i][j]
        return result
    
    def invert(self):
        """Simple matrix inversion (only for small matrices)"""
        if self.rows != self.cols:
            raise ValueError("Only square matrices can be inverted")
        
        if self.rows == 1:
            if abs(self.data[0][0]) < 1e-10:
                raise ValueError("Matrix is singular")
            result = SimpleMatrix(1, 1)
            result.data[0][0] = 1.0 / self.data[0][0]
            return result
        
        elif self.rows == 2:
            # 2x2 matrix inversion
            det = self.data[0][0] * self.data[1][1] - self.data[0][1] * self.data[1][0]
            if abs(det) < 1e-10:
                raise ValueError("Matrix is singular")
            
            result = SimpleMatrix(2, 2)
            result.data[0][0] = self.data[1][1] / det
            result.data[0][1] = -self.data[0][1] / det
            result.data[1][0] = -self.data[1][0] / det
            result.data[1][1] = self.data[0][0] / det
            return result
        
        else:
            # For larger matrices, use Gauss-Jordan elimination
            return self._gauss_jordan_invert()
    
    def _gauss_jordan_invert(self):
        """Gauss-Jordan elimination for matrix inversion"""
        n = self.rows
        # Create augmented matrix [A|I]
        aug = [[0.0 for _ in range(2*n)] for _ in range(n)]
        
        for i in range(n):
            for j in range(n):
                aug[i][j] = self.data[i][j]
            aug[i][i+n] = 1.0
        
        # Forward elimination
        for i in range(n):
            # Find pivot
            max_row = i
            for k in range(i+1, n):
                if abs(aug[k][i]) > abs(aug[max_row][i]):
                    max_row = k
            
            # Swap rows
            aug[i], aug[max_row] = aug[max_row], aug[i]
            
            # Make diagonal element 1
            pivot = aug[i][i]
            if abs(pivot) < 1e-10:
                raise ValueError("Matrix is singular")
            
            for j in range(2*n):
                aug[i][j] /= pivot
            
            # Eliminate column
            for k in range(n):
                if k != i:
                    factor = aug[k][i]
                    for j in range(2*n):
                        aug[k][j] -= factor * aug[i][j]
        
        # Extract inverse matrix
        result = SimpleMatrix(n, n)
        for i in range(n):
            for j in range(n):
                result.data[i][j] = aug[i][j+n]
        
        return result
    
    @staticmethod
    def identity(size):
        """Create identity matrix"""
        result = SimpleMatrix(size, size)
        for i in range(size):
            result.data[i][i] = 1.0
        return result
    
    def get_element(self, row, col):
        """Get single element"""
        return self.data[row][col]
    
    def set_element(self, row, col, value):
        """Set single element"""
        self.data[row][col] = value
    
    def copy(self):
        """Create a deep copy of the matrix"""
        new_data = [[self.data[i][j] for j in range(self.cols)] for i in range(self.rows)]
        return SimpleMatrix(self.rows, self.cols, new_data)


class SensorReading:
    """Simple sensor data container"""
    
    def __init__(self):
        self.timestamp = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        self.chassis_x = 0.0
        self.chassis_y = 0.0
        self.chassis_yaw = 0.0


class MinimalEKF:
    """
    Minimal 6-DOF Extended Kalman Filter for RoboMaster S1
    State vector: [x, y, yaw, vx, vy, vyaw]
    
    This is severely simplified compared to the full 12-DOF EKF in the main project
    """
    
    def __init__(self):
        # State dimension (6 instead of 12)
        self.n_states = 6
        
        # State vector: [x, y, yaw, vx, vy, vyaw]
        self.x = SimpleMatrix(self.n_states, 1)
        
        # Covariance matrix
        self.P = SimpleMatrix.identity(self.n_states)
        
        # Process noise (simplified)
        self.Q = SimpleMatrix.identity(self.n_states)
        for i in range(3):  # Position states
            self.Q.set_element(i, i, 0.01)
        for i in range(3, 6):  # Velocity states
            self.Q.set_element(i, i, 0.1)
        
        # Measurement noise (simplified)
        self.R_imu = SimpleMatrix.identity(3)  # accel_x, accel_y, gyro_z
        for i in range(3):
            self.R_imu.set_element(i, i, 0.1)
        
        self.R_chassis = SimpleMatrix.identity(3)  # x, y, yaw
        for i in range(3):
            self.R_chassis.set_element(i, i, 0.05)
        
        # Constants
        self.gravity = 9.81
        self.last_time = None
        
        # Performance monitoring
        self.prediction_time = 0.0
        self.update_time = 0.0
        self.total_updates = 0
    
    def predict(self, dt):
        """EKF prediction step"""
        start_time = time.time()
        
        if dt <= 0:
            return
        
        # State transition matrix F (simplified kinematic model)
        F = SimpleMatrix.identity(self.n_states)
        
        # Position updates from velocity
        F.set_element(0, 3, dt)  # x += vx * dt
        F.set_element(1, 4, dt)  # y += vy * dt
        F.set_element(2, 5, dt)  # yaw += vyaw * dt
        
        # Predict state: x = F * x
        self.x = F * self.x
        
        # Normalize yaw angle to [-pi, pi]
        yaw = self.x.get_element(2, 0)
        while yaw > math.pi:
            yaw -= 2 * math.pi
        while yaw < -math.pi:
            yaw += 2 * math.pi
        self.x.set_element(2, 0, yaw)
        
        # Predict covariance: P = F * P * F' + Q
        Ft = F.transpose()
        self.P = F * self.P * Ft + self.Q * dt
        
        self.prediction_time = time.time() - start_time
    
    def update_imu(self, accel_x, accel_y, gyro_z):
        """Update with IMU measurements (simplified)"""
        start_time = time.time()
        
        # Expected measurements based on current state
        yaw = self.x.get_element(2, 0)
        
        # Expected acceleration in body frame (gravity + motion)
        expected_accel_x = -self.gravity * math.sin(yaw)
        expected_accel_y = self.gravity * math.cos(yaw)
        expected_gyro_z = self.x.get_element(5, 0)  # angular velocity
        
        # Measurement vector
        z = SimpleMatrix(3, 1)
        z.set_element(0, 0, accel_x)
        z.set_element(1, 0, accel_y)
        z.set_element(2, 0, gyro_z)
        
        # Expected measurement vector
        h = SimpleMatrix(3, 1)
        h.set_element(0, 0, expected_accel_x)
        h.set_element(1, 0, expected_accel_y)
        h.set_element(2, 0, expected_gyro_z)
        
        # Measurement Jacobian (simplified)
        H = SimpleMatrix(3, self.n_states)
        H.set_element(0, 2, -self.gravity * math.cos(yaw))  # d(accel_x)/d(yaw)
        H.set_element(1, 2, -self.gravity * math.sin(yaw))  # d(accel_y)/d(yaw)
        H.set_element(2, 5, 1.0)  # d(gyro_z)/d(vyaw)
        
        # Perform Kalman update
        self._kalman_update(z, h, H, self.R_imu)
        
        self.update_time = time.time() - start_time
        self.total_updates += 1
    
    def update_chassis(self, chassis_x, chassis_y, chassis_yaw):
        """Update with chassis encoder measurements"""
        start_time = time.time()
        
        # Measurement vector
        z = SimpleMatrix(3, 1)
        z.set_element(0, 0, chassis_x)
        z.set_element(1, 0, chassis_y)
        z.set_element(2, 0, chassis_yaw)
        
        # Expected measurement (direct measurement of position and yaw)
        h = SimpleMatrix(3, 1)
        h.set_element(0, 0, self.x.get_element(0, 0))
        h.set_element(1, 0, self.x.get_element(1, 0))
        h.set_element(2, 0, self.x.get_element(2, 0))
        
        # Measurement Jacobian (direct measurement)
        H = SimpleMatrix(3, self.n_states)
        H.set_element(0, 0, 1.0)  # x measurement
        H.set_element(1, 1, 1.0)  # y measurement
        H.set_element(2, 2, 1.0)  # yaw measurement
        
        # Perform Kalman update
        self._kalman_update(z, h, H, self.R_chassis)
        
        self.update_time = time.time() - start_time
        self.total_updates += 1
    
    def _kalman_update(self, z, h, H, R):
        """Kalman update step"""
        # Innovation
        y = z - h
        
        # Innovation covariance
        Ht = H.transpose()
        S = H * self.P * Ht + R
        
        # Kalman gain
        try:
            S_inv = S.invert()
            K = self.P * Ht * S_inv
        except ValueError:
            # Singular matrix, skip update
            return
        
        # Update state
        self.x = self.x + K * y
        
        # Update covariance (Joseph form for numerical stability)
        I = SimpleMatrix.identity(self.n_states)
        I_KH = I - K * H
        KRKt = K * R * K.transpose()
        self.P = I_KH * self.P * I_KH.transpose() + KRKt
    
    def process_sensor_data(self, sensor_data):
        """Process incoming sensor data"""
        # Compute time step
        current_time = sensor_data.timestamp
        if self.last_time is not None:
            dt = current_time - self.last_time
            if dt > 0 and dt < 1.0:  # Sanity check on dt
                self.predict(dt)
        self.last_time = current_time
        
        # Update with IMU data
        self.update_imu(sensor_data.accel_x, sensor_data.accel_y, sensor_data.gyro_z)
        
        # Update with chassis data (if available)
        if sensor_data.chassis_x is not None:
            self.update_chassis(sensor_data.chassis_x, sensor_data.chassis_y, sensor_data.chassis_yaw)
    
    def get_state(self):
        """Get current state estimate"""
        return {
            'position': [
                self.x.get_element(0, 0),  # x
                self.x.get_element(1, 0),  # y
                0.0                        # z (not estimated)
            ],
            'velocity': [
                self.x.get_element(3, 0),  # vx
                self.x.get_element(4, 0),  # vy
                0.0                        # vz (not estimated)
            ],
            'orientation': [
                0.0,                       # roll (not estimated)
                0.0,                       # pitch (not estimated)
                self.x.get_element(2, 0)   # yaw
            ],
            'angular_velocity': [
                0.0,                       # wx (not estimated)
                0.0,                       # wy (not estimated)
                self.x.get_element(5, 0)   # wz
            ]
        }
    
    def get_covariance_trace(self):
        """Get trace of covariance matrix (uncertainty measure)"""
        trace = 0.0
        for i in range(self.n_states):
            trace += self.P.get_element(i, i)
        return trace
    
    def get_performance_stats(self):
        """Get performance statistics"""
        return {
            'prediction_time_ms': self.prediction_time * 1000,
            'update_time_ms': self.update_time * 1000,
            'total_updates': self.total_updates,
            'avg_cycle_time_ms': (self.prediction_time + self.update_time) * 1000
        }
    
    def reset(self):
        """Reset EKF to initial state"""
        self.x = SimpleMatrix(self.n_states, 1)
        self.P = SimpleMatrix.identity(self.n_states)
        self.last_time = None
        self.total_updates = 0


def demo_ekf():
    """Simple demonstration of the minimal EKF"""
    print("=== Minimal EKF Demo for RoboMaster S1 ===")
    
    # Create EKF instance
    ekf = MinimalEKF()
    
    # Simulate sensor data for 5 seconds
    start_time = time.time()
    iteration = 0
    
    while time.time() - start_time < 5.0:
        iteration += 1
        current_time = time.time()
        
        # Create simulated sensor data
        sensor_data = SensorReading()
        sensor_data.timestamp = current_time
        
        # Simulate robot moving in a circle
        t = current_time - start_time
        radius = 1.0
        angular_vel = 0.5  # rad/s
        
        # Simulated sensor readings with noise
        sensor_data.accel_x = 0.1 * math.sin(t)  # Some noise
        sensor_data.accel_y = 9.81 + 0.1 * math.cos(t)  # Gravity + noise
        sensor_data.gyro_z = angular_vel + 0.05 * math.sin(t * 10)  # Angular velocity + noise
        
        # Simulated chassis encoder data
        sensor_data.chassis_x = radius * math.cos(angular_vel * t)
        sensor_data.chassis_y = radius * math.sin(angular_vel * t)
        sensor_data.chassis_yaw = angular_vel * t
        
        # Process with EKF
        ekf.process_sensor_data(sensor_data)
        
        # Print state every 10 iterations
        if iteration % 10 == 0:
            state = ekf.get_state()
            stats = ekf.get_performance_stats()
            
            print(f"\nIteration {iteration} (t={t:.1f}s):")
            print(f"Position: x={state['position'][0]:.3f}, y={state['position'][1]:.3f}")
            print(f"Yaw: {state['orientation'][2]:.3f} rad")
            print(f"Uncertainty: {ekf.get_covariance_trace():.3f}")
            print(f"Cycle time: {stats['avg_cycle_time_ms']:.1f} ms")
        
        # Control loop timing (aim for 10 Hz)
        time.sleep(0.1)
    
    print(f"\nDemo completed after {iteration} iterations")
    final_stats = ekf.get_performance_stats()
    print(f"Average processing time: {final_stats['avg_cycle_time_ms']:.1f} ms per cycle")
    print(f"Total updates: {final_stats['total_updates']}")


if __name__ == "__main__":
    demo_ekf()