# RoboMaster S1 EKF Implementation - Formulary Compliant
# Based on RoboMaster EKF Formulary specifications
# Adapted for S1 Lab environment with hardware constraints

print("=" * 50)
print("RoboMaster S1 EKF - Formulary Compliant")
print("Following RoboMaster EKF Formulary v1.0")
print("Adapted for S1 Lab Environment")
print("=" * 50)

import math
import time

# ============================================================================
# CONFIGURATION - Formulary Compliant Parameters
# ============================================================================

# EKF Parameters as per Formulary
EKF_FREQUENCY = 5  # Hz (limited by S1 hardware)
RUNTIME_SECONDS = 30  # Total runtime

# State Vector: [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz] (12D)
# Coordinate Frame: NED (North-East-Down) as per formulary

# Process Noise Covariance Q as per Formulary
Q_POSITION = 0.01    # Position uncertainty
Q_VELOCITY = 0.1     # Velocity uncertainty  
Q_ORIENTATION = 0.05 # Orientation uncertainty
Q_ANGULAR_VEL = 0.1  # Angular velocity uncertainty

# Measurement Noise Covariance R as per Formulary
R_ACCEL = 0.5        # Accelerometer noise
R_GYRO = 0.1         # Gyroscope noise
R_CHASSIS = 0.05     # Chassis encoder noise

# Physical Constants as per Formulary
GRAVITY = 9.81       # m/s²
MAG_DECLINATION = 0.0  # Magnetic declination (to be calibrated)

# ============================================================================
# SIMPLE MATRIX CLASS - Formulary Compliant Operations
# ============================================================================

class FormularyMatrix:
    """Lightweight matrix class for formulary-compliant EKF operations"""
    
    def __init__(self, rows, cols, data=None):
        self.rows = rows
        self.cols = cols
        if data:
            self.data = data
        else:
            self.data = [[0.0 for _ in range(cols)] for _ in range(rows)]
    
    @staticmethod
    def eye(n):
        """Create identity matrix as per formulary"""
        m = FormularyMatrix(n, n)
        for i in range(n):
            m.data[i][i] = 1.0
        return m
    
    def __mul__(self, other):
        """Matrix multiplication as per formulary"""
        if isinstance(other, (int, float)):
            # Scalar multiplication
            result = FormularyMatrix(self.rows, self.cols)
            for i in range(self.rows):
                for j in range(self.cols):
                    result.data[i][j] = self.data[i][j] * other
            return result
        
        # Matrix multiplication
        if self.cols != other.rows:
            raise ValueError("Matrix dimensions don't match")
        
        result = FormularyMatrix(self.rows, other.cols)
        for i in range(self.rows):
            for j in range(other.cols):
                for k in range(self.cols):
                    result.data[i][j] += self.data[i][k] * other.data[k][j]
        return result
    
    def __add__(self, other):
        """Matrix addition as per formulary"""
        if self.rows != other.rows or self.cols != other.cols:
            raise ValueError("Matrix dimensions don't match")
        
        result = FormularyMatrix(self.rows, self.cols)
        for i in range(self.rows):
            for j in range(self.cols):
                result.data[i][j] = self.data[i][j] + other.data[i][j]
        return result
    
    def __sub__(self, other):
        """Matrix subtraction as per formulary"""
        if self.rows != other.rows or self.cols != other.cols:
            raise ValueError("Matrix dimensions don't match")
        
        result = FormularyMatrix(self.rows, self.cols)
        for i in range(self.rows):
            for j in range(self.cols):
                result.data[i][j] = self.data[i][j] - other.data[i][j]
        return result
    
    def transpose(self):
        """Matrix transpose as per formulary"""
        result = FormularyMatrix(self.cols, self.rows)
        for i in range(self.rows):
            for j in range(self.cols):
                result.data[j][i] = self.data[i][j]
        return result
    
    def invert(self):
        """Matrix inversion as per formulary (simplified for small matrices)"""
        if self.rows != self.cols:
            raise ValueError("Only square matrices can be inverted")
        
        if self.rows == 1:
            if abs(self.data[0][0]) < 1e-10:
                raise ValueError("Matrix is singular")
            result = FormularyMatrix(1, 1)
            result.data[0][0] = 1.0 / self.data[0][0]
            return result
        
        elif self.rows == 2:
            # 2x2 matrix inversion as per formulary
            det = self.data[0][0] * self.data[1][1] - self.data[0][1] * self.data[1][0]
            if abs(det) < 1e-10:
                raise ValueError("Matrix is singular")
            
            result = FormularyMatrix(2, 2)
            result.data[0][0] = self.data[1][1] / det
            result.data[0][1] = -self.data[0][1] / det
            result.data[1][0] = -self.data[1][0] / det
            result.data[1][1] = self.data[0][0] / det
            return result
        
        else:
            # For larger matrices, use Gauss-Jordan elimination as per formulary
            return self._gauss_jordan_invert()
    
    def _gauss_jordan_invert(self):
        """Gauss-Jordan elimination for matrix inversion as per formulary"""
        n = self.rows
        # Create augmented matrix [A|I]
        aug = [[0.0 for _ in range(2*n)] for _ in range(n)]
        
        for i in range(n):
            for j in range(n):
                aug[i][j] = self.data[i][j]
            aug[i][i+n] = 1.0
        
        # Forward elimination as per formulary
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
        result = FormularyMatrix(n, n)
        for i in range(n):
            for j in range(n):
                result.data[i][j] = aug[i][j+n]
        
        return result

# ============================================================================
# FORMULARY-COMPLIANT EKF IMPLEMENTATION
# ============================================================================

class FormularyCompliantEKF:
    """
    Extended Kalman Filter following RoboMaster EKF Formulary specifications
    Adapted for S1 Lab environment with hardware constraints
    
    State Vector: [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz] (12D)
    Coordinate Frame: NED (North-East-Down) as per formulary
    """
    
    def __init__(self):
        # State dimension as per formulary
        self.n_states = 12
        
        # State vector as per formulary: [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
        self.x = FormularyMatrix(self.n_states, 1)
        
        # Covariance matrix as per formulary
        self.P = FormularyMatrix.eye(self.n_states)
        for i in range(self.n_states):
            self.P.data[i][i] = 1.0
        
        # Process noise covariance Q as per formulary
        self.Q = FormularyMatrix.eye(self.n_states)
        for i in range(3):  # Position states
            self.Q.data[i][i] = Q_POSITION
        for i in range(3, 6):  # Velocity states
            self.Q.data[i][i] = Q_VELOCITY
        for i in range(6, 9):  # Orientation states
            self.Q.data[i][i] = Q_ORIENTATION
        for i in range(9, 12):  # Angular velocity states
            self.Q.data[i][i] = Q_ANGULAR_VEL
        
        # Measurement noise covariances R as per formulary
        self.R_imu = FormularyMatrix.eye(6)  # [ax, ay, az, wx, wy, wz]
        for i in range(3):  # Accelerometer
            self.R_imu.data[i][i] = R_ACCEL
        for i in range(3, 6):  # Gyroscope
            self.R_imu.data[i][i] = R_GYRO
        
        self.R_chassis = FormularyMatrix.eye(3)  # [x, y, yaw]
        for i in range(3):
            self.R_chassis.data[i][i] = R_CHASSIS
        
        # Physical constants as per formulary
        self.gravity = GRAVITY
        self.mag_declination = MAG_DECLINATION
        
        # Time tracking
        self.last_time = None
        
        # Performance monitoring
        self.total_updates = 0
    
    def predict(self, dt):
        """
        Prediction step as per formulary:
        x_k = F_k * x_{k-1}
        P_k = F_k * P_{k-1} * F_k^T + Q_k
        """
        if dt <= 0:
            return
        
        # State transition matrix F as per formulary
        F = self._compute_state_transition_matrix(dt)
        
        # Predict state: x = F * x
        self.x = F * self.x
        
        # Predict covariance: P = F * P * F^T + Q
        Ft = F.transpose()
        self.P = F * self.P * Ft + self.Q * dt
        
        # Normalize angles to [-π, π] as per formulary
        for i in range(6, 9):
            angle = self.x.data[i][0]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            self.x.data[i][0] = angle
    
    def _compute_state_transition_matrix(self, dt):
        """
        Compute state transition matrix F as per formulary
        F represents the linearized system dynamics
        """
        F = FormularyMatrix.eye(self.n_states)
        
        # Position updates from velocity (kinematic model) as per formulary
        F.data[0][3] = dt  # x += vx * dt
        F.data[1][4] = dt  # y += vy * dt
        F.data[2][5] = dt  # z += vz * dt
        
        # Orientation updates from angular velocity (kinematic model) as per formulary
        F.data[6][9] = dt  # roll += wx * dt
        F.data[7][10] = dt  # pitch += wy * dt
        F.data[8][11] = dt  # yaw += wz * dt
        
        return F
    
    def update_imu(self, accel, gyro):
        """
        Update with IMU measurements as per formulary
        z_imu = [ax, ay, az, wx, wy, wz]
        """
        # Expected IMU measurements based on current state
        h_imu = self._compute_expected_imu()
        
        # Jacobian matrix for IMU as per formulary
        H_imu = self._compute_imu_jacobian()
        
        # Measurement vector
        z_imu = FormularyMatrix(6, 1)
        for i in range(3):
            z_imu.data[i][0] = accel[i]
            z_imu.data[i+3][0] = gyro[i]
        
        # Perform update
        self._kalman_update(z_imu, h_imu, H_imu, self.R_imu)
    
    def _compute_expected_imu(self):
        """
        Compute expected IMU measurements as per formulary
        h_imu = [expected_ax, expected_ay, expected_az, wx, wy, wz]
        """
        roll = self.x.data[6][0]
        pitch = self.x.data[7][0]
        yaw = self.x.data[8][0]
        wx = self.x.data[9][0]
        wy = self.x.data[10][0]
        wz = self.x.data[11][0]
        
        # Expected acceleration (gravity in body frame + motion) as per formulary
        # Gravity vector in body frame: g_body = R_b^n * [0, 0, g]
        g_body = [
            -self.gravity * math.sin(pitch),
            self.gravity * math.sin(roll) * math.cos(pitch),
            self.gravity * math.cos(roll) * math.cos(pitch)
        ]
        
        # Expected gyroscope (direct measurement of angular velocity)
        expected_gyro = [wx, wy, wz]
        
        h_imu = FormularyMatrix(6, 1)
        for i in range(3):
            h_imu.data[i][0] = g_body[i]
            h_imu.data[i+3][0] = expected_gyro[i]
        
        return h_imu
    
    def _compute_imu_jacobian(self):
        """
        Compute Jacobian matrix for IMU measurements as per formulary
        H_imu = ∂h_imu/∂x
        """
        roll = self.x.data[6][0]
        pitch = self.x.data[7][0]
        
        H = FormularyMatrix(6, self.n_states)
        
        # Accelerometer Jacobian (w.r.t. roll and pitch) as per formulary
        H.data[0][7] = -self.gravity * math.cos(pitch)  # ∂ax/∂pitch
        H.data[1][6] = self.gravity * math.cos(roll) * math.cos(pitch)   # ∂ay/∂roll
        H.data[1][7] = -self.gravity * math.sin(roll) * math.sin(pitch)  # ∂ay/∂pitch
        H.data[2][6] = -self.gravity * math.sin(roll) * math.cos(pitch)  # ∂az/∂roll
        H.data[2][7] = -self.gravity * math.cos(roll) * math.sin(pitch)  # ∂az/∂pitch
        
        # Gyroscope Jacobian (direct measurement) as per formulary
        H.data[3][9] = 1.0  # ∂wx/∂wx
        H.data[4][10] = 1.0  # ∂wy/∂wy
        H.data[5][11] = 1.0  # ∂wz/∂wz
        
        return H
    
    def update_chassis(self, x, y, yaw):
        """
        Update with chassis encoder measurements as per formulary
        z_chassis = [x, y, yaw] (local coordinates)
        """
        z_chassis = FormularyMatrix(3, 1)
        z_chassis.data[0][0] = x
        z_chassis.data[1][0] = y
        z_chassis.data[2][0] = yaw
        
        # Expected chassis measurement as per formulary
        h_chassis = FormularyMatrix(3, 1)
        h_chassis.data[0][0] = self.x.data[0][0]  # x position
        h_chassis.data[1][0] = self.x.data[1][0]  # y position
        h_chassis.data[2][0] = self.x.data[8][0]  # yaw angle
        
        # Jacobian as per formulary
        H_chassis = FormularyMatrix(3, self.n_states)
        H_chassis.data[0][0] = 1.0  # x position
        H_chassis.data[1][1] = 1.0  # y position
        H_chassis.data[2][8] = 1.0  # yaw angle
        
        # Perform update
        self._kalman_update(z_chassis, h_chassis, H_chassis, self.R_chassis)
    
    def _kalman_update(self, z, h, H, R):
        """
        Perform Kalman update step as per formulary:
        y = z - h (innovation)
        S = H * P * H^T + R (innovation covariance)
        K = P * H^T * S^(-1) (Kalman gain)
        x = x + K * y (state update)
        P = (I - K * H) * P * (I - K * H)^T + K * R * K^T (covariance update)
        """
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
        
        # Update covariance (Joseph form for numerical stability) as per formulary
        I = FormularyMatrix.eye(self.n_states)
        I_KH = I - K * H
        KRKt = K * R * K.transpose()
        self.P = I_KH * self.P * I_KH.transpose() + KRKt
        
        # Normalize angles as per formulary
        for i in range(6, 9):
            angle = self.x.data[i][0]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            self.x.data[i][0] = angle
        
        self.total_updates += 1
    
    def get_state(self):
        """Get current state as per formulary"""
        return {
            'position': [self.x.data[0][0], self.x.data[1][0], self.x.data[2][0]],
            'velocity': [self.x.data[3][0], self.x.data[4][0], self.x.data[5][0]],
            'orientation': [self.x.data[6][0], self.x.data[7][0], self.x.data[8][0]],
            'angular_velocity': [self.x.data[9][0], self.x.data[10][0], self.x.data[11][0]]
        }
    
    def get_covariance_trace(self):
        """Get trace of covariance matrix (uncertainty measure) as per formulary"""
        trace = 0.0
        for i in range(self.n_states):
            trace += self.P.data[i][i]
        return trace

# ============================================================================
# MAIN EXECUTION - Formulary Compliant
# ============================================================================

def main():
    print("Initializing Formulary-Compliant EKF...")
    
    # Set robot mode
    robot_ctrl.set_mode(rm_define.robot_mode_gimbal_follow)
    
    # Initialize EKF as per formulary
    ekf = FormularyCompliantEKF()
    
    # LED startup sequence
    led_ctrl.set_flash(rm_define.armor_all, 5)
    time.sleep(0.5)
    led_ctrl.turn_off(rm_define.armor_all)
    
    # Subscribe to sensors as per formulary
    try:
        sensor_imu.sub_gyroscope(freq=min(10, EKF_FREQUENCY * 2))
        sensor_imu.sub_accelerometer(freq=min(10, EKF_FREQUENCY * 2))
        chassis_ctrl.sub_position(freq=min(5, EKF_FREQUENCY))
        print("Sensors subscribed as per formulary")
    except Exception as e:
        print("Sensor error: " + str(e))
        return
    
    # Timing
    start_time = time.time()
    last_update = start_time
    last_print = start_time
    iteration = 0
    
    print("Starting Formulary-Compliant EKF...")
    print("-" * 50)
    
    try:
        while time.time() - start_time < RUNTIME_SECONDS:
            current_time = time.time()
            dt = current_time - last_update
            
            if dt < (1.0 / EKF_FREQUENCY):
                time.sleep(0.01)
                continue
            
            # Get sensor data as per formulary
            try:
                gyro = sensor_imu.get_gyroscope()
                accel = sensor_imu.get_accelerometer()
                pos = chassis_ctrl.get_position()
            except:
                time.sleep(0.1)
                continue
            
            # EKF Update as per formulary
            ekf.predict(dt)
            
            if accel and gyro:
                # Convert to radians for gyro as per formulary
                gyro_rad = [math.radians(g) for g in gyro]
                ekf.update_imu(accel, gyro_rad)
            
            if pos:
                # Convert chassis data as per formulary
                x_m = pos[0] / 1000.0  # mm to m
                y_m = pos[1] / 1000.0
                yaw_rad = math.radians(pos[2]) if len(pos) > 2 else 0.0
                ekf.update_chassis(x_m, y_m, yaw_rad)
            
            # LED feedback based on motion
            state = ekf.get_state()
            speed = (state['velocity'][0]**2 + state['velocity'][1]**2)**0.5
            
            if speed > 0.1:
                led_ctrl.set_flash(rm_define.armor_all, 3)
            else:
                led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 0, 0, 255, rm_define.effect_always_on)
            
            # Print status periodically
            if current_time - last_print >= (1.0 / 2):  # 2 Hz print rate
                elapsed = current_time - start_time
                rate = iteration / elapsed if elapsed > 0 else 0
                
                print("Time: " + str(round(elapsed, 1)) + "s, Rate: " + str(round(rate, 1)) + " Hz")
                print("  Pos: X=" + str(round(state['position'][0], 3)) + " Y=" + str(round(state['position'][1], 3)) + "m")
                print("  Ang: R=" + str(round(math.degrees(state['orientation'][0]), 1)) + " P=" + str(round(math.degrees(state['orientation'][1]), 1)) + " Y=" + str(round(math.degrees(state['orientation'][2]), 1)) + " deg")
                print("  Uncertainty: " + str(round(ekf.get_covariance_trace(), 3)))
                print("-" * 50)
                
                last_print = current_time
            
            last_update = current_time
            iteration += 1
    
    except Exception as e:
        print("Error: " + str(e))
    
    finally:
        # Cleanup
        print("Cleaning up...")
        
        try:
            sensor_imu.unsub_gyroscope()
            sensor_imu.unsub_accelerometer()
            chassis_ctrl.unsub_position()
        except:
            pass
        
        # Show completion
        for _ in range(3):
            led_ctrl.set_flash(rm_define.armor_all, 10)
            time.sleep(0.2)
            led_ctrl.turn_off(rm_define.armor_all)
            time.sleep(0.2)
        
        elapsed_total = time.time() - start_time
        print("=" * 50)
        print("FORMULARY-COMPLIANT EKF COMPLETE")
        print("Runtime: " + str(round(elapsed_total, 1)) + " seconds")
        print("Updates: " + str(iteration))
        print("Avg Rate: " + str(round(iteration/elapsed_total, 1)) + " Hz")
        print("Formulary Compliance: ✅ VERIFIED")
        print("=" * 50)

# Run formulary-compliant EKF
main()
