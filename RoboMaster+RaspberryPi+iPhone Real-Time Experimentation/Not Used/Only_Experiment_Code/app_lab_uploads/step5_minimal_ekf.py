# Step 5: Minimal EKF
# Simplest possible EKF implementation

import time
import math

print("=" * 40)
print("STEP 5: MINIMAL EKF")
print("=" * 40)

robot_ctrl.set_mode(rm_define.robot_mode_gimbal_follow)

# EKF State [x, y, yaw]
state = [0.0, 0.0, 0.0]

# Simple covariance (diagonal only)
P = [1.0, 1.0, 1.0]

# Process noise
Q = 0.01

# Measurement noise
R_chassis = 0.05
R_gyro = 0.1

# Subscribe
sensor_imu.sub_gyroscope(freq=5)
chassis_ctrl.sub_position(freq=5)

print("Running minimal EKF for 20 seconds...")

start_time = time.time()
last_time = start_time
iteration = 0

while time.time() - start_time < 20:
    current_time = time.time()
    dt = current_time - last_time
    
    if dt < 0.2:
        time.sleep(0.05)
        continue
    
    # PREDICT
    # Simple motion model (constant position, integrate yaw)
    gyro = sensor_imu.get_gyroscope()
    if gyro and dt > 0:
        # Predict yaw from gyro
        state[2] += math.radians(gyro[2]) * dt
        
        # Increase uncertainty
        for i in range(3):
            P[i] += Q * dt
    
    # UPDATE
    pos = chassis_ctrl.get_position()
    if pos:
        # Measurement
        z_x = pos[0] / 1000.0
        z_y = pos[1] / 1000.0
        z_yaw = math.radians(pos[2]) if len(pos) > 2 else state[2]
        
        # Innovation
        y_x = z_x - state[0]
        y_y = z_y - state[1]
        y_yaw = z_yaw - state[2]
        
        # Normalize yaw innovation
        while y_yaw > math.pi: y_yaw -= 2*math.pi
        while y_yaw < -math.pi: y_yaw += 2*math.pi
        
        # Kalman gain (simplified)
        K_x = P[0] / (P[0] + R_chassis)
        K_y = P[1] / (P[1] + R_chassis)
        K_yaw = P[2] / (P[2] + R_gyro)
        
        # State update
        state[0] += K_x * y_x
        state[1] += K_y * y_y
        state[2] += K_yaw * y_yaw
        
        # Covariance update
        P[0] *= (1 - K_x)
        P[1] *= (1 - K_y)
        P[2] *= (1 - K_yaw)
    
    # Normalize yaw
    while state[2] > math.pi: state[2] -= 2*math.pi
    while state[2] < -math.pi: state[2] += 2*math.pi
    
    iteration += 1
    
    # Print
    if iteration % 5 == 0:
        elapsed = current_time - start_time
        print("T=" + str(round(elapsed, 1)) + "s")
        print("  State: X=" + str(round(state[0], 3)) + " Y=" + str(round(state[1], 3)) + " Yaw=" + str(round(math.degrees(state[2]), 1)))
        print("  Uncertainty: " + str(round(P[0], 3)) + ", " + str(round(P[1], 3)) + ", " + str(round(P[2], 3)))
        
        # LED
        if P[0] < 0.5 and P[1] < 0.5:
            led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 0, 255, 0, rm_define.effect_always_on)
        else:
            led_ctrl.set_flash(rm_define.armor_all, 2)
    
    last_time = current_time

# Cleanup
sensor_imu.unsub_gyroscope()
chassis_ctrl.unsub_position()

print("=" * 40)
print("MINIMAL EKF COMPLETE")
print("Iterations: " + str(iteration))
print("Final: X=" + str(round(state[0], 3)) + " Y=" + str(round(state[1], 3)) + " Yaw=" + str(round(math.degrees(state[2]), 1)))
print("=" * 40)
