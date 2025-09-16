# Step 2: IMU Fusion Test
# Tests accelerometer and gyroscope fusion

import time
import math

print("=" * 40)
print("STEP 2: IMU FUSION")
print("=" * 40)

robot_ctrl.set_mode(rm_define.robot_mode_gimbal_follow)

# Simple orientation variables
roll = 0.0
pitch = 0.0
yaw = 0.0

# Subscribe to IMU
sensor_imu.sub_gyroscope(freq=5)
sensor_imu.sub_accelerometer(freq=5)

print("Testing IMU fusion for 20 seconds...")
print("Move the robot to see orientation changes")

start_time = time.time()
last_time = start_time
iteration = 0

while time.time() - start_time < 20:
    current_time = time.time()
    dt = current_time - last_time
    
    if dt < 0.2:  # 5Hz
        time.sleep(0.05)
        continue
    
    # Get IMU data
    gyro = sensor_imu.get_gyroscope()
    accel = sensor_imu.get_accelerometer()
    
    if gyro and accel and dt > 0:
        # Gyro integration
        roll += math.radians(gyro[0]) * dt
        pitch += math.radians(gyro[1]) * dt
        yaw += math.radians(gyro[2]) * dt
        
        # Accel correction for roll/pitch
        ax, ay, az = accel[0], accel[1], accel[2]
        if abs(az) > 0.1:
            accel_roll = math.atan2(ay, az)
            accel_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
            
            # Simple complementary filter
            roll = 0.9 * roll + 0.1 * accel_roll
            pitch = 0.9 * pitch + 0.1 * accel_pitch
        
        # Normalize angles
        while roll > math.pi: roll -= 2*math.pi
        while roll < -math.pi: roll += 2*math.pi
        while pitch > math.pi: pitch -= 2*math.pi
        while pitch < -math.pi: pitch += 2*math.pi
        while yaw > math.pi: yaw -= 2*math.pi
        while yaw < -math.pi: yaw += 2*math.pi
        
        iteration += 1
        
        # Print every 5 iterations
        if iteration % 5 == 0:
            elapsed = current_time - start_time
            print("T=" + str(round(elapsed, 1)) + "s:")
            print("  Roll=" + str(round(math.degrees(roll), 1)) + " deg")
            print("  Pitch=" + str(round(math.degrees(pitch), 1)) + " deg")
            print("  Yaw=" + str(round(math.degrees(yaw), 1)) + " deg")
            
            # LED feedback
            if abs(roll) > 0.1 or abs(pitch) > 0.1:
                led_ctrl.set_flash(rm_define.armor_all, 5)
            else:
                led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 0, 0, 255, rm_define.effect_always_on)
    
    last_time = current_time

# Cleanup
sensor_imu.unsub_gyroscope()
sensor_imu.unsub_accelerometer()

print("=" * 40)
print("IMU FUSION TEST COMPLETE")
print("Total iterations: " + str(iteration))
print("Final orientation:")
print("  Roll=" + str(round(math.degrees(roll), 1)) + " deg")
print("  Pitch=" + str(round(math.degrees(pitch), 1)) + " deg")
print("  Yaw=" + str(round(math.degrees(yaw), 1)) + " deg")
print("=" * 40)
