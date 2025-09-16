# Step 4: Simple Sensor Fusion
# Combines IMU and chassis data without full EKF

import time
import math

print("=" * 40)
print("STEP 4: SIMPLE FUSION")
print("=" * 40)

robot_ctrl.set_mode(rm_define.robot_mode_gimbal_follow)

# State variables
x = 0.0
y = 0.0
roll = 0.0
pitch = 0.0
yaw = 0.0

# Subscribe to sensors
sensor_imu.sub_gyroscope(freq=5)
sensor_imu.sub_accelerometer(freq=5)
chassis_ctrl.sub_position(freq=5)

print("Testing simple fusion for 20 seconds...")

start_time = time.time()
last_time = start_time
iteration = 0

while time.time() - start_time < 20:
    current_time = time.time()
    dt = current_time - last_time
    
    if dt < 0.2:  # 5Hz
        time.sleep(0.05)
        continue
    
    # Get all sensor data
    gyro = sensor_imu.get_gyroscope()
    accel = sensor_imu.get_accelerometer()
    pos = chassis_ctrl.get_position()
    
    # Update orientation from IMU
    if gyro and dt > 0:
        roll += math.radians(gyro[0]) * dt
        pitch += math.radians(gyro[1]) * dt
        yaw += math.radians(gyro[2]) * dt
    
    # Correct with accelerometer
    if accel:
        ax, ay, az = accel[0], accel[1], accel[2]
        if abs(az) > 0.1:
            accel_roll = math.atan2(ay, az)
            accel_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
            roll = 0.9 * roll + 0.1 * accel_roll
            pitch = 0.9 * pitch + 0.1 * accel_pitch
    
    # Update position from chassis
    if pos:
        x = pos[0] / 1000.0  # Convert to meters
        y = pos[1] / 1000.0
        if len(pos) >= 3:
            # Use chassis yaw with high trust
            chassis_yaw = math.radians(pos[2])
            yaw = 0.8 * chassis_yaw + 0.2 * yaw
    
    # Normalize angles
    for angle in [roll, pitch, yaw]:
        while angle > math.pi: angle -= 2*math.pi
        while angle < -math.pi: angle += 2*math.pi
    
    iteration += 1
    
    # Print status
    if iteration % 5 == 0:
        elapsed = current_time - start_time
        print("T=" + str(round(elapsed, 1)) + "s, Iter=" + str(iteration))
        print("  Pos: " + str(round(x, 3)) + ", " + str(round(y, 3)) + " m")
        print("  Ang: R=" + str(round(math.degrees(roll), 1)) + " P=" + str(round(math.degrees(pitch), 1)) + " Y=" + str(round(math.degrees(yaw), 1)) + " deg")
        
        # LED feedback
        distance = math.sqrt(x*x + y*y)
        if distance > 0.1:
            led_ctrl.set_flash(rm_define.armor_all, 3)
        else:
            led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 0, 0, 255, rm_define.effect_always_on)
    
    last_time = current_time

# Cleanup
sensor_imu.unsub_gyroscope()
sensor_imu.unsub_accelerometer()
chassis_ctrl.unsub_position()

print("=" * 40)
print("SIMPLE FUSION COMPLETE")
print("Total iterations: " + str(iteration))
print("Final state:")
print("  Position: " + str(round(x, 3)) + ", " + str(round(y, 3)) + " m")
print("  Orientation: R=" + str(round(math.degrees(roll), 1)) + " P=" + str(round(math.degrees(pitch), 1)) + " Y=" + str(round(math.degrees(yaw), 1)) + " deg")
print("=" * 40)
