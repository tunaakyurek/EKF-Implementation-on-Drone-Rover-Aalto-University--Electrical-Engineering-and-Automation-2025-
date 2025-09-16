# RoboMaster S1 Simple EKF - Lab Environment (FIXED)
# This uses the correct on-device Lab API syntax

import time
import math

print("=== S1 EKF Demo - On-Device Lab Environment ===")
print("Fixed for RoboMaster app Lab APIs")

# Set robot mode
robot_ctrl.set_mode(rm_define.robot_mode_gimbal_follow)

# Simple state variables
roll_estimate = 0.0
pitch_estimate = 0.0
yaw_estimate = 0.0
x_estimate = 0.0
y_estimate = 0.0

# Filter weights
gyro_weight = 0.9
accel_weight = 0.1

print("Starting sensor data collection...")

# Subscribe to sensor data at low frequency
chassis_ctrl.sub_position(freq=2)
sensor_imu.sub_gyroscope(freq=3)
sensor_imu.sub_accelerometer(freq=3)

start_time = time.time()
iteration = 0

try:
    # Run for 20 seconds
    while time.time() - start_time < 20:
        current_time = time.time()
        elapsed = current_time - start_time
        
        # Get sensor data using on-device APIs
        try:
            # Get IMU data
            gyro_data = sensor_imu.get_gyroscope()
            accel_data = sensor_imu.get_accelerometer()
            
            # Get chassis position
            pos_data = chassis_ctrl.get_position()
            
        except:
            print(f"Iteration {iteration}: Sensor read failed")
            time.sleep(0.33)
            continue
        
        # Process gyroscope data
        if gyro_data:
            dt = 0.33  # ~3Hz
            # Simple integration (convert to radians)
            roll_estimate += math.radians(gyro_data[0]) * dt
            pitch_estimate += math.radians(gyro_data[1]) * dt
            yaw_estimate += math.radians(gyro_data[2]) * dt
        
        # Process accelerometer data
        if accel_data:
            ax, ay, az = accel_data[0], accel_data[1], accel_data[2]
            
            # Calculate attitude from accelerometer
            if abs(az) > 0.1:  # Avoid division by zero
                accel_roll = math.atan2(ay, az)
                accel_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
                
                # Complementary filter
                roll_estimate = gyro_weight * roll_estimate + accel_weight * accel_roll
                pitch_estimate = gyro_weight * pitch_estimate + accel_weight * accel_pitch
        
        # Process position data
        if pos_data:
            x_estimate = pos_data[0] / 1000.0  # Convert mm to m
            y_estimate = pos_data[1] / 1000.0
            yaw_estimate = math.radians(pos_data[2])  # Use chassis yaw
        
        # Keep angles in reasonable range
        roll_estimate = ((roll_estimate + math.pi) % (2 * math.pi)) - math.pi
        pitch_estimate = ((pitch_estimate + math.pi) % (2 * math.pi)) - math.pi
        yaw_estimate = ((yaw_estimate + math.pi) % (2 * math.pi)) - math.pi
        
        iteration += 1
        
        # Print results every 3 iterations
        if iteration % 3 == 0:
            print(f"Time: {elapsed:.1f}s (Update #{iteration})")
            print(f"Attitude: R={math.degrees(roll_estimate):.1f}° "
                  f"P={math.degrees(pitch_estimate):.1f}° "
                  f"Y={math.degrees(yaw_estimate):.1f}°")
            print(f"Position: X={x_estimate:.3f}m Y={y_estimate:.3f}m")
            
            # Show raw data if available
            if gyro_data:
                print(f"Gyro: {gyro_data[0]:.1f} {gyro_data[1]:.1f} {gyro_data[2]:.1f} deg/s")
            if accel_data:
                print(f"Accel: {accel_data[0]:.2f} {accel_data[1]:.2f} {accel_data[2]:.2f} m/s²")
        
        time.sleep(0.33)  # ~3Hz updates

except Exception as e:
    print(f"Error during execution: {e}")

finally:
    # Cleanup - unsubscribe from sensors
    try:
        sensor_imu.unsub_gyroscope()
        sensor_imu.unsub_accelerometer()
        chassis_ctrl.unsub_position()
    except:
        pass
    
    elapsed_total = time.time() - start_time
    print(f"=== Demo Completed ===")
    print(f"Total runtime: {elapsed_total:.1f} seconds")
    print(f"Total updates: {iteration}")
    print(f"Average rate: {iteration/elapsed_total:.1f} Hz")
    print("Final estimates:")
    print(f"  Roll={math.degrees(roll_estimate):.1f}° "
          f"Pitch={math.degrees(pitch_estimate):.1f}° "
          f"Yaw={math.degrees(yaw_estimate):.1f}°")
    print(f"  Position: X={x_estimate:.3f}m Y={y_estimate:.3f}m")