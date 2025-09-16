# RoboMaster S1 Basic Sensor Test - Lab Environment (FIXED)
# Ultra-simple sensor test using correct on-device APIs

import time

print("=== Basic Sensor Test ===")
print("Testing RoboMaster S1 Lab environment")

# Set robot mode
robot_ctrl.set_mode(rm_define.robot_mode_gimbal_follow)

print("Starting 10-second sensor test...")

try:
    for i in range(10):  # 10 iterations, 1 per second
        print(f"Test {i+1}/10:")
        
        # Try to get basic sensor data
        try:
            # Get IMU data using on-device APIs
            gyro_data = sensor_imu.get_gyroscope()
            accel_data = sensor_imu.get_accelerometer()
            
            if gyro_data:
                print(f"  Gyro: X={gyro_data[0]:.1f} Y={gyro_data[1]:.1f} Z={gyro_data[2]:.1f} deg/s")
            else:
                print("  Gyro: No data")
            
            if accel_data:
                print(f"  Accel: X={accel_data[0]:.2f} Y={accel_data[1]:.2f} Z={accel_data[2]:.2f} m/s²")
            else:
                print("  Accel: No data")
            
            # Simple computation test
            if gyro_data and accel_data:
                total_gyro = abs(gyro_data[0]) + abs(gyro_data[1]) + abs(gyro_data[2])
                total_accel = abs(accel_data[0]) + abs(accel_data[1]) + abs(accel_data[2])
                print(f"  Totals: Gyro={total_gyro:.1f} Accel={total_accel:.2f}")
                
        except Exception as e:
            print(f"  Error getting sensor data: {e}")
        
        # Simple LED indicator
        try:
            led_ctrl.set_flash(rm_define.armor_all, 2)  # Flash all LEDs at 2Hz
            time.sleep(0.5)
            led_ctrl.turn_off(rm_define.armor_all)
            time.sleep(0.5)
        except:
            time.sleep(1)

except Exception as e:
    print(f"Test error: {e}")

print("=== Test Complete ===")
print("If you saw sensor data, the Lab environment is working!")
print("Try the EKF demo next.")