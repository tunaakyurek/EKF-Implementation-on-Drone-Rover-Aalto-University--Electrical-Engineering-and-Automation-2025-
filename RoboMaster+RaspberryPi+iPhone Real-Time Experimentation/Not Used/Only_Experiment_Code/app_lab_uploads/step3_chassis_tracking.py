# Step 3: Chassis Position Tracking
# Tests wheel encoder position tracking

import time
import math

print("=" * 40)
print("STEP 3: CHASSIS TRACKING")
print("=" * 40)

robot_ctrl.set_mode(rm_define.robot_mode_gimbal_follow)

# Position variables
x_m = 0.0
y_m = 0.0
yaw_deg = 0.0

# Subscribe to chassis
chassis_ctrl.sub_position(freq=5)

print("Testing chassis tracking for 20 seconds...")
print("Move the robot to see position changes")

start_time = time.time()
iteration = 0
max_x = 0.0
max_y = 0.0

while time.time() - start_time < 20:
    # Get chassis position
    pos = chassis_ctrl.get_position()
    
    if pos and len(pos) >= 2:
        # Update position (convert mm to m)
        x_m = pos[0] / 1000.0
        y_m = pos[1] / 1000.0
        if len(pos) >= 3:
            yaw_deg = pos[2]
        
        # Track maximum displacement
        if abs(x_m) > max_x:
            max_x = abs(x_m)
        if abs(y_m) > max_y:
            max_y = abs(y_m)
        
        iteration += 1
        
        # Print every 10 iterations
        if iteration % 10 == 0:
            elapsed = time.time() - start_time
            distance = math.sqrt(x_m*x_m + y_m*y_m)
            
            print("T=" + str(round(elapsed, 1)) + "s:")
            print("  X=" + str(round(x_m, 3)) + "m")
            print("  Y=" + str(round(y_m, 3)) + "m")
            print("  Yaw=" + str(round(yaw_deg, 1)) + " deg")
            print("  Distance=" + str(round(distance, 3)) + "m")
            
            # LED feedback based on distance
            if distance > 0.1:
                led_ctrl.set_flash(rm_define.armor_all, 3)
            else:
                led_ctrl.turn_off(rm_define.armor_all)
    
    time.sleep(0.2)  # 5Hz

# Cleanup
chassis_ctrl.unsub_position()

print("=" * 40)
print("CHASSIS TRACKING COMPLETE")
print("Total iterations: " + str(iteration))
print("Final position:")
print("  X=" + str(round(x_m, 3)) + "m")
print("  Y=" + str(round(y_m, 3)) + "m")
print("  Yaw=" + str(round(yaw_deg, 1)) + " deg")
print("Max displacement:")
print("  Max X=" + str(round(max_x, 3)) + "m")
print("  Max Y=" + str(round(max_y, 3)) + "m")
print("=" * 40)
