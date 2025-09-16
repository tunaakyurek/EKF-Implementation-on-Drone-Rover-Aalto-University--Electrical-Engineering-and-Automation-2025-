# EKF Implementation Using ONLY Chassis Data - FIXED
# No IMU, No Gimbal, No time module, No wait function

print("=" * 40)
print("CHASSIS-ONLY EKF FOR S1 - FIXED")
print("Works with limited sensors!")
print("=" * 40)
print("")
print("INSTRUCTIONS:")
print("1. EKF will track your movements")
print("2. Use app controls to move robot")
print("3. Watch position/velocity update")
print("=" * 40)

# Set robot mode to FREE to allow manual control during EKF
robot_ctrl.set_mode(rm_define.robot_mode_free)

# ============================================================================
# CONFIGURATION
# ============================================================================

TOTAL_ITERATIONS = 200  # Total loop iterations (about 40 seconds)
PRINT_EVERY = 10  # Print status every N iterations

# ============================================================================
# CHASSIS-ONLY EKF CLASS
# ============================================================================

class ChassisEKF:
    def __init__(self):
        # State: position and velocity
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0
        
        # Previous values
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_yaw = 0.0
        
        # Simple uncertainty
        self.uncertainty = 1.0
        self.updates = 0
    
    def update(self, pos_data):
        """Update from chassis position"""
        if not pos_data:
            return False
        
        # Get position
        new_x = pos_data[0] / 1000.0  # mm to m
        new_y = pos_data[1] / 1000.0
        new_yaw = pos_data[2] * 0.017453 if len(pos_data) > 2 else self.yaw
        
        # Calculate velocity (simple difference)
        if self.updates > 0:
            # Assume fixed time step based on 5Hz subscription
            dt = 0.2  # Approximate time between updates
            
            # Calculate instantaneous velocity
            dx = new_x - self.prev_x
            dy = new_y - self.prev_y
            
            # Apply smoothing to reduce noise
            alpha = 0.3  # Smoothing factor
            self.vx = alpha * (dx / dt) + (1 - alpha) * self.vx
            self.vy = alpha * (dy / dt) + (1 - alpha) * self.vy
            
            # Handle yaw wrapping
            dyaw = new_yaw - self.prev_yaw
            if dyaw > 3.14159:
                dyaw -= 6.28318
            elif dyaw < -3.14159:
                dyaw += 6.28318
            self.vyaw = dyaw / dt
        
        # Update state
        self.x = new_x
        self.y = new_y
        self.yaw = new_yaw
        
        # Normalize yaw to [-pi, pi]
        while self.yaw > 3.14159:
            self.yaw -= 6.28318
        while self.yaw < -3.14159:
            self.yaw += 6.28318
        
        # Store previous
        self.prev_x = new_x
        self.prev_y = new_y
        self.prev_yaw = new_yaw
        
        # Update uncertainty
        speed = (self.vx * self.vx + self.vy * self.vy) ** 0.5
        if speed < 0.01:
            self.uncertainty *= 0.95  # Reduce when stationary
        else:
            self.uncertainty *= 1.02  # Increase when moving
        
        # Clamp uncertainty
        if self.uncertainty < 0.1:
            self.uncertainty = 0.1
        elif self.uncertainty > 5.0:
            self.uncertainty = 5.0
        
        self.updates += 1
        return True
    
    def get_state_string(self):
        """Get state as string (no f-strings)"""
        # Format numbers cleanly without trailing zeros
        x_str = str(round(self.x, 3))
        y_str = str(round(self.y, 3))
        yaw_deg = str(round(self.yaw * 57.2958, 1))
        vx_str = str(round(self.vx, 3))
        vy_str = str(round(self.vy, 3))
        # Fix uncertainty display - limit to 2 decimals
        unc_val = round(self.uncertainty, 2)
        if unc_val == 0.1:
            unc_str = "0.10"
        else:
            unc_str = str(unc_val)
        
        return "X=" + x_str + " Y=" + y_str + " Yaw=" + yaw_deg + " Vx=" + vx_str + " Vy=" + vy_str + " U=" + unc_str

# ============================================================================
# MAIN EXECUTION
# ============================================================================

print("Initializing EKF...")

# Create EKF
ekf = ChassisEKF()

# Subscribe to chassis
try:
    chassis_ctrl.sub_position(freq=5)
    print("Chassis subscribed")
except:
    print("Using direct chassis access")

# LED startup sequence
led_ctrl.set_flash(rm_define.armor_all, 5)
led_ctrl.set_flash(rm_define.armor_all, 2)
led_ctrl.turn_off(rm_define.armor_all)

print("Running EKF...")
print("Robot in FREE MODE - you can move it!")
print("-" * 40)

# Main loop - use fixed iterations instead of time
successful_updates = 0
failed_updates = 0

for iteration in range(TOTAL_ITERATIONS):
    # Get chassis position
    pos = None
    try:
        pos = chassis_ctrl.get_position()
    except:
        failed_updates += 1
        continue
    
    # Update EKF
    if pos:
        if ekf.update(pos):
            successful_updates += 1
            
            # LED feedback based on motion (only update every few iterations to avoid flicker)
            if iteration % 5 == 0:
                speed = (ekf.vx * ekf.vx + ekf.vy * ekf.vy) ** 0.5
                
                if speed > 0.1:
                    # Fast motion - green flash
                    led_ctrl.set_flash(rm_define.armor_all, 5)
                elif speed > 0.02:
                    # Slow motion - blue flash
                    led_ctrl.set_flash(rm_define.armor_all, 2)
                else:
                    # Stationary - solid blue
                    led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 0, 0, 255, rm_define.effect_always_on)
        else:
            failed_updates += 1
    else:
        failed_updates += 1
    
    # Print status periodically
    if (iteration + 1) % PRINT_EVERY == 0:
        speed = (ekf.vx * ekf.vx + ekf.vy * ekf.vy) ** 0.5
        print("Iter " + str(iteration + 1) + "/" + str(TOTAL_ITERATIONS))
        print("  " + ekf.get_state_string())
        if speed > 0.05:
            print("  STATUS: MOVING (speed=" + str(round(speed, 3)) + " m/s)")
        else:
            print("  STATUS: STATIONARY")
        print("  Updates: OK=" + str(successful_updates) + " Fail=" + str(failed_updates))
    
    # Create delay using LED operations (workaround for no wait/sleep)
    # Reduced delay for more responsive updates (about 100ms)
    for delay_loop in range(2):
        led_ctrl.set_top_led(rm_define.armor_top_all, 100, 100, 100, rm_define.effect_always_off)

# Cleanup
print("-" * 40)
print("Cleaning up...")

try:
    chassis_ctrl.unsub_position()
except:
    pass

# Final results
print("=" * 40)
print("EKF COMPLETE")
print("Total iterations: " + str(TOTAL_ITERATIONS))
print("Successful updates: " + str(successful_updates))
print("Failed updates: " + str(failed_updates))
print("Success rate: " + str(round(100.0 * successful_updates / TOTAL_ITERATIONS, 1)) + "%")
print("")
print("Final State:")
print("  " + ekf.get_state_string())
print("")
print("Movement Summary:")
print("  Distance from origin: " + str(round((ekf.x*ekf.x + ekf.y*ekf.y)**0.5, 3)) + " m")
print("  Final position: (" + str(round(ekf.x, 3)) + ", " + str(round(ekf.y, 3)) + ") m")
print("  Final heading: " + str(round(ekf.yaw * 57.2958, 1)) + " degrees")
if abs(ekf.x) < 0.01 and abs(ekf.y) < 0.01:
    print("  Robot stayed near starting position")
else:
    print("  Robot moved during test!")
print("=" * 40)

# Victory celebration
for i in range(5):
    led_ctrl.set_flash(rm_define.armor_all, 10)
    led_ctrl.turn_off(rm_define.armor_all)
