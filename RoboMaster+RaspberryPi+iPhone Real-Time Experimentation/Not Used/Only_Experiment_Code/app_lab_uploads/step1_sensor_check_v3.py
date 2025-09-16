# Step 1 V3: Alternative Sensor Access Methods
# Tests different API approaches for S1 Lab

print("=" * 40)
print("STEP 1 V3: ALTERNATIVE SENSOR ACCESS")
print("=" * 40)

# Set robot mode
robot_ctrl.set_mode(rm_define.robot_mode_gimbal_follow)

# Method 1: Direct chassis (this worked for you)
print("Method 1: Chassis Direct Access")
try:
    pos = chassis_ctrl.get_position()
    if pos:
        print("  SUCCESS: Chassis = " + str(pos))
    else:
        print("  FAIL: No chassis data")
except:
    print("  ERROR: Chassis not accessible")

print("-" * 40)

# Method 2: Try gimbal as alternative orientation source
print("Method 2: Gimbal for Orientation")
try:
    pitch = gimbal_ctrl.get_pitch()
    yaw = gimbal_ctrl.get_yaw()
    print("  Gimbal Pitch: " + str(pitch))
    print("  Gimbal Yaw: " + str(yaw))
    print("  SUCCESS: Gimbal working")
except:
    print("  FAIL: Gimbal not accessible")

print("-" * 40)

# Method 3: Try robot attitude (might work instead of IMU)
print("Method 3: Robot Attitude")
try:
    # Some S1 versions use this instead of sensor_imu
    attitude = robot_ctrl.get_attitude()
    if attitude:
        print("  SUCCESS: Attitude = " + str(attitude))
    else:
        print("  No attitude data")
except:
    print("  Attitude API not available")

print("-" * 40)

# Method 4: Test movement to infer sensor availability
print("Method 4: Movement-based Sensor Test")
initial_pos = None
final_pos = None

try:
    # Get initial position
    initial_pos = chassis_ctrl.get_position()
    print("  Initial: " + str(initial_pos))
    
    # Small rotation to test sensors
    chassis_ctrl.rotate(rm_define.clockwise, 10)
    robot_ctrl.wait(1)
    
    # Get final position
    final_pos = chassis_ctrl.get_position()
    print("  Final: " + str(final_pos))
    
    if initial_pos and final_pos:
        if initial_pos[2] != final_pos[2]:
            print("  SUCCESS: Yaw sensor working")
        else:
            print("  WARNING: No yaw change detected")
except:
    print("  Movement test failed")

print("-" * 40)

# Method 5: Check what APIs are actually available
print("Method 5: Available APIs Check")

# Test each API module
apis_available = []
apis_missing = []

# Test chassis_ctrl
try:
    chassis_ctrl.get_position()
    apis_available.append("chassis_ctrl")
except:
    apis_missing.append("chassis_ctrl")

# Test gimbal_ctrl
try:
    gimbal_ctrl.get_pitch()
    apis_available.append("gimbal_ctrl")
except:
    apis_missing.append("gimbal_ctrl")

# Test led_ctrl
try:
    led_ctrl.turn_off(rm_define.armor_all)
    apis_available.append("led_ctrl")
except:
    apis_missing.append("led_ctrl")

# Test robot_ctrl
try:
    robot_ctrl.get_mode()
    apis_available.append("robot_ctrl")
except:
    apis_missing.append("robot_ctrl")

# Test sensor_imu (probably won't work based on your test)
try:
    sensor_imu.get_gyroscope()
    apis_available.append("sensor_imu")
except:
    apis_missing.append("sensor_imu")

print("Available APIs:")
for api in apis_available:
    print("  + " + api)

print("Missing APIs:")
for api in apis_missing:
    print("  - " + api)

print("=" * 40)
print("ANALYSIS:")
if "sensor_imu" not in apis_available:
    print("IMU not directly accessible!")
    print("ALTERNATIVES:")
    print("1. Use chassis yaw for orientation")
    print("2. Use gimbal angles for tilt")
    print("3. Estimate from chassis movement")
else:
    print("IMU is available")

if len(apis_available) >= 3:
    print("Enough sensors for basic EKF")
else:
    print("Limited sensors - use simplified approach")
print("=" * 40)
