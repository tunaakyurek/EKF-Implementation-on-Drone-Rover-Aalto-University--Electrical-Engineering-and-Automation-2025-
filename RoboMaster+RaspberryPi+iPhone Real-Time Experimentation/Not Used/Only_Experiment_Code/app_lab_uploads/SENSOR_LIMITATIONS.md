# S1 Lab Sensor Limitations - Critical Findings

## 🔴 IMPORTANT DISCOVERY

Your testing revealed that **IMU sensors (accelerometer & gyroscope) are NOT accessible** in the S1 Lab environment, but chassis sensors ARE working. This is a significant limitation that changes our approach.

## What This Means

### ❌ Not Available in S1 Lab:
- `sensor_imu.get_gyroscope()` - **FAILS**
- `sensor_imu.get_accelerometer()` - **FAILS**
- Direct IMU access of any kind

### ✅ Available in S1 Lab:
- `chassis_ctrl.get_position()` - **WORKS** (x, y, yaw)
- `gimbal_ctrl.get_pitch()` - May work
- `gimbal_ctrl.get_yaw()` - May work
- `led_ctrl.*` - Works
- `robot_ctrl.*` - Works

## Why This Happens

The S1 Lab environment (unlike EP model or rooted S1):
1. **Sandboxes sensor access** for safety
2. **Limits direct hardware access** to prevent damage
3. **Only exposes high-level APIs** like chassis position
4. **Blocks low-level sensor data** like raw IMU

## Alternative Solutions

### Solution 1: Chassis-Only EKF
**File:** `ekf_chassis_only.py`
- Uses ONLY chassis position data
- Derives velocity from position changes
- Estimates orientation from chassis yaw
- **Pros:** Works with available sensors
- **Cons:** Less accurate, no tilt detection

### Solution 2: Gimbal-Assisted Estimation
**File:** `step1_sensor_check_v3.py` (tests gimbal)
- Use gimbal angles for orientation
- Combine with chassis for position
- **Pros:** Better orientation data
- **Cons:** Gimbal may also be limited

### Solution 3: Motion-Based Inference
- Command specific movements
- Measure position changes
- Infer sensor characteristics
- **Pros:** Works around limitations
- **Cons:** Not real-time

## Recommended Approach

Given the sensor limitations, use this progression:

### Step 1: Verify Available Sensors
Run `step1_sensor_check_v3.py` to determine exactly what's accessible

### Step 2: Use Chassis-Only EKF
Run `ekf_chassis_only.py` which doesn't require IMU

### Step 3: Enhance if Possible
If gimbal works, add gimbal data for better orientation

## Modified EKF Strategy

### Original Plan (With IMU):
```
State = [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
Sensors = IMU + Chassis + GPS
Update Rate = 10-50 Hz
```

### Revised Plan (Without IMU):
```
State = [x, y, yaw, vx, vy, vyaw]
Sensors = Chassis only
Update Rate = 5 Hz
```

## What You Lose Without IMU

1. **No tilt detection** (roll, pitch)
2. **No vibration sensing**
3. **No acceleration measurement**
4. **Reduced update rate**
5. **Less accurate motion detection**

## What Still Works

1. **2D position tracking** ✅
2. **Yaw (heading) estimation** ✅
3. **Velocity estimation** ✅
4. **Basic state estimation** ✅
5. **Trajectory recording** ✅

## Code Modifications Needed

### Remove All IMU References:
```python
# DON'T USE:
gyro = sensor_imu.get_gyroscope()
accel = sensor_imu.get_accelerometer()

# USE INSTEAD:
pos = chassis_ctrl.get_position()
# Derive everything from position changes
```

### Simplify State Vector:
```python
# Original: 12 states
state = [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]

# Simplified: 6 states
state = [x, y, yaw, vx, vy, vyaw]
```

### Adjust Filter Parameters:
```python
# Increase chassis trust since it's our only sensor
CHASSIS_TRUST = 0.99  # Was 0.95

# Reduce process noise since we have less uncertainty
Q_POSITION = 0.005  # Was 0.01
```

## Performance Expectations

| Metric | With IMU | Without IMU (Chassis Only) |
|--------|----------|---------------------------|
| Update Rate | 10-50 Hz | 5 Hz |
| States | 12 | 6 |
| Accuracy | ±1° orientation | ±5° yaw only |
| Position | ±1cm | ±5cm |
| Capabilities | Full 3D | 2D only |

## Bottom Line

**The S1 Lab environment has significant sensor limitations compared to the documentation.** The standard EKF implementations won't work because they assume IMU access. 

**Use `ekf_chassis_only.py` instead** - it's specifically designed to work with only the sensors that ARE available in your S1 Lab environment.

## Future Options

1. **Root the S1** - Would give full sensor access
2. **Use External Processing** - Raspberry Pi with full SDK
3. **Accept Limitations** - Work within chassis-only constraints
4. **Hybrid Approach** - Use S1 for motors, Pi for sensors

---

**Key Takeaway:** Your S1 can still do EKF state estimation, just with reduced capabilities due to sensor limitations. The chassis-only approach is your best option for now.
