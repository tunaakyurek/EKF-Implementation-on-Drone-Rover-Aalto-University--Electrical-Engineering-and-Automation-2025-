# RoboMaster S1 EKF Implementation Guide

## 🎯 Quick Decision Tree

```
Do you have root access to S1?
├── NO → Use Lab Environment (this guide)
│   ├── Need simple testing? → Use `basic_sensor_test_FIXED.py`
│   ├── Need basic EKF? → Use `lab_ekf_simple_FIXED.py`
│   ├── Need full EKF? → Use `ekf_complete_s1.py`
│   └── Need advanced features? → Use `advanced_ekf_s1.py`
└── YES → Use main SDK approach (../src/)
    └── Full Raspberry Pi implementation recommended
```

## 📱 Step-by-Step Implementation (No Root Required)

### Step 1: Test Basic Connectivity
1. Open RoboMaster app on your phone/tablet
2. Connect to your S1 via WiFi
3. Navigate to **Lab** → **Python**
4. Copy entire contents of `basic_sensor_test_FIXED.py`
5. Paste into Lab editor
6. Click **Run**
7. Verify you see sensor data output

### Step 2: Run Simple EKF
1. Once basic test works, copy `lab_ekf_simple_FIXED.py`
2. Paste into Lab editor
3. Run for 20 seconds
4. Observe attitude and position estimates

### Step 3: Deploy Complete EKF
1. Copy entire `ekf_complete_s1.py` (single file, ~600 lines)
2. Paste into Lab editor
3. Adjust configuration at top of file:
   ```python
   EKF_FREQUENCY = 5  # Adjust based on performance
   RUNTIME_SECONDS = 60  # How long to run
   USE_LED_FEEDBACK = True  # Visual indicators
   ```
4. Run and monitor performance

### Step 4: Advanced Features (Optional)
1. Use `advanced_ekf_s1.py` for:
   - Adaptive filtering
   - Outlier rejection
   - Motion detection
   - Performance monitoring
   - Extended state estimation

## 🔧 Configuration Options

### Basic Parameters
```python
# In ekf_complete_s1.py, adjust these:

# Filter trust weights (0-1)
ACCEL_TRUST = 0.15      # Lower = trust gyro more
GYRO_TRUST = 0.85       # Higher = trust gyro more
CHASSIS_TRUST = 0.95    # Trust in wheel encoders

# Process noise (uncertainty growth)
Q_POSITION = 0.01       # Position uncertainty
Q_VELOCITY = 0.1        # Velocity uncertainty
Q_ANGLE = 0.05          # Angle uncertainty

# Measurement noise (sensor trust)
R_ACCEL = 0.5           # Accelerometer noise
R_GYRO = 0.1            # Gyroscope noise
R_CHASSIS = 0.05        # Encoder noise
```

### Advanced Features
```python
# In advanced_ekf_s1.py:

ENABLE_VISION = False   # CPU intensive
ENABLE_GIMBAL = True    # Extra orientation data
ADAPTIVE_FREQUENCY = True  # Auto-adjust rate
USE_ADAPTIVE_NOISE = True  # Motion-based filtering
```

## 📊 Expected Performance

### Lab Environment (S1 Internal)
| Metric | Basic | Complete | Advanced |
|--------|-------|----------|----------|
| **Update Rate** | 3 Hz | 5-10 Hz | 3-8 Hz |
| **CPU Usage** | 10% | 25% | 35% |
| **Memory** | 3 MB | 8 MB | 12 MB |
| **Accuracy** | ±10° | ±5° | ±3° |
| **Features** | Minimal | Standard | Full |

### With Raspberry Pi (Main Project)
| Metric | Value |
|--------|-------|
| **Update Rate** | 50 Hz |
| **CPU Usage** | 15% |
| **Memory** | 50 MB |
| **Accuracy** | ±1° |
| **Features** | Complete |

## 🚨 Troubleshooting

### Common Issues & Solutions

#### 1. "Failed Error Line Number: X"
- **Cause**: Using wrong API syntax
- **Solution**: Ensure using Lab APIs, not SDK APIs
- **Example**: Use `robot_ctrl.set_mode()` not `import robot`

#### 2. "Sensor data not updating"
- **Cause**: Subscription frequency too high
- **Solution**: Reduce frequency to 5 Hz or less
```python
sensor_imu.sub_gyroscope(freq=5)  # Not 10 or 50
```

#### 3. "Robot becomes unresponsive"
- **Cause**: CPU overload
- **Solution**: 
  - Reduce EKF_FREQUENCY
  - Disable LED feedback
  - Use simpler filter (complementary)

#### 4. "Position drifting"
- **Cause**: No absolute reference (no GPS/magnetometer)
- **Solution**: 
  - Increase CHASSIS_TRUST
  - Implement zero-velocity updates when static
  - Use vision markers if available

#### 5. "Memory error after running"
- **Cause**: Data buffer overflow
- **Solution**: Reduce buffer sizes
```python
class DataRecorder:
    def __init__(self, max_size=200):  # Reduce from 500
```

## 🎓 Understanding the Limitations

### Why Not Full 50Hz EKF?
1. **Hardware**: ARM Cortex-A7 @ ~1GHz (vs Pi4 @ 1.5GHz quad-core)
2. **Memory**: 272MB total (vs 2-8GB on Pi)
3. **Python**: Interpreted, sandboxed (vs full Python with NumPy)
4. **System Load**: Shared with DJI processes

### What Works Well
- ✅ Basic sensor fusion (IMU + encoders)
- ✅ Orientation estimation
- ✅ Relative positioning
- ✅ Motion detection
- ✅ Simple trajectory recording

### What Doesn't Work
- ❌ Complex matrix operations (no NumPy)
- ❌ High-frequency updates (>10Hz)
- ❌ Large state vectors (>12 dimensions)
- ❌ Real-time networking
- ❌ External sensor integration (without root)

## 🔄 Migration Path

### From S1 Lab to Full System
1. **Start**: Test with Lab environment
2. **Validate**: Confirm sensor data quality
3. **Optimize**: Tune parameters for your use case
4. **Scale**: Move to Raspberry Pi for production
5. **Integrate**: Add network streaming, visualization

### Code Reusability
The mathematical core (EKF equations) can be reused:
```python
# Same algorithm works in both environments
def predict(self, dt):
    # Position update
    self.state[0] += self.velocity[0] * dt
    # ... same logic
```

## 📈 Optimization Tips

### 1. Pre-compute Constants
```python
# Instead of computing every loop
sin_30 = math.sin(math.radians(30))

# Pre-compute once
SIN_TABLE = [math.sin(i*0.01) for i in range(628)]
```

### 2. Use Integer Math
```python
# Instead of float
x = 1.234  # meters

# Use integer
x_mm = 1234  # millimeters
```

### 3. Reduce Function Calls
```python
# Instead of
for i in range(3):
    update_state(i)

# Inline critical code
state[0] += velocity[0] * dt
state[1] += velocity[1] * dt
state[2] += velocity[2] * dt
```

### 4. Adaptive Processing
```python
# Skip expensive operations when static
if robot_is_moving():
    full_ekf_update()
else:
    simple_predict()
```

## 🎯 Best Practices

1. **Start Simple**: Use basic test first
2. **Monitor Performance**: Check CPU and memory
3. **Tune Gradually**: Adjust one parameter at a time
4. **Document Changes**: Keep notes on what works
5. **Test Systematically**: Use consistent test patterns
6. **Backup Working Code**: Save versions that work

## 🔗 File Reference

| File | Purpose | Complexity | Use When |
|------|---------|------------|----------|
| `basic_sensor_test_FIXED.py` | Sensor validation | ⭐ | First time setup |
| `lab_ekf_simple_FIXED.py` | Basic filtering | ⭐⭐ | Learning EKF |
| `ekf_complete_s1.py` | Full EKF system | ⭐⭐⭐ | Production use |
| `advanced_ekf_s1.py` | Extended features | ⭐⭐⭐⭐ | Research/Advanced |

## 📚 Further Reading

- [S1 Capabilities Analysis](S1_CAPABILITIES_ANALYSIS.md) - Deep dive into hardware
- [API Fix Summary](API_FIX_SUMMARY.md) - Understanding API differences
- [Testing Guide](TESTING_GUIDE.md) - Comprehensive testing procedures
- [Main Project](../README.md) - Full Raspberry Pi implementation

## ✅ Success Criteria

Your EKF implementation is successful when:
1. **Sensor data** updates consistently
2. **Orientation** tracks robot movement
3. **Position** remains stable when static
4. **Performance** maintains target frequency
5. **System** remains responsive

## 🚀 Next Steps

1. **Implement**: Start with `ekf_complete_s1.py`
2. **Tune**: Adjust parameters for your robot
3. **Test**: Run systematic tests
4. **Document**: Record performance metrics
5. **Optimize**: Improve based on results
6. **Scale**: Move to Raspberry Pi for production

---

Remember: The S1 Lab environment is limited but capable. With proper optimization and realistic expectations, sophisticated state estimation is achievable!