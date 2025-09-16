# RoboMaster S1 Direct Execution Experiment

## ⚠️ EXPERIMENTAL SETUP DISCLAIMER
This experimental setup explores running custom Python code directly on the RoboMaster S1's internal computing unit, **bypassing the Raspberry Pi entirely**. This approach has significant limitations and is primarily for research and educational purposes.

## 🚀 Quick Start (Updated for App Import)

### For Beginners (Level 1 - Copy/Paste Fixed Code):
1. **Open** RoboMaster app and connect to S1
2. **Create** new Python project in Lab
3. **Copy** code from `app_lab_uploads/basic_sensor_test_FIXED.py`
4. **Paste** into Lab editor and run
5. **Observe** sensor data output

### Code Options (Final Versions):
- **basic_sensor_test_FIXED.py**: Ultra-simple 10-second sensor test (start here)
- **lab_ekf_simple_FIXED.py**: 20-second basic state estimation demo
- **ekf_chassis_only_FIXED.py**: Working EKF using only chassis data (MAIN SOLUTION)
- **ekf_complete_s1_FIXED.py**: Complete EKF with syntax fixes (limited by IMU access)

### Key Fix Applied:
- ❌ **Old**: `import robot` (external SDK syntax)
- ✅ **New**: `robot_ctrl.set_mode()` (on-device Lab API)

📖 **Full Guide**: See `TESTING_GUIDE.md` for complete instructions

## 🔍 Research Findings

### RoboMaster S1 Hardware Specifications
Based on our research, the RoboMaster S1 has the following internal specifications:

```
CPU: ARMv7 Processor rev 5 (v7l) - 5 cores
Memory: 271,708 KB (~272 MB total RAM)
OS: Android 4.4.4 (KTU84Q)
Python: Internal interpreter at /data/python_files/bin/python
Storage: Limited internal storage at /data/script/file/
```

### Key Limitations
1. **Memory Constraints**: Only ~272 MB total RAM with significant system overhead
2. **CPU Performance**: Limited ARM Cortex-A7 performance (~26 BogoMIPS per core)
3. **Python Sandbox**: Restricted execution environment with limited library access
4. **No Direct Hardware Access**: Requires root access for full functionality
5. **System Load**: Multiple DJI processes consuming resources continuously

## 📁 Experimental Setup Structure

```
Only_Experiment_Code/
├── README.md                    # This documentation
├── TESTING_GUIDE.md            # Comprehensive testing guide
├── direct_ekf/                 # Simplified EKF for S1 execution
│   ├── minimal_ekf.py          # Lightweight EKF implementation
│   ├── sensor_minimal.py       # Minimal sensor interface
│   └── direct_main.py          # Main execution script
├── sandbox_escape/             # Tools for gaining system access
│   ├── root_exploit.py         # Sandbox escape utilities
│   ├── adb_setup.sh           # ADB setup instructions
│   └── upload_helper.py       # File upload utilities
├── can_interface/              # Direct CAN bus communication
│   ├── can_direct.py          # Direct CAN communication
│   └── hardware_interface.py  # Low-level hardware access
├── app_lab_uploads/           # FINAL: Working Lab code only
│   ├── basic_sensor_test_FIXED.py # Ultra-simple sensor test (START HERE)
│   ├── lab_ekf_simple_FIXED.py    # Basic EKF demo (FIXED APIs)
│   ├── ekf_chassis_only_FIXED.py  # Working chassis-only EKF (MAIN SOLUTION)
│   ├── ekf_complete_s1_FIXED.py   # Complete EKF (syntax fixed, IMU limited)
│   ├── step1_sensor_check_v3.py   # Final sensor availability test
│   ├── step2_imu_fusion.py        # IMU fusion test (may fail - no IMU access)
│   ├── step3_chassis_tracking.py  # Chassis position tracking test
│   ├── step4_simple_fusion.py     # Simple sensor fusion test
│   ├── step5_minimal_ekf.py       # Minimal EKF implementation
│   └── SENSOR_LIMITATIONS.md      # Critical S1 sensor access limitations
├── S1_CAPABILITIES_ANALYSIS.md # Comprehensive S1 capabilities analysis (NEW)
└── utilities/                 # Helper tools and scripts
    ├── memory_profiler.py     # Memory usage monitoring
    ├── system_analyzer.py     # System resource analysis
    └── benchmark.py           # Performance benchmarking
```

## 🎯 Approach Comparison

| Approach | Feasibility | Performance | Complexity | Hardware Access |
|----------|-------------|-------------|------------|-----------------|
| **Standard SDK** | ✅ High | ✅ Good | 🟡 Medium | 🟡 Limited |
| **Direct Execution** | 🟡 Limited | ❌ Poor | ❌ High | ✅ Full |
| **Sandbox Escape** | 🟡 Possible | 🟡 Fair | ❌ Very High | ✅ Full |
| **CAN Bus Direct** | ✅ Good | ✅ Good | ❌ High | ✅ Hardware Only |

## 🚫 Why This Approach Has Limitations

### 1. **Insufficient Computational Resources**
- The S1's ARM processor cannot handle a 50Hz EKF with visualization
- System processes consume ~80% of available memory
- No dedicated FPU for efficient matrix operations

### 2. **Restricted Execution Environment** 
- Python sandbox limits library imports
- No access to NumPy/SciPy without rooting
- File system restrictions prevent data persistence

### 3. **System Interference**
- DJI vision/camera processes consume significant CPU
- Real-time constraints conflict with existing system tasks
- Thermal throttling under sustained computational load

### 4. **Networking Limitations**
- No direct WiFi control for external communication
- Limited to internal socket communication
- Cannot establish ground station connection

## ✅ What IS Possible

### 1. **Simple State Estimation**
- Basic complementary filter (instead of full EKF)
- Sensor data logging at reduced frequencies (5-10 Hz)
- Simple trajectory recording

### 2. **Sensor Data Collection**
- Direct IMU/chassis encoder access
- Raw sensor data streaming
- Basic preprocessing and filtering

### 3. **Educational Experiments**
- Understanding embedded Python constraints
- Learning Android/Linux cross-compilation
- Exploring hardware-software interfaces

## 🛠️ Installation and Setup

See `TESTING_GUIDE.md` for detailed setup instructions including:
- Rooting the RoboMaster S1
- Setting up ADB access
- Uploading and executing custom code
- Monitoring system resources

## ⚡ Quick Start for Lab Upload

For the safest approach (using the official RoboMaster app):

1. Open the RoboMaster app
2. Navigate to Lab → Python
3. Upload `app_lab_uploads/lab_ekf_simple.py`
4. Run and observe basic sensor readings

## 🎓 Educational Value

This experimental setup is valuable for:
- Understanding embedded system constraints
- Learning about Python sandbox environments
- Exploring alternative hardware communication methods
- Researching edge computing limitations in robotics

## ⚠️ Important Notes

1. **Warranty Void**: Rooting the S1 will void warranty
2. **Brick Risk**: Improper modifications can render the S1 inoperable
3. **Limited Practicality**: This approach is not recommended for serious EKF applications
4. **Use Raspberry Pi Instead**: For actual EKF implementation, use the main project architecture

## 📚 References

- [RoboMaster S1 Hacking Resources](https://github.com/collabnix/robomaster)
- [Python Sandbox Escape Techniques](https://ctf-wiki.org/pwn/sandbox/python/python-sandbox-escape/)
- [Android ADB Debugging](https://developer.android.com/studio/releases/platform-tools)
- [ARM Cortex-A7 Technical Reference](https://developer.arm.com/documentation/ddi0464/latest/)

---

**Conclusion**: While technically fascinating, direct execution on the RoboMaster S1 is severely limited by hardware constraints. The main project architecture using a Raspberry Pi remains the practical approach for EKF implementation.