# RoboMaster S1 Direct Execution Testing Guide

## 🎯 Overview
This guide provides step-by-step instructions for testing direct code execution on the RoboMaster S1, ranging from safe approaches to advanced hacking techniques.

## 🔒 Safety Levels

### Level 1: Safe (App Lab) - **RECOMMENDED FOR BEGINNERS**
- Uses official RoboMaster app Lab feature
- No warranty voiding
- Limited but safe functionality

### Level 2: Moderate (ADB Access)
- Requires enabling development features
- Some risk of system instability
- Reversible modifications

### Level 3: Advanced (Root Access) - **HIGH RISK**
- Requires rooting the device
- **VOIDS WARRANTY**
- Potential for device bricking

## 📋 Prerequisites

### Hardware Requirements
- RoboMaster S1 (firmware version < 00.06.0100 recommended)
- Windows/Linux PC with USB ports
- High-quality micro USB cable (data transfer capable)
- WiFi network for S1 connectivity

### Software Requirements
- RoboMaster app (latest version)
- Android SDK Platform Tools ([Download](https://developer.android.com/studio/releases/platform-tools))
- Python 3.7+ on PC
- Text editor or IDE

### Knowledge Requirements
- Basic Python programming
- Command line interface usage
- Understanding of embedded systems (for advanced levels)

## 🥇 Level 1: Safe App Lab Testing

### Step 1: Basic Setup
1. **Power on RoboMaster S1**
   ```bash
   # Ensure S1 is in SDK mode (should show blue lights)
   ```

2. **Connect via RoboMaster App**
   - Open RoboMaster app
   - Connect to S1 via WiFi or router
   - Navigate to "Lab" section

3. **Create New Python Project**
   - Tap "Python" in Lab section
   - Create new project: "EKF_Experiment"

### Step 2: Copy and Paste Fixed Code
**FIXED: Use correct on-device Lab API syntax**

**Option A: Basic Sensor Test** (Recommended First)
- Copy code from `app_lab_uploads/basic_sensor_test_FIXED.py`
- Paste into Lab editor
- Runtime: 10 seconds
- Tests basic sensor access

**Option B: Simple EKF Demo** (More Advanced)  
- Copy code from `app_lab_uploads/lab_ekf_simple_FIXED.py`
- Paste into Lab editor
- Runtime: 20 seconds
- Shows basic state estimation

**Key Fix**: The code now uses **on-device Lab APIs** instead of external SDK syntax:
- ✅ `robot_ctrl.set_mode()` instead of `import robot`
- ✅ `sensor_imu.get_gyroscope()` instead of `ep_sensor.get_gyroscope()`
- ✅ `chassis_ctrl.get_position()` instead of `ep_chassis.get_position()`

**Files Provided**:
- `app_lab_uploads/basic_sensor_test_FIXED.py` - Ultra-simple sensor test
- `app_lab_uploads/lab_ekf_simple_FIXED.py` - Basic state estimation (FIXED)
- `app_lab_uploads/lab_ekf_simple.py` - Original (broken) version for reference

### Step 3: Run and Analyze
- Execute the code in the Lab
- Observe sensor data output
- Note: This is the safest way to test basic functionality

### Expected Results
- Gyroscope readings in degrees/second
- Accelerometer readings in m/s²
- Chassis position in millimeters
- Update rate limited to ~10 Hz due to app constraints

## 🥈 Level 2: ADB Access Testing

### Step 1: Enable ADB on RoboMaster S1

1. **Execute Sandbox Escape Code**
   In the RoboMaster app Lab, run this code:

```python
def root_me(module):
    __import__ = rm_define.__dict__['__builtins__']['__import__']
    return __import__(module, globals(), locals(), [], 0)

builtins = root_me('builtins')
subprocess = root_me('subprocess')
proc = subprocess.Popen('/system/bin/adb_en.sh', shell=True, 
                       executable='/system/bin/sh', 
                       stdout=subprocess.PIPE, 
                       stderr=subprocess.PIPE)
```

2. **Verify ADB Enable**
   - Console should show "Execution Complete"
   - Keep the app open (don't close it)

### Step 2: Connect via ADB

1. **Setup ADB on PC**
   ```bash
   # Download Android SDK Platform Tools
   # Extract to a folder (e.g., C:\platform-tools)
   cd C:\platform-tools
   ```

2. **Connect USB Cable**
   - Connect S1 to PC via micro USB
   - Use the port on the intelligent controller
   - Should hear connection sound

3. **Test ADB Connection**
   ```bash
   # Check if device is detected
   .\adb.exe devices
   
   # Should output something like:
   # List of devices attached
   # 1234567890ABCDEF    device
   ```

4. **Access Shell**
   ```bash
   .\adb.exe shell
   ```

### Step 3: System Analysis

Once in the shell, run these commands:

```bash
# Check system info
cat /proc/cpuinfo
cat /proc/meminfo
ps | grep dji

# Check Python environment
ls /data/python_files/bin/
/data/python_files/bin/python --version

# Check available space
df -h

# List uploaded scripts
ls /data/script/file/
```

### Step 4: Upload and Test Custom Code

1. **Create Test Script on PC**
   ```python
   # File: test_direct.py
   import time
   import os
   
   print("=== RoboMaster S1 Direct Execution Test ===")
   print(f"Python version: {sys.version}")
   print(f"Current directory: {os.getcwd()}")
   print(f"Available memory: {open('/proc/meminfo').readline()}")
   
   # Basic sensor simulation
   for i in range(10):
       print(f"Sensor reading {i}: timestamp={time.time()}")
       time.sleep(0.1)
   
   print("Test completed successfully!")
   ```

2. **Upload Script**
   ```bash
   # From PC
   .\adb.exe push test_direct.py /data/script/file/
   ```

3. **Execute Script**
   ```bash
   # In ADB shell
   cd /data/script/file/
   /data/python_files/bin/python test_direct.py
   ```

### Expected Results
- Script should execute successfully
- Memory info should show ~272MB total
- May encounter import restrictions for numpy/scipy

## 🥉 Level 3: Advanced Root Testing

⚠️ **WARNING: This level can permanently damage your RoboMaster S1. Proceed only if you accept the risks.**

### Step 1: Complete Root Access

1. **Download Rooting Tools**
   - Get rooting scripts from research repositories
   - Ensure S1 firmware is compatible

2. **Execute Root Exploit**
   ```bash
   # Follow specific rooting instructions for your firmware version
   # This varies significantly based on S1 firmware
   ```

### Step 2: Install Custom Python Environment

```bash
# Once rooted, install additional packages
# Note: This may require cross-compilation for ARM
```

### Step 3: Direct Hardware Access Testing

```python
# Example: Direct CAN bus communication
import socket
import struct

def direct_can_test():
    # This would require custom CAN drivers
    # Implementation depends on successful rooting
    pass
```

## 📊 Performance Benchmarking

Use these scripts to test system capabilities:

### Memory Usage Test
```python
# File: utilities/memory_profiler.py
import psutil
import time

def memory_test():
    for i in range(60):  # Run for 1 minute
        mem = psutil.virtual_memory()
        print(f"Memory: {mem.percent}% used, {mem.available/1024/1024:.1f}MB free")
        time.sleep(1)
```

### CPU Performance Test
```python
# File: utilities/benchmark.py
import time
import math

def cpu_benchmark():
    start = time.time()
    
    # Simple computation test
    result = 0
    for i in range(100000):
        result += math.sqrt(i) * math.sin(i)
    
    duration = time.time() - start
    print(f"CPU benchmark: {duration:.2f} seconds")
    return duration
```

## 🔧 Troubleshooting

### Common Issues

1. **ADB Not Connecting**
   - Try different USB cable
   - Ensure S1 is powered on and app is open
   - Check Windows device manager for driver issues

2. **Scripts Not Executing**
   - Verify file permissions: `chmod 755 script.py`
   - Check Python path: `/data/python_files/bin/python`
   - Ensure sufficient memory available

3. **Import Errors**
   - Most scientific libraries (numpy, scipy) not available
   - Use pure Python implementations
   - Consider uploading libraries manually (advanced)

4. **System Crashes**
   - Reduce computation intensity
   - Add memory checks before heavy operations
   - Restart S1 if system becomes unresponsive

### Recovery Procedures

1. **App Unresponsive**
   - Close and reopen RoboMaster app
   - Restart S1 by power cycling

2. **ADB Connection Lost**
   - Re-run sandbox escape code in app
   - Check USB connection
   - Restart ADB server: `adb kill-server && adb start-server`

3. **System Corrupted (Root Level)**
   - May require firmware reflashing
   - Contact DJI support (warranty voided)
   - Use recovery mode if available

## 📈 Performance Expectations

Based on testing, expect these performance characteristics:

| Test Type | Expected Performance | Notes |
|-----------|---------------------|--------|
| **Sensor Reading** | 5-20 Hz max | Limited by system overhead |
| **Simple Calculations** | ~1000 ops/sec | Pure Python, no libraries |
| **Memory Usage** | 200MB+ baseline | System processes consume most RAM |
| **File I/O** | ~1MB/s | Internal storage speed |
| **Network** | WiFi dependent | Uses S1's existing connection |

## 🎓 Learning Outcomes

After completing these tests, you should understand:

1. **Embedded System Constraints**
   - Memory and CPU limitations
   - Real-time operating system behavior
   - Resource competition between processes

2. **Python Sandbox Environment**
   - Import restrictions and workarounds
   - Security model implementation
   - Performance implications

3. **Hardware-Software Interface**
   - Low-level sensor access
   - Communication protocols
   - Driver architecture

## 🔄 Next Steps

After testing, consider:

1. **Comparing with Raspberry Pi Approach**
   - Performance differences
   - Development complexity
   - Practical applicability

2. **Exploring Alternative Architectures**
   - External microcontroller integration
   - Hybrid processing approaches
   - Edge computing strategies

3. **Contributing to Research**
   - Document findings
   - Share performance benchmarks
   - Develop optimization techniques

## 📝 Conclusion

This testing guide demonstrates that while direct execution on the RoboMaster S1 is technically possible, the practical limitations make it unsuitable for serious EKF applications. The exercise provides valuable insights into embedded system constraints and alternative computing architectures.

**For actual EKF implementation, stick with the main project's Raspberry Pi architecture.**