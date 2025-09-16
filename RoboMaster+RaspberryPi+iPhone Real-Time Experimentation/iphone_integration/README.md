# iPhone-EKF Integration System for RoboMaster S1

## Overview

This subsystem integrates iPhone sensor data with an 8-DOF Extended Kalman Filter (EKF) for RoboMaster S1. It uses `pi_phone_connection/ekf_robomaster_8dof.py` and the main runner `pi_phone_connection/main_integration_robomaster.py`.

## System Architecture

```
iPhone (Sensors) → Raspberry Pi (EKF Processing + Logging)
                           ↓
                    Offline Analysis (MATLAB/Python)
```

## Features

### 8-DOF EKF Implementation
- **State Vector**: `[x, y, theta, vx, vy, bias_accel_x, bias_accel_y, bias_angular_velocity]`
- **Coordinate Frame**: Planar XY + yaw (theta)
- **Update Rate**: ~50 Hz processing
- **Sensor Fusion**: iPhone IMU + optional GPS/Magnetometer; constraints (NHC, ZUPT, ZARU)

### iPhone Sensor Integration
- Real-time sensor data streaming via UDP (default)
- Automatic calibration and bias compensation
- Support for all iPhone motion sensors:
  - Accelerometer (3-axis)
  - Gyroscope (3-axis)
  - Magnetometer (3-axis)
  - GPS (when available)
  - Barometer

### Optional Autonomous Control
- Example controller available under `iphone_integration/robomaster_control/` (if present)

### Offline Analysis Tools
- **Python**: Comprehensive trajectory and error analysis
- **MATLAB**: Advanced visualization and performance metrics
- Batch processing for multiple log files
- Ground truth comparison capabilities

## Directory Structure

```
iphone_integration/
├── pi_phone_connection/
│   ├── iphone_sensor_receiver.py
│   ├── ekf_robomaster_8dof.py
│   └── main_integration_robomaster.py
├── robomaster_control/
├── offline_analysis/
│   ├── python/
│   └── matlab/
├── config/
│   └── system_config.json
├── data/
├── tests/
└── analysis_results/
```

## Quick Start

### 1. Hardware Setup

#### iPhone Mounting
1. Mount iPhone securely on the drone
2. Ensure rigid mounting to minimize vibrations
3. Note mounting position and orientation for configuration

#### Raspberry Pi Setup
```bash
# From repo root
pip install -r requirements.txt
cd iphone_integration
```

### 2. iPhone App Configuration

Install a sensor streaming app on your iPhone (e.g., "Sensor Logger" or custom app) that can stream:
- IMU data (accelerometer, gyroscope)
- GPS location (if available)
- Magnetometer data
- Barometer readings

Configure the app to stream to:
- Protocol: UDP
- IP: Raspberry Pi IP address
- Port: 5555 (default)
- Format: JSON

### 3. System Calibration

```bash
# Auto-calibration runs on start (keep iPhone stationary)
python pi_phone_connection/main_integration_robomaster.py

# Skip auto-calibration
python pi_phone_connection/main_integration_robomaster.py --no-calibrate
```

### 4. Start EKF Processing

```bash
# Start the integration system
python pi_phone_connection/main_integration_robomaster.py --config config/system_config.json
```

### 5. Autonomous Navigation

If enabled, autonomous control can consume EKF state and issue commands. See `iphone_integration/robomaster_control/`.

## Configuration

Edit `config/system_config.json` to customize:

### EKF Parameters
```json
"ekf_config": {
  "q_accel": 0.1,
  "q_gyro": 0.005,
  "q_accel_bias": 1e-5,
  "q_gyro_bias": 1e-4,
  "r_gps_pos": 0.5,
  "r_gps_vel": 0.2,
  "r_yaw": 0.2,
  "r_nhc": 0.05,
  "r_zupt": 0.005,
  "r_zaru": 0.0005
}
```

### Other Parameters
```json
"calibration": { "duration": 5.0, "min_samples": 100, "auto_calibrate": true },
"logging": { "enabled": true, "rate": 50 }
```

## Data Analysis

### Python Analysis
```bash
# Analyze single log file
python offline_analysis/python/ekf_analysis.py data/robomaster_ekf_log_*.csv

# Batch analysis
python offline_analysis/python/ekf_analysis.py data/ --batch
```

### MATLAB Analysis
```matlab
% Load and analyze data
analyzer = ekf_analyzer('data/robomaster_ekf_log_*.csv');
analyzer.plot_trajectory_3d();
analyzer.analyze_performance();
analyzer.generate_report('output_dir');
```

## EKF Algorithm Details

### State Prediction
Following the formulary specifications:
```
x_k = F_k * x_{k-1}
P_k = F_k * P_{k-1} * F_k^T + Q_k
```

### Measurement Update
Kalman update equations:
```
y = z - h (innovation)
S = H * P * H^T + R (innovation covariance)
K = P * H^T * S^(-1) (Kalman gain)
x = x + K * y (state update)
P = (I - K * H) * P * (I - K * H)^T + K * R * K^T
```

### IMU Measurement Model
Expected acceleration in body frame:
```python
expected_accel = [
    -gravity * sin(pitch),
    gravity * sin(roll) * cos(pitch),
    gravity * cos(roll) * cos(pitch)
]
```

## Performance Metrics

### Typical Performance (indicative)
- Planar position accuracy depends on GPS availability and motion
- Yaw stability improved via GPS course, magnetometer, and constraints
- Update rate ~50 Hz pipeline on Raspberry Pi 4

### Resource Usage
- **CPU**: ~30% on Raspberry Pi 4
- **Memory**: ~50MB
- **Storage**: ~10MB/minute of logging

## Troubleshooting

### Common Issues

1. **No sensor data received**
   - Check iPhone app configuration
   - Verify network connectivity
   - Check firewall settings

2. **Poor position estimates**
   - Run calibration procedure
   - Check sensor mounting
   - Adjust process noise parameters

3. **Control instability (if autonomous enabled)**
   - Reduce PID gains
   - Lower maximum velocity
   - Check for sensor delays

### Debug Mode
Enable verbose logging:
```json
"debug": {
  "verbose_logging": true,
  "save_raw_data": true,
  "log_level": "DEBUG"
}
```

## API Reference

Refer to:
- `pi_phone_connection/ekf_robomaster_8dof.py` (EKF methods)
- `pi_phone_connection/iphone_sensor_receiver.py` (receiver/processor APIs)

## Safety Considerations

1. **Always test in safe environment**
2. **Implement emergency stop procedures**
3. **Set conservative velocity limits initially**
4. **Monitor battery levels**
5. **Use geofencing for outdoor operations**

## Contributing

Please follow the coding standards:
- Python PEP 8 style guide
- Comprehensive docstrings
- Unit tests for new features
- Update documentation

## License

This project follows the RoboMaster SDK license agreements.

## References

1. **RoboMaster EKF Formulary** - Mathematical specifications
2. **RoboMaster SDK Documentation** - [Link](https://robomaster-dev.readthedocs.io/)
3. **Extended Kalman Filter Theory** - Probabilistic Robotics (Thrun et al.)

## Support

For issues and questions:
- Check the troubleshooting section
- Review the example scripts in `tests/`
- Consult the RoboMaster forums

## Changelog

### Version 1.0.0 (2025)
- Initial release for `main_integration_robomaster.py`
- 8-DOF EKF implementation
- iPhone sensor integration
- Optional autonomous control
- Offline analysis tools

---

**Note**: This system is designed for educational and research purposes. Always prioritize safety when operating autonomous vehicles.
