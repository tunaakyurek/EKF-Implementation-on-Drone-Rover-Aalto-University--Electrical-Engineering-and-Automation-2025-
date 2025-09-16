# RoboMaster S1 EKF Implementation

## System Overview
This project implements an Extended Kalman Filter (EKF) for state estimation on a RoboMaster S1 platform, running on a Raspberry Pi with real-time data transmission to a ground PC for visualization and analysis.

The EKF relies solely on sensor measurements (IMU, chassis encoders) without using thrust torque inputs, providing robust state estimation for autonomous navigation and control applications.

## Architecture
```
RoboMaster S1 ←→ Raspberry Pi ←→ Ground PC
    (Sensors)      (EKF Processing)   (Visualization)
```

## System Features

### 🚁 **Real-time State Estimation**
- 12-dimensional state vector: position, velocity, orientation, angular velocity
- 50 Hz EKF updates with IMU and chassis encoder fusion
- Automatic sensor bias estimation and calibration

### 📡 **Network Communication**
- Real-time data streaming from Pi to Ground PC
- ZeroMQ-based high-performance messaging
- Automatic reconnection and fault tolerance

### 📊 **Comprehensive Visualization**
- Live 2D trajectory plotting
- Real-time sensor data monitoring
- Uncertainty estimation visualization
- RMSE analysis and performance metrics

### 💾 **Data Management**
- Comprehensive data logging (CSV format)
- Session-based analysis and comparison
- Offline data analysis tools

## Quick Start

### 1. Environment Setup
```bash
# Install dependencies
python scripts/setup_environment.py

# Or manually:
pip install -r requirements.txt
```

### 2. Configuration
```bash
# Edit network configuration
nano config/network_config.json

# Edit system parameters
nano config/system_config.json
```

### 3. Run System

**On Raspberry Pi:**
```bash
python src/main.py
```

**On Ground PC:**
```bash
python src/ground_station.py
```

## System Components

### Core Modules
- **`src/ekf/`**: Extended Kalman Filter implementation
- **`src/robomaster_interface/`**: RoboMaster S1 communication
- **`src/data_collection/`**: Data logging and analysis
- **`src/visualization/`**: Real-time plotting and analysis
- **`src/communication/`**: Network communication

### Applications
- **`src/main.py`**: Main Raspberry Pi application
- **`src/ground_station.py`**: Ground PC visualization application

### Utilities
- **`scripts/setup_environment.py`**: Environment setup and validation
- **`scripts/run_tests.py`**: Comprehensive test suite
- **`config/`**: System and network configuration files

## Usage Examples

### Basic Operation
```bash
# Start the system on Raspberry Pi
python src/main.py

# Start visualization on Ground PC
python src/ground_station.py
```

### Advanced Configuration
```bash
# Custom configuration file
python src/main.py --config custom_config.json

# Analysis mode only
python src/ground_station.py --analysis

# No visualization (headless)
python src/ground_station.py --no-viz
```

### Testing
```bash
# Run full test suite
python scripts/run_tests.py

# Check system status
python scripts/setup_environment.py
```

## Data Analysis

### Real-time Analysis
The ground station provides real-time visualization of:
- 2D trajectory comparison (EKF vs chassis odometry)
- Position, velocity, and orientation time series
- IMU sensor data monitoring
- Estimation uncertainty (covariance trace)

### Offline Analysis
```bash
# Launch analysis mode
python src/ground_station.py --analysis

# Generate trajectory plots
python -c "
from src.visualization.real_time_plotter import AnalysisPlotter
from src.data_collection.data_logger import DataAnalyzer
analyzer = DataAnalyzer()
sessions = analyzer.get_available_sessions()
AnalysisPlotter.plot_rmse_analysis(sessions)
"
```

### RMSE Calculation
The system automatically calculates Root Mean Square Error (RMSE) between EKF estimates and chassis odometry:
- Position RMSE in X and Y dimensions
- Total trajectory RMSE
- Statistical analysis across multiple sessions

## System Architecture

For detailed information about the system architecture, communication protocols, and implementation details, see [SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md).

### Key Technical Specifications
- **EKF State**: 12D vector [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
- **Update Rate**: 50 Hz sensor processing
- **Communication**: ZeroMQ TCP/IP networking
- **Data Format**: JSON messaging, CSV logging
- **Platform**: Raspberry Pi 4, Python 3.7+

## Configuration

### Network Setup
Edit `config/network_config.json`:
```json
{
  "ground_pc_ip": "192.168.1.100",
  "port": 5555,
  "max_queue_size": 1000,
  "communication_rate": 100
}
```

### System Parameters
Edit `config/system_config.json`:
```json
{
  "sensor_frequency": 50,
  "enable_network": true,
  "enable_logging": true,
  "ekf_params": {
    "process_noise_scale": 1.0,
    "measurement_noise_scale": 1.0
  }
}
```

## Troubleshooting

### Common Issues

1. **RoboMaster Connection Failed**
   - Ensure RoboMaster S1 is powered on and in SDK mode
   - Check WiFi connection between Pi and RoboMaster
   - Verify RoboMaster SDK installation

2. **Network Communication Issues**
   - Check IP addresses in network configuration
   - Ensure firewall allows traffic on specified port
   - Verify Ground PC and Pi are on same network

3. **Performance Issues**
   - Monitor CPU usage on Raspberry Pi
   - Reduce sensor frequency if needed
   - Check available memory and storage

### Debug Mode
```bash
# Enable debug logging
python src/main.py --log-level DEBUG

# Test individual components
python scripts/run_tests.py
```

## Contributing

1. Follow PEP 8 style guidelines
2. Add tests for new functionality
3. Update documentation for changes
4. Test on actual hardware before submitting

## License

This project is developed for academic and research purposes. Please refer to individual component licenses for specific usage rights.

## Acknowledgments

- DJI RoboMaster S1 SDK
- Extended Kalman Filter implementation based on standard robotics literature
- Visualization components built with Matplotlib
- Network communication using ZeroMQ