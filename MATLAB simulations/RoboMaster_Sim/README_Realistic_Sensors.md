# RoboMaster Simulation with Realistic Sensors

## Overview
This document describes the new realistic sensor simulation that addresses the overly noisy sensor behavior in the original implementation. The new simulation uses realistic noise parameters based on actual sensor specifications and real-world performance data.

## 🚨 Problem with Original Implementation

### **Excessive Noise Levels:**
- **GPS Position**: ±0.5-1.0m (should be ±2-5m)
- **GPS Velocity**: ±1.0-1.5 m/s (should be ±0.1-0.3 m/s)
- **Magnetometer**: ±0.1-0.2 rad (should be ±0.01-0.05 rad)
- **Gyroscope**: ±0.1-0.2 rad/s (should be ±0.001-0.01 rad/s)

### **Why This Happened:**
The original sensor model used simplified noise parameters that were too aggressive, leading to unrealistic sensor behavior that made EKF performance analysis difficult.

## ✅ Solution: Realistic Sensor Models

### **New/Updated Files:**
1. **`main_rover_sim_realistic.m`** - Main simulation using realistic parameters and wheel-encoder velocity updates
2. **`parameters_rm_realistic.m`** - Realistic sensor noise parameters
3. **`sensor_model_wheel_encoders.m`** - Default sensor model (GPS position, wheel-encoder velocity, IMU, magnetometer)
4. **`sensor_model_realistic.m`** - Optional standalone model demonstrating realistic noise and quality indicators

### **Realistic Sensor Specifications:**

#### **GPS (Consumer Grade):**
- **Position Accuracy**: 2.0 m RMS (typical open sky conditions)
- **Velocity Accuracy**: 0.2 m/s RMS
- **Additional Errors**:
  - Multipath: ±0.5m (urban/canyon environments)
  - Atmospheric: ±0.3m (ionospheric delay variations)

#### **MEMS IMU (Typical):**
- **Accelerometer**: 0.01 m/s² RMS noise
- **Gyroscope**: 0.001 rad/s RMS noise
- **Bias Drift**: Realistic slow variations over time

#### **Magnetometer:**
- **Accuracy**: 0.01 rad RMS (0.5°)
- **Interference**: Simulates nearby metal objects, power lines
- **Field Strength**: Realistic magnetic field measurements

## 🔧 How to Use

### **1. Run Realistic Simulation:**
```matlab
main_rover_sim_realistic
```

### **2. Compare with Original:**
```matlab
% Original (standard sensors with wheel-encoder velocity)
main_rover_sim

% Realistic parameters version
main_rover_sim_realistic
```

### **3. Key Differences:**
- **Sensor updates**: Wheel-encoder velocity replaces GPS velocity in both versions
- **Noise levels**: Realistic parameters for GPS, IMU, magnetometer
- **Error sources**: Optional multipath/atmospheric/interference models
- **Performance**: Closer to real-world EKF behavior; better for tuning

### **4. Realistic EKF Initialization:**
The new simulation uses **realistic initial conditions** typical of rover experimentation:

#### **Position Initialization:**
- **With GPS**: Uses GPS position with ±1m uncertainty
- **Without GPS**: Assumes origin with ±5m uncertainty
- **No random initialization**: Based on actual sensor data or reasonable assumptions

#### **Heading Initialization:**
- **With Magnetometer**: Uses compass reading with ±0.1 rad uncertainty
- **Without Magnetometer**: Assumes north (0°) with ±0.5 rad uncertainty
- **Proper angle handling**: Wrapped to [-π, π] range

#### **Velocity Initialization:**
- **Assumes rest**: [0, 0] m/s with ±0.1 m/s uncertainty
- **Wheel encoders**: Provide early velocity updates when available

#### **Bias Initialization:**
- **Accelerometer**: 0 m/s² with ±0.1 m/s² uncertainty
- **Gyroscope**: 0 rad/s with ±0.01 rad/s uncertainty
- **High initial uncertainty**: Allows EKF to learn biases during operation

#### **Initial Measurement Updates:**
- **Conditional updates**: Only applies sensor updates if measurements are valid
- **Uncertainty reduction**: Helps refine initial estimates without overriding realistic starting conditions
- **Robust handling**: Gracefully handles missing or invalid sensor data

## 📊 Expected Results

### **Realistic Noise Ranges:**
- **GPS Position**: ±2-5m (vs. ±0.5-1.0m before)
- **GPS Velocity**: ±0.1-0.3 m/s (vs. ±1.0-1.5 m/s before)
- **Magnetometer**: ±0.01-0.05 rad (vs. ±0.1-0.2 rad before)
- **Gyroscope**: ±0.001-0.01 rad/s (vs. ±0.1-0.2 rad/s before)

### **EKF Performance:**
- **Better Convergence**: Realistic noise allows proper EKF learning
- **Stable Tracking**: Reduced false alarms and divergence
- **Realistic Errors**: Performance metrics match real-world expectations

## 🎯 MATLAB Built-in Features Used

### **1. Random Number Generation:**
```matlab
% Proper Gaussian noise
noise = sqrt(variance) * randn();

% Persistent variables for correlated errors
persistent multipath_x multipath_y
```

### **2. Mathematical Functions:**
```matlab
% Trigonometric transformations
a_x_true = a_forward * cos(theta) - a_lateral * sin(theta);

% Angle wrapping
yaw_meas = mod(yaw_meas + pi, 2*pi) - pi;
```

### **3. Advanced Error Modeling:**
- **Multipath Simulation**: Correlated GPS errors
- **Atmospheric Effects**: Slowly varying ionospheric delays
- **Magnetic Interference**: Realistic electromagnetic disturbances

## 🔍 Sensor Quality Indicators

The new sensor model provides additional realistic information:

### **GPS Quality:**
- **HDOP**: Horizontal dilution of precision
- **Satellites**: Number of visible satellites

### **Magnetometer Quality:**
- **Field Strength**: Magnetic field magnitude
- **Interference Level**: Current interference amplitude

### **IMU Quality:**
- **Temperature**: Sensor temperature
- **Calibration Status**: Calibration state

## 📈 Performance Analysis

### **Before (Original):**
- Excessive noise masked EKF performance
- Difficult to distinguish algorithm issues from sensor problems
- Unrealistic error magnitudes

### **After (Realistic):**
- Clean sensor data reveals true EKF performance
- Realistic error bounds for algorithm evaluation
- Better understanding of EKF convergence and stability

## 🚀 Next Steps

### **1. Run Both Simulations:**
Compare the performance and noise characteristics.

### **2. Analyze EKF Behavior:**
- Convergence speed
- Steady-state error
- Bias estimation accuracy

### **3. Tune EKF Parameters:**
Use realistic sensors to optimize:
- Process noise (Q matrix)
- Initial covariance (P0)
- Measurement noise (R matrix)

## 💡 Technical Notes

### **Noise Distributions:**
- **Gaussian (Normal)**: Most sensor noise follows this distribution
- **Correlated Errors**: GPS multipath and atmospheric effects
- **Bias Drift**: Slow random walks typical of MEMS sensors

### **Error Sources:**
- **Random Noise**: Fundamental sensor limitations
- **Systematic Errors**: Bias, scale factor, misalignment
- **Environmental**: Temperature, magnetic fields, multipath

### **Calibration:**
- **Factory Calibration**: Basic sensor corrections
- **Online Calibration**: EKF bias estimation
- **Environmental Compensation**: Temperature and magnetic field

## 🔗 Related Files

- **Original**: `main_rover_sim.m`, `parameters_rm.m`, `sensor_model_wheel_encoders.m`
- **Realistic**: `main_rover_sim_realistic.m`, `parameters_rm_realistic.m`, `sensor_model_realistic.m` (optional)
- **Common**: All EKF functions, dynamics, animation

## 📚 References

- **GPS Accuracy**: RTCA DO-229D, GPS Standard Positioning Service
- **IMU Specifications**: MEMS sensor datasheets (ST, Bosch, InvenSense)
- **Magnetometer**: Magnetic field standards and interference models
- **EKF Tuning**: Navigation and sensor fusion literature
