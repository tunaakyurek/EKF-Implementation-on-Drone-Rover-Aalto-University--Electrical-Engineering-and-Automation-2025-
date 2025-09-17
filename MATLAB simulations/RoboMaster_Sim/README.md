# RoboMaster 2D Rover Simulation with 8-State EKF

## Overview
This simulation implements a 2D rover (land robot) with an 8-state Extended Kalman Filter (EKF) for state estimation. The rover operates in the XY-plane with car-like motion dynamics.

## State Vector
`[x, y, theta, v_x, v_y, b_a_x, b_a_y, b_w]`
- `x, y`: Position in meters
- `theta`: Yaw angle in radians
- `v_x, v_y`: Velocity components in m/s
- `b_a_x, b_a_y`: Accelerometer bias in m/s²
- `b_w`: Gyroscope bias in rad/s

## Features

### 1. **Post-Smoothing ON/OFF Switch**
The simulation now includes offline spike suppression and zero-phase smoothing, similar to the drone simulation.

**To enable/disable post-smoothing:**
```matlab
% In main_rover_sim.m, around the top of the file (near logging setup):
ENABLE_POST_SMOOTHING = true;  % <-- ONE-LINE SWITCH: true=ON, false=OFF
```

**What happens when enabled:**
- Applies median filtering to suppress spikes/outliers
- Applies Savitzky-Golay zero-phase smoothing
- Shows comparison plots: Raw EKF vs Smoothed EKF
- Animation displays both raw (red dashed) and smoothed (green solid) trajectories

**What happens when disabled:**
- Only raw EKF results are shown
- Faster execution (no post-processing)
- Standard error plots only

### 2. **Sensor Configuration**
- **IMU**: 100 Hz (accelerometer + gyroscope)
- **GPS**: 5 Hz (position)
- **Wheel encoders**: used for velocity updates (replace GPS velocity)
- **Magnetometer**: 10 Hz (yaw)
- **Noise levels**: Configurable in `parameters_rm.m`

### 3. **Motion Model**
- **Extended orbital dynamics**: 300-second simulation with complex multi-orbit patterns
- **Multi-orbit patterns**: Figure-8, spiral orbits, and orbital transitions for comprehensive testing
- **Car-like constraints**: Non-holonomic constraint (moves only in facing direction)
- **Steering-based rotation**: Lateral acceleration creates complex orbital patterns
- **Physical limits**: Max speed (5 m/s), max acceleration (3 m/s²) constraints
- **Dynamic motion**: Varying orbital radii and speeds for better EKF performance analysis

### 4. **Sensor Data Visualization**
- **Raw sensor plots**: Shows all raw measurements over time
- **Sensor vs. Estimated comparison**: Direct comparison of raw measurements vs. EKF estimates vs. true values
- **Sensor noise analysis**: Quantifies measurement noise for each sensor
- **Enhanced animation**: Displays GPS measurement points and sensor availability

## Files

### Core Simulation
- `main_rover_sim.m` - Main simulation script
- `parameters_rm.m` - All simulation parameters
- `rover_dynamics.m` - True motion model
- `sensor_model_wheel_encoders.m` - Sensor simulation (GPS position, wheel-encoder velocity, mag, IMU)

### EKF Implementation
- `ekf8_init.m` - EKF initialization
- `ekf8_predict.m` - EKF prediction step
- `ekf8_update_*.m` - Various measurement updates
- `ekf8_apply_constraints.m` - Constraint application
- `kalman_update.m` - Generic Kalman update

### Post-Processing
- `post_smooth_estimates_8state.m` - Spike suppression + smoothing for 8-state system
- `wrapToPi_local.m` - Angle normalization utility

### Visualization
- `animate_rover_xy.m` - XY-plane animation with optional smoothed trajectory

## Usage

1. **Run simulation:**
   ```matlab
   main_rover_sim
   ```

2. **Control post-smoothing:**
   - Edit `ENABLE_POST_SMOOTHING` in `main_rover_sim.m`
   - `true` = Enable post-smoothing (default)
   - `false` = Disable post-smoothing

3. **View results:**
   - **Consolidated plots**: Clean 2x2 subplot layouts for better readability
   - **XY trajectory & states**: Combined view of path and key state variables
   - **Raw sensor data**: Streamlined sensor measurement visualization
   - **Sensor vs. estimated comparison**: Direct comparison plots
   - **Sensor noise analysis**: Clean noise quantification plots
   - **Error analysis**: Consolidated error plots for easy comparison
   - **Animation**: 60-second duration with sensor measurements
   - **Post-smoothing comparison**: Clean Raw vs Smoothed EKF performance

## Post-Smoothing Parameters
- **Median filter window**: 0.12 seconds (spike suppression)
- **Savitzky-Golay window**: 0.24 seconds (smoothing)
- **Polynomial order**: 3 (preserves derivatives)

## Expected Results
With realistic sensor noise:
- **Raw EKF**: Some estimation error due to sensor noise
- **Smoothed EKF**: Reduced noise, smoother trajectories
- **Animation**: Visual comparison of all three trajectories
- **Error plots**: Quantitative comparison of performance


