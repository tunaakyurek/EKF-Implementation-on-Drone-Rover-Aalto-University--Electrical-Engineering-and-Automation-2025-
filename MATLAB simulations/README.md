## EKF Drone (9‑State) and RoboMaster Rover (8‑State) Simulations (MATLAB)

This repository contains two complementary sensor‑fusion simulations:
- A 6‑DOF multirotor with a 9‑state EKF (IMU‑predicted, GPS/Baro/Mag updates)
- A 2D RoboMaster‑style rover with an 8‑state EKF (IMU‑predicted, GPS position, magnetometer yaw, and wheel‑encoder velocity updates)

### Drone (9‑state) contents
- `main_random.m`: Runs the end‑to‑end simulation, logging and playback.
- `parameters.m`: Central configuration (timing, vehicle, sensor noise, EKF tuning).
- `drone_dynamics.m`: Nonlinear 9‑state truth model with simplified cross‑coupling.
- `sensor_model.m`: Generates noisy IMU/GPS/Baro/Mag measurements from truth.
- `ekf_sensor_only.m`: EKF with IMU‑only prediction and sensor updates.
- `drone_dynamics_imu.m`: Strapdown mechanization used in EKF prediction.
- `calculate_F_sensor_only.m`: State Jacobian used in EKF covariance update.
- `rotation_matrix.m`: ZYX yaw‑pitch‑roll rotation helper.

### Drone quick start
1) Open MATLAB.
2) Set the working folder to the project root.
3) Open `parameters.m` and select a profile: set `profile = 'QAV250'` or `'S500'`.
4) Run `main_random.m`.

You should see console progress during the run, followed by an animation and optional analysis plots.

### Drone simulation overview
- State vector: `[x y z vx vy vz roll pitch yaw]` in NED, radians for angles.
- Truth integration: `drone_dynamics.m` uses thrust and torques computed from a smooth velocity command in `main_random.m`.
- Sensors: `sensor_model.m` outputs IMU (specific force, body rates), GPS (position), barometer (altitude = −z), magnetometer (yaw).
- EKF: `ekf_sensor_only.m` predicts via `drone_dynamics_imu.m` (IMU only) and updates per sensor at their own sample rates. The linearized transition `F` comes from `calculate_F_sensor_only.m`.

### Drone key tunables
- Timing: `params.Ts.physics`, `params.Ts.IMU`, `params.Ts.GPS`, `params.Ts.Baro`, `params.Ts.Mag`, `params.sim_duration`.
- Vehicle: `params.mass`, `params.I` (set via profile in `parameters.m`).
- EKF process noise: `params.Q` (diagonal 9×9, scaled by `dt` in code).
- Measurement noise: `params.R_gps`, `params.R_baro`, `params.R_mag`.

### Drone: how it fits together
1) `main_random.m` loads `parameters.m`, initializes state/covariance, and iterates over time.
2) It generates a smooth random‑walk target velocity (Ornstein–Uhlenbeck), converts to thrust/torques, and integrates the truth model.
3) `sensor_model.m` produces measurements with noise/bias.
4) `ekf_sensor_only.m` runs: predict at IMU rate using `drone_dynamics_imu.m`, and update with GPS/Baro/Mag at their rates.
5) Results are logged, animated, and optionally analyzed.

### Drone notes and conventions
- Frames: Body (b) and NED (n). `rotation_matrix.m` maps body→NED via ZYX.
- Barometer returns altitude, which is `−z` in NED convention.
- Multiple numerical safety checks (regularization, SVD conditioning, angle clamps) keep the simulation stable.

### Drone troubleshooting
- If the animation or analysis functions are missing in your MATLAB path, comment out those calls in `main_random.m` or add your own plotting. The core simulation and EKF will still run.
- If you observe NaN/Inf warnings, try reducing `params.sim_duration`, loosening `params.Q`, or lowering controller gains in `main_random.m`.
---

### Rover (8‑state) overview
- Entry point: `RoboMaster_Sim/main_rover_sim.m`
- Realistic sensors variant: `RoboMaster_Sim/main_rover_sim_realistic.m`
- Parameters: `RoboMaster_Sim/parameters_rm.m`, `RoboMaster_Sim/parameters_rm_realistic.m`
- Core EKF files: `RoboMaster_Sim/ekf8_*.m`, prediction in `ekf8_predict.m`, measurement updates in `ekf8_update_*.m`
- Sensors: IMU (accel, gyro), GPS (position), magnetometer (yaw), wheel encoders (velocity)

State vector `[x, y, theta, v_x, v_y, b_a_x, b_a_y, b_w]`.

Key details matching the code:
- Wheel‑encoder velocity replaces GPS velocity updates (see `sensor_model_wheel_encoders.m` and `ekf8_update_wheel_vel.m`).
- Post‑smoothing switch is a single line in `main_rover_sim.m`:
  `ENABLE_POST_SMOOTHING = true; % true=ON, false=OFF`
- Animation shows raw vs smoothed estimates when smoothing is enabled (see `animate_rover_xy.m`).

Rover quick start:
1) Open MATLAB and set the working folder to the repo root.
2) Run `RoboMaster_Sim/main_rover_sim.m` (standard sensors) or `RoboMaster_Sim/main_rover_sim_realistic.m`.
3) Toggle the post‑smoothing line in `main_rover_sim.m` if desired.

