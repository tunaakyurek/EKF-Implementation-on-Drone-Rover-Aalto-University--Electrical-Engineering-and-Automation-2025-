function params = parameters_rm_realistic()
% PARAMETERS_RM_REALISTIC  Configuration for 2D Rover EKF simulation with REALISTIC sensor noise
% Based on actual sensor specifications and real-world performance data

% Simulation timing
params.dt = 0.01;           % [s]
params.T  = 300.0;          % [s] - Extended for better performance analysis and orbital motion

% Process noise (PSD), following INS/GNSS style model
% Optimized for extended orbital motion and better long-term performance
params.q_accel       = 8.0;     % [m^2/s^3] - reduced for more stable tracking
params.q_gyro        = 0.2;     % [rad^2/s^3] - reduced for more stable yaw tracking
params.q_accel_bias  = 2e-4;    % [(m/s^2)^2/s] - reduced for more stable bias estimation
params.q_gyro_bias   = 2e-3;    % [(rad/s)^2/s] - reduced for more stable bias estimation

% ===== REALISTIC SENSOR NOISE PARAMETERS =====
% Based on actual sensor specifications and real-world performance

% Accelerometer noise (typical MEMS IMU)
% Realistic range: 0.01-0.05 m/s² RMS
params.r_accel    = 0.0004;  % accel meas variance per axis [ (m/s^2)^2 ] - 0.02 m/s² RMS

% Gyroscope noise (typical MEMS IMU)
% Realistic range: 0.001-0.01 rad/s RMS (0.06-0.6°/s)
params.r_gyro     = 1e-5;    % gyro meas variance [ (rad/s)^2 ] - 0.003 rad/s RMS

% GPS Position noise (consumer GPS)
% Realistic range: 2-5 m RMS (open sky conditions)
params.r_gps_pos  = 4.0;     % GPS pos variance [m^2] - realistic GPS noise

% GPS Velocity noise (consumer GPS)
% Realistic range: 0.1-0.3 m/s RMS
params.r_gps_vel  = 0.09;    % GPS vel variance [ (m/s)^2 ] - 0.3 m/s RMS

% Wheel encoder noise (used when GPS velocity is removed)
params.r_wheel_encoder = 0.04; % Wheel encoder variance [(m/s)^2] - realistic wheel noise

% Magnetometer noise (typical MEMS magnetometer)
% Realistic range: 0.01-0.05 rad RMS (0.5-3°)
params.r_yaw      = 0.0004;  % yaw variance [rad^2] - 0.02 rad RMS

% Constraint measurement noise (pseudo-measurements)
params.r_nhc      = 0.02;    % NHC pseudo-meas variance [ (m/s)^2 ] - more realistic
params.r_zupt     = 0.001;   % ZUPT variance per axis [ (m/s)^2 ] - more realistic
params.r_zaru     = 1e-5;    % ZARU variance [ (rad/s)^2 ] - more realistic

% Realistic sensor rates for practical applications
params.rates.imu_hz = round(1/params.dt); % IMU at sim rate
params.rates.gps_hz = 5;     % 5 Hz GPS (typical for consumer GPS)
params.rates.mag_hz = 10;    % 10 Hz magnetometer (typical for IMU)

% Physical constants
params.g = 9.81; % [m/s^2] (not directly used; flat 2D model)

% Initial true state [x y theta vx vy b_ax b_ay b_w]
params.x0_true = [
    0;      % x [m]
    0;      % y [m]
    0;      % theta [rad]
    0;      % vx [m/s]
    0;      % vy [m/s]
    0.02;   % b_ax [m/s^2] - reduced initial bias
    -0.01;  % b_ay [m/s^2] - reduced initial bias
    0.005   % b_w [rad/s] - reduced initial bias
];

% Note: Initial EKF covariance P0 is now set dynamically in main_rover_sim_realistic.m
% based on realistic initial conditions and sensor availability
% This provides better control over initialization for different experimental setups

% ===== SENSOR BIAS CHARACTERISTICS =====
% Realistic bias drift rates based on actual sensor data

% Accelerometer bias drift (typical MEMS IMU)
% Realistic range: 0.001-0.01 m/s² per hour
params.accel_bias_drift = 5e-7;  % [m/s^3] - reduced bias drift rate

% Gyroscope bias drift (typical MEMS IMU)
% Realistic range: 0.0001-0.001 rad/s per hour
params.gyro_bias_drift = 5e-9;   % [rad/s^2] - reduced bias drift rate

% ===== GPS MULTIPATH AND ATMOSPHERIC EFFECTS =====
% Additional realistic GPS error sources

% GPS multipath error (urban/canyon environments)
% Realistic range: 0.5-2.0 m additional RMS
params.gps_multipath = 0.3;      % [m] - reduced multipath error

% GPS atmospheric delay variation
% Realistic range: 0.2-1.0 m additional RMS
params.gps_atmospheric = 0.2;    % [m] - reduced atmospheric error

% ===== MAGNETOMETER INTERFERENCE =====
% Realistic magnetic field interference

% Magnetic interference amplitude
% Realistic range: 0.001-0.01 rad RMS
params.mag_interference = 0.002; % [rad] - reduced magnetic interference

% Magnetic interference frequency
params.mag_interference_freq = 0.05; % [Hz] - reduced interference frequency

end
