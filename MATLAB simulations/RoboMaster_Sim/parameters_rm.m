function params = parameters_rm()
% PARAMETERS_RM  Configuration for 2D Rover EKF simulation

% Simulation timing
params.dt = 0.01;           % [s]
params.T  = 300.0;          % [s] - Extended for better performance analysis and orbital motion

% Process noise (PSD), following INS/GNSS style model
% Optimized for extended orbital motion and better long-term performance
params.q_accel       = 12.0;    % [m^2/s^3] - increased for agility and velocity tracking
params.q_gyro        = 0.3;     % [rad^2/s^3] - increased for yaw bias/lock
params.q_accel_bias  = 5e-4;    % [(m/s^2)^2/s] - increased for faster bias learning in long runs
params.q_gyro_bias   = 5e-3;    % [(rad/s)^2/s] - increased for faster bias learning in long runs

% Measurement noise (variances)
params.r_accel    = 0.1;    % accel meas variance per axis [ (m/s^2)^2 ]
params.r_gyro     = 0.01;   % gyro meas variance [ (rad/s)^2 ]
% Trust measurements more to reduce lag
params.r_gps_pos  = 0.20;   % GPS pos variance [m^2] - trust GPS a bit more
params.r_gps_vel  = 0.2;    % GPS vel variance [ (m/s)^2 ] (kept; not used in wheel setup)
% Wheel encoder noise (used when GPS velocity is removed)
params.r_wheel_encoder = 0.005; % Wheel encoder variance [(m/s)^2] - trust wheels more
params.r_yaw      = 0.05;   % yaw variance [rad^2] - trust magnetometer more
params.r_nhc      = 0.03;   % NHC pseudo-meas variance [ (m/s)^2 ] - tighter lateral constraint
params.r_zupt     = 0.005;  % ZUPT variance per axis [ (m/s)^2 ] - tighter stationarity
params.r_zaru     = 0.001;  % ZARU variance [ (rad/s)^2 ]

% Realistic sensor rates for practical applications
params.rates.imu_hz = round(1/params.dt); % IMU at sim rate
params.rates.gps_hz = 5;     % 5 Hz GPS (typical for consumer GPS)
params.rates.mag_hz = 10;    % 10 Hz magnetometer (typical for IMU)

% Physical constants
params.g = 9.81; % [m/s^2] (not directly used; flat 2D model)

% Sensor bias drift characteristics (used by wheel encoder/realistic sensor models)
params.accel_bias_drift = 1e-6;  % [m/s^3] - accelerometer bias RW rate
params.gyro_bias_drift  = 1e-8;  % [rad/s^2] - gyroscope bias RW rate

% Additional GPS and magnetometer environment effects
params.gps_multipath    = 0.5;   % [m] additional multipath RMS
params.gps_atmospheric  = 0.3;   % [m] atmospheric delay variation RMS
params.mag_interference = 0.005; % [rad] magnetic interference amplitude
params.mag_interference_freq = 0.1; % [Hz] interference frequency

% Initial true state [x y theta vx vy b_ax b_ay b_w]
params.x0_true = [
    0;      % x [m]
    0;      % y [m]
    0;      % theta [rad]
    0;      % vx [m/s]
    0;      % vy [m/s]
    0.05;   % b_ax [m/s^2]
    -0.03;  % b_ay [m/s^2]
    0.01    % b_w [rad/s]
];

% Initial EKF covariance P - optimized for extended orbital motion
params.P0 = eye(8);
params.P0(1:2,1:2) = params.P0(1:2,1:2) * 1.5;   % balanced pos var for stable GPS tracking
params.P0(3,3)     = 0.2;                        % reduced yaw var for faster mag lock
params.P0(4:5,4:5) = params.P0(4:5,4:5) * 0.3;   % reduced vel var for faster GPS vel lock
params.P0(6:7,6:7) = params.P0(6:7,6:7) * 0.1;   % reduced accel bias var for faster learning
params.P0(8,8)     = 0.02;                       % reduced gyro bias var for faster learning

end


