function params = parameters_rm_optimized()
% PARAMETERS_RM_OPTIMIZED  Optimized configuration for 2D Rover EKF simulation
% Addresses performance issues while maintaining proper observability

% Simulation timing
params.dt = 0.01;           % [s]
params.T  = 300.0;          % [s] - Extended for better performance analysis

% ===== OPTIMIZED PROCESS NOISE (Q) =====
% Increased to handle the observed performance issues
params.q_accel       = 12.0;    % [m^2/s^3] - INCREASED from 8.0 to handle position drift
params.q_gyro        = 0.3;     % [rad^2/s^3] - INCREASED from 0.2 to handle yaw bias
params.q_accel_bias  = 1e-3;    % [(m/s^2)^2/s] - INCREASED for faster bias learning
params.q_gyro_bias   = 1e-2;    % [(rad/s)^2/s] - INCREASED for faster bias learning

% ===== OPTIMIZED MEASUREMENT NOISE (R) =====
% Balanced to maintain observability while reducing noise sensitivity
params.r_accel    = 0.0001;  % accel meas variance per axis [ (m/s^2)^2 ]
params.r_gyro     = 1e-6;    % gyro meas variance [ (rad/s)^2 ]

% GPS noise - balanced for observability
params.r_gps_pos  = 2.0;     % GPS pos variance [m^2] - REDUCED from 4.0 for better tracking

% Wheel encoder noise (replaces GPS velocity)
params.r_wheel_encoder = 0.01;  % Wheel encoder variance [(m/s)^2] - Much better than GPS velocity

% Magnetometer - balanced for yaw estimation
params.r_yaw      = 0.0001;  % yaw variance [rad^2]

% Constraint measurement noise (pseudo-measurements)
params.r_nhc      = 0.01;    % NHC pseudo-meas variance [ (m/s)^2 ]
params.r_zupt     = 0.0001;  % ZUPT variance per axis [ (m/s)^2 ]
params.r_zaru     = 1e-6;    % ZARU variance [ (rad/s)^2 ]

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
    0.05;   % b_ax [m/s^2]
    -0.03;  % b_ay [m/s^2]
    0.01    % b_w [rad/s]
];

% ===== OPTIMIZED INITIAL COVARIANCE (P0) =====
% Balanced initial uncertainty for stable convergence
params.P0 = eye(8);
params.P0(1:2,1:2) = params.P0(1:2,1:2) * 2.0;   % position uncertainty - BALANCED
params.P0(3,3)     = 0.1;                          % yaw uncertainty - REDUCED for faster lock
params.P0(4:5,4:5) = params.P0(4:5,4:5) * 0.5;   % velocity uncertainty - REDUCED for faster lock
params.P0(6:7,6:7) = params.P0(6:7,6:7) * 0.2;   % accel bias uncertainty - REDUCED for faster learning
params.P0(8,8)     = 0.01;                         % gyro bias uncertainty - REDUCED for faster learning

% ===== SENSOR BIAS CHARACTERISTICS =====
% Realistic bias drift rates
params.accel_bias_drift = 1e-6;  % [m/s^3] - bias drift rate
params.gyro_bias_drift = 1e-8;   % [rad/s^2] - bias drift rate

% ===== GPS MULTIPATH AND ATMOSPHERIC EFFECTS =====
params.gps_multipath = 0.5;      % [m] - additional multipath error
params.gps_atmospheric = 0.3;    % [m] - additional atmospheric error

% ===== MAGNETOMETER INTERFERENCE =====
params.mag_interference = 0.005; % [rad] - magnetic interference amplitude
params.mag_interference_freq = 0.1; % [Hz] - interference frequency

end
