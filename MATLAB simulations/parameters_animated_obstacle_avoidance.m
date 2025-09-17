function params = parameters_animated_obstacle_avoidance()
%% parameters_animated_obstacle_avoidance.m - Optimized Parameters for Animated Obstacle Avoidance
% PURPOSE
% Specialized parameter set optimized for the animated obstacle avoidance test
% with improved EKF tuning for turning maneuvers and obstacle avoidance scenarios.
%
% KEY IMPROVEMENTS
% - Enhanced process noise for maneuvering scenarios
% - Optimized sensor noise parameters for obstacle avoidance
% - Improved EKF tuning for turning point performance
% - Better numerical conditioning parameters

%% 1. General Simulation & Timing Parameters
params.solver_type = 'Fixed-step';
params.Ts.physics = 0.01;   % Physics sim rate (100 Hz) - matches test_animated_obstacle_avoidance
params.Ts.IMU    = 0.01;    % IMU sample time (100 Hz) - high rate for maneuvers
params.Ts.GPS    = 0.1;     % GPS sample time (10 Hz)
params.Ts.Baro   = 0.02;    % Barometer sample time (50 Hz)
params.Ts.Mag    = 0.02;    % Magnetometer sample time (50 Hz)
params.sim_duration = 60;   % seconds

%% 2. Location-Specific Parameters (Espoo, Finland)
params.g = [0; 0; 9.81];   % Gravity (m/s^2), NED positive down
params.mag_NED = [15.16; 2.65; 49.78]; % Magnetic field (uT)

%% 3. Drone-Specific Physical Parameters (Optimized for Obstacle Avoidance)
params.arm_length = 0.125;      % meters
params.prop_radius = 0.0635;    % meters
params.CT = 1.2e-6;             % Thrust coefficient
params.CQ = 2.1e-8;             % Torque coefficient
params.mass = 0.5;              % kg
params.I = diag([0.0023, 0.0023, 0.004]); % kg*m^2
params.drag_coeff = 0.05;       % effective drag area coefficient

%% 4. Enhanced Sensor Noise & Error Parameters (Optimized for Obstacle Avoidance)
% IMU parameters - slightly more conservative for maneuvering scenarios
params.IMU.accel_noise_density = 0.01;   % (m/s^2)/sqrt(Hz) - increased for robustness
params.IMU.accel_bias_instab  = 0.005;   % m/s^2 - reduced for better stability
params.IMU.gyro_noise_density = 0.001;   % (rad/s)/sqrt(Hz) - increased for turning robustness
params.IMU.gyro_bias_instab   = 0.0001;  % rad/s - reduced for better stability

% GPS parameters - optimized for obstacle avoidance scenarios
params.GPS.sigma_xy = 1.5;               % meters (horizontal) - slightly increased for robustness
params.GPS.sigma_z  = 2.0;               % meters (vertical) - increased for 3D maneuvering

% Barometer parameters - optimized for altitude control during maneuvers
params.Baro.sigma_z = 0.4;               % meters - slightly increased for robustness

% Magnetometer parameters - optimized for heading control during turns
params.Mag.sigma_deg = 2.5;              % degrees - slightly increased for turning robustness
params.Mag.sigma_rad = params.Mag.sigma_deg * pi/180; % radians

%% 5. ENHANCED EKF Tuning Parameters (Optimized for Turning Maneuvers)
% IMPROVED: Process noise matrix specifically tuned for obstacle avoidance scenarios
% Position: Low uncertainty for smooth tracking
% Velocity: Increased uncertainty for better maneuver tracking
% Attitude: Significantly increased for turning point robustness

% Base process noise (9x9 matrix for 9-state EKF)
params.Q = diag([
    0.02, 0.02, 0.02,           % Position uncertainty (m^2/s^2)
    0.15, 0.15, 0.18,           % Velocity uncertainty (m^2/s^4) - INCREASED for maneuvers
    0.08, 0.08, 0.10            % Attitude uncertainty (rad^2/s^2) - INCREASED for turns
]);

% Measurement noise matrices
params.R_gps = diag([params.GPS.sigma_xy^2, params.GPS.sigma_xy^2, params.GPS.sigma_z^2]);
params.R_baro = params.Baro.sigma_z^2;
params.R_mag = params.Mag.sigma_rad^2;

%% 6. Additional EKF Enhancement Parameters
% Maximum angular rate for numerical stability
params.max_angular_rate = deg2rad(200);  % rad/s

% Maneuver detection thresholds
params.maneuver.omega_threshold = deg2rad(30);  % rad/s - angular rate threshold
params.maneuver.accel_threshold = 2.0;          % m/s^2 - acceleration threshold
params.maneuver.vel_threshold = 3.0;            % m/s - velocity threshold

% Process noise enhancement factors for maneuvers
params.maneuver.omega_factor = 1.5;      % Factor for high angular rates
params.maneuver.accel_factor = 1.3;      % Factor for high accelerations
params.maneuver.vel_factor = 1.2;        % Factor for high velocities

%% 7. Numerical Conditioning Parameters
% Covariance conditioning limits
params.conditioning.min_singular_value = 1e-12;  % Minimum singular value
params.conditioning.max_singular_value = 1e6;    % Maximum singular value

% Innovation covariance conditioning
params.conditioning.innovation_regularization = 1e-6;  % Regularization term
params.conditioning.max_condition_number = 1e12;       % Maximum condition number

%% 8. Obstacle Avoidance Specific Parameters
% Drone physical parameters for collision detection
params.drone.radius = 0.35;              % meters - drone collision radius
params.drone.safety_margin = 0.8;        % meters - additional safety margin

% Guidance parameters
params.guidance.max_velocity = 5.0;      % m/s - maximum velocity
params.guidance.max_yaw_rate = deg2rad(45); % rad/s - maximum yaw rate

% VFH parameters
params.vfh.distance_limits = [0.5, 8.0]; % meters - VFH distance limits
params.vfh.robot_radius = 0.5;           % meters - VFH robot radius
params.vfh.safety_distance = 0.8;        % meters - VFH safety distance
params.vfh.num_angular_sectors = 72;     % VFH angular sectors

% Control gains for autonomous flight
params.control.Kp_pos = [1.5; 1.5; 2.0]; % Position control gains
params.control.Kd_vel = [0.8; 0.8; 1.2]; % Velocity control gains
params.control.Kp_att = [0.8; 0.8; 0.4]; % Attitude control gains
params.control.Kd_att = [0.3; 0.3; 0.15]; % Attitude rate control gains

%% 9. Animation and Visualization Parameters
params.animation.update_rate = 1;        % Update animation every N steps
params.animation.goal_tolerance = 1.5;   % meters - goal reaching tolerance
params.animation.progress_interval = 20; % Show progress every N% of simulation

fprintf('Loaded optimized parameters for animated obstacle avoidance\n');
fprintf('  - Enhanced EKF tuning for turning maneuvers\n');
fprintf('  - Optimized sensor noise parameters\n');
fprintf('  - Improved numerical conditioning\n');
fprintf('  - Obstacle avoidance specific settings\n');
end
