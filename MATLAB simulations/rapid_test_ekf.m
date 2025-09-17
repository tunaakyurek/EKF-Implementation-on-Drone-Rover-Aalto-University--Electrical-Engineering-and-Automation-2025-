function results = rapid_test_ekf(varargin)
%% RAPID EKF-ONLY TESTING - No Animation, Fast Parameter Tuning
% Quickly test EKF-9 performance with different parameters and initial conditions
% Focuses ONLY on the 9-state EKF (no other filters for speed)

% Add necessary paths for EKF functions
addpath('noanim_benchmarks/filters');
%
% USAGE:
%   results = rapid_test_ekf()                    % Default parameters
%   results = rapid_test_ekf('Q_scale', 1.5)     % Scale process noise
%   results = rapid_test_ekf('duration', 30)     % Shorter test
%   results = rapid_test_ekf('init_pos', [10;5;2]) % Different initial position
%
% PARAMETERS TO TUNE:
%   'duration'     - Simulation duration (default: 60s for full performance)
%   'Q_scale'      - Process noise scaling factor (default: 1.0)
%   'R_gps_scale'  - GPS measurement noise scaling (default: 1.0)
%   'R_baro_scale' - Barometer noise scaling (default: 1.0)
%   'R_mag_scale'  - Magnetometer noise scaling (default: 1.0)
%   'init_pos'     - Initial position [x;y;z] (default: [0;0;0])
%   'init_vel'     - Initial velocity [vx;vy;vz] (default: [0;0;0])
%   'init_att'     - Initial attitude [roll;pitch;yaw] in deg (default: [0;0;0])
%   'P_scale'      - Initial covariance scaling (default: 1.0)
%   'verbose'      - Show progress (default: true)
%
% OUTPUT:
%   results - Structure with EKF performance metrics and timing

%% Parse input parameters
p = inputParser;
addParameter(p, 'duration', 60, @(x) isnumeric(x) && x > 0);
addParameter(p, 'Q_scale', 1.0, @(x) isnumeric(x) && x > 0);
addParameter(p, 'R_gps_scale', 1.0, @(x) isnumeric(x) && x > 0);
addParameter(p, 'R_baro_scale', 1.0, @(x) isnumeric(x) && x > 0);
addParameter(p, 'R_mag_scale', 1.0, @(x) isnumeric(x) && x > 0);
addParameter(p, 'Q_att_scale', 1.0, @(x) isnumeric(x) && x > 0);  % Attitude process noise scaling
addParameter(p, 'init_pos', [0;0;0], @(x) isnumeric(x) && length(x) == 3);
addParameter(p, 'init_vel', [0;0;0], @(x) isnumeric(x) && length(x) == 3);
addParameter(p, 'init_att', [0;0;0], @(x) isnumeric(x) && length(x) == 3);
addParameter(p, 'P_scale', 1.0, @(x) isnumeric(x) && x > 0);
addParameter(p, 'verbose', true, @islogical);
parse(p, varargin{:});

% Extract parameters
duration = p.Results.duration;
Q_scale = p.Results.Q_scale;
R_gps_scale = p.Results.R_gps_scale;
R_baro_scale = p.Results.R_baro_scale;
R_mag_scale = p.Results.R_mag_scale;
Q_att_scale = p.Results.Q_att_scale;
init_pos = p.Results.init_pos(:);
init_vel = p.Results.init_vel(:);
init_att = deg2rad(p.Results.init_att(:));
P_scale = p.Results.P_scale;
verbose = p.Results.verbose;

if verbose
    fprintf('=== RAPID EKF TEST ===\n');
    fprintf('Duration: %.1fs, Q_scale: %.2f, P_scale: %.2f\n', duration, Q_scale, P_scale);
    fprintf('Initial: pos=[%.1f %.1f %.1f], vel=[%.1f %.1f %.1f], att=[%.1f %.1f %.1f] deg\n', ...
        init_pos(1), init_pos(2), init_pos(3), init_vel(1), init_vel(2), init_vel(3), ...
        rad2deg(init_att(1)), rad2deg(init_att(2)), rad2deg(init_att(3)));
    fprintf('Noise scales: GPS=%.2f, Baro=%.2f, Mag=%.2f\n', R_gps_scale, R_baro_scale, R_mag_scale);
end

%% Load and modify parameters
parameters; % Load base parameters

% Modify simulation duration
params.sim_duration = duration;

% Scale process noise
params.Q = params.Q * Q_scale;

% Scale attitude process noise separately for better attitude tuning
params.Q(7:9, 7:9) = params.Q(7:9, 7:9) * Q_att_scale;

% Scale measurement noise
params.R_gps = params.R_gps * R_gps_scale;
params.R_baro = params.R_baro * R_baro_scale;
params.R_mag = params.R_mag * R_mag_scale;

%% Simulation setup
dt = params.Ts.physics;
T_end = params.sim_duration;
t = 0:dt:T_end;
N = length(t);

% Initial conditions
x_true = [init_pos; init_vel; init_att];
x_est = x_true; % Start with perfect knowledge
P = diag([0.5^2, 0.5^2, 0.4^2, 0.2^2, 0.2^2, 0.2^2, deg2rad(2)^2, deg2rad(2)^2, deg2rad(3)^2]) * P_scale;

% Storage
x_true_hist = zeros(9, N);
x_est_hist = zeros(9, N);
imu_hist = zeros(6, N);
gps_hist = NaN(3, N);
baro_hist = NaN(1, N);
mag_hist = NaN(1, N);

% Performance metrics
pos_error_hist = NaN(1, N);
vel_error_hist = NaN(1, N);
att_error_hist = NaN(1, N);

% Random walk parameters
lambda_v = 0.6;
sigma_v = [0.8; 0.8; 0.5];
v_max = [3; 3; 1.5];
vel_cmd_target = zeros(3,1);

% Controller gains
Kp_vel = [0.6; 0.6; 0.8];
Kp_att = [0.8; 0.8; 0.4];
Kd_att = [0.3; 0.3; 0.15];
prev_att_err = zeros(3,1);
max_tilt = deg2rad(10);

warmup_T = 2.0;
warmup_steps = min(N, round(warmup_T/dt));

%% Main simulation loop
tic;
for k = 1:N
    current_time = t(k);
    
    %% Command generation
    if k <= warmup_steps
        vel_cmd = [0;0;0];
    else
        dW = sqrt(dt) * randn(3,1);
        vel_cmd_target = vel_cmd_target + (-lambda_v .* vel_cmd_target) * dt + sigma_v .* dW;
        vel_cmd = max(min(vel_cmd_target, v_max), -v_max);
    end
    
    %% Control law
    vel_est = x_est(4:6);
    att_est = x_est(7:9);
    vel_error = vel_cmd - vel_est;
    accel_des = Kp_vel .* vel_error;
    accel_des(3) = accel_des(3) + abs(params.g(3));
    
    accel_norm = norm(accel_des);
    if accel_norm < 1e-6
        accel_des = [0;0;abs(params.g(3))];
        accel_norm = norm(accel_des);
    end
    
    z_body_des = accel_des / accel_norm;
    roll_des = asin(max(min(-z_body_des(2), sin(max_tilt)), -sin(max_tilt)));
    if abs(cos(roll_des)) > 1e-6
        pitch_des = asin(max(min(z_body_des(1) / cos(roll_des), sin(max_tilt)), -sin(max_tilt)));
    else
        pitch_des = 0;
    end
    
    if norm(vel_cmd(1:2)) > 0.3
        yaw_des = atan2(vel_cmd(2), vel_cmd(1));
    else
        yaw_des = att_est(3);
    end
    
    att_ref = [roll_des; pitch_des; yaw_des];
    att_err = att_ref - att_est;
    att_err(3) = atan2(sin(att_err(3)), cos(att_err(3)));
    att_derr = (att_err - prev_att_err) / dt;
    prev_att_err = att_err;
    
    tau_cmd = Kp_att .* att_err + Kd_att .* att_derr;
    max_alpha = deg2rad(200);
    tau_cmd = max(min(tau_cmd, max_alpha), -max_alpha);
    tau = params.I * tau_cmd;
    
    thrust = params.mass * accel_norm;
    max_thrust = 2.0 * params.mass * abs(params.g(3));
    min_thrust = 0.1 * params.mass * abs(params.g(3));
    thrust = max(min(thrust, max_thrust), min_thrust);
    
    max_torque = 0.08;
    tau = max(min(tau, max_torque), -max_torque);
    u = [thrust; tau(:)];
    
    %% True dynamics
    x_dot = drone_dynamics(current_time, x_true, u, params);
    x_true = x_true + x_dot * dt;
    x_true(7:9) = wrapToPi(x_true(7:9));
    
    if any(~isfinite(x_true))
        error('Simulation diverged at t=%.3f s', current_time);
    end
    
    %% Sensors and EKF
    sensors = sensor_model(x_true, params, current_time);
    imu_meas = [sensors.accel; sensors.gyro];
    
    % EKF prediction (IMU rate)
    if mod(k-1, round(params.Ts.IMU/dt)) == 0
        [x_est, P, ~, ~, ~] = ekf9_sensor_only_step(x_est, P, imu_meas, params, params.Ts.IMU);
        imu_hist(:,k) = imu_meas;
    end
    
    % GPS update
    if mod(k-1, round(params.Ts.GPS/dt)) == 0
        [x_est, P] = ekf_sensor_only(x_est, P, imu_meas, sensors.gps, params, 0, 'GPS');
        gps_hist(:,k) = sensors.gps;
    end
    
    % Barometer update
    if mod(k-1, round(params.Ts.Baro/dt)) == 0
        [x_est, P] = ekf_sensor_only(x_est, P, imu_meas, sensors.baro, params, 0, 'Baro');
        baro_hist(k) = sensors.baro;
    end
    
    % Magnetometer update
    if mod(k-1, round(params.Ts.Mag/dt)) == 0
        [x_est, P] = ekf_sensor_only(x_est, P, imu_meas, sensors.mag, params, 0, 'Mag');
        mag_hist(k) = sensors.mag;
    end
    
    %% Log data
    x_true_hist(:,k) = x_true;
    x_est_hist(:,k) = x_est;
    
    % Compute errors
    pos_error_hist(k) = norm(x_est(1:3) - x_true(1:3));
    vel_error_hist(k) = norm(x_est(4:6) - x_true(4:6));
    att_error_hist(k) = norm(x_est(7:9) - x_true(7:9));
    
    % Progress indicator
    if verbose && mod(k, round(5/dt)) == 1
        fprintf('t=%5.1fs  pos_err=%.2fm  vel_err=%.2fm/s  att_err=%.1fdeg\n', ...
            current_time, pos_error_hist(k), vel_error_hist(k), rad2deg(att_error_hist(k)));
    end
end

sim_time = toc;

%% Compute performance metrics
% Remove warmup period for statistics
analysis_start = round(warmup_T/dt);
if analysis_start < N
    pos_errors = pos_error_hist(analysis_start:end);
    vel_errors = vel_error_hist(analysis_start:end);
    att_errors = att_error_hist(analysis_start:end);
else
    pos_errors = pos_error_hist;
    vel_errors = vel_error_hist;
    att_errors = att_error_hist;
end

% Remove NaN values
pos_errors = pos_errors(~isnan(pos_errors));
vel_errors = vel_errors(~isnan(vel_errors));
att_errors = att_errors(~isnan(att_errors));

% Compute statistics
results = struct();
results.sim_time = sim_time;
results.duration = duration;
results.steps = N;

% Position accuracy
results.pos_rmse = sqrt(mean(pos_errors.^2));
results.pos_max_error = max(pos_errors);
results.pos_mean_error = mean(pos_errors);
results.pos_std_error = std(pos_errors);

% Velocity accuracy
results.vel_rmse = sqrt(mean(vel_errors.^2));
results.vel_max_error = max(vel_errors);
results.vel_mean_error = mean(vel_errors);
results.vel_std_error = std(vel_errors);

% Attitude accuracy (convert to degrees)
results.att_rmse_deg = rad2deg(sqrt(mean(att_errors.^2)));
results.att_max_error_deg = rad2deg(max(att_errors));
results.att_mean_error_deg = rad2deg(mean(att_errors));
results.att_std_error_deg = rad2deg(std(att_errors));

% Overall performance score (lower is better)
% Heavily weight attitude since it's critical and baseline was much better
% Add extra penalty if attitude error exceeds baseline (9 deg)
attitude_penalty = 0;
if results.att_rmse_deg > 9.0
    attitude_penalty = (results.att_rmse_deg - 9.0) * 2; % Heavy penalty for exceeding baseline
end

results.performance_score = results.pos_rmse + results.vel_rmse + results.att_rmse_deg/2 + attitude_penalty;
results.attitude_penalty = attitude_penalty;

% Parameters used
results.parameters = struct();
results.parameters.Q_scale = Q_scale;
results.parameters.R_gps_scale = R_gps_scale;
results.parameters.R_baro_scale = R_baro_scale;
results.parameters.R_mag_scale = R_mag_scale;
results.parameters.Q_att_scale = Q_att_scale;
results.parameters.init_pos = init_pos;
results.parameters.init_vel = init_vel;
results.parameters.init_att_deg = rad2deg(init_att);
results.parameters.P_scale = P_scale;

% Store full histories for detailed analysis
results.histories = struct();
results.histories.t = t;
results.histories.x_true = x_true_hist;
results.histories.x_est = x_est_hist;
results.histories.pos_error = pos_error_hist;
results.histories.vel_error = vel_error_hist;
results.histories.att_error = att_error_hist;
results.histories.imu = imu_hist;
results.histories.gps = gps_hist;
results.histories.baro = baro_hist;
results.histories.mag = mag_hist;

if verbose
    fprintf('\n=== PERFORMANCE RESULTS ===\n');
    fprintf('Simulation time: %.2f seconds\n', sim_time);
    fprintf('Position RMSE: %.3f m (max: %.3f m)\n', results.pos_rmse, results.pos_max_error);
    fprintf('Velocity RMSE: %.3f m/s (max: %.3f m/s)\n', results.vel_rmse, results.vel_max_error);
    fprintf('Attitude RMSE: %.2f deg (max: %.2f deg)', results.att_rmse_deg, results.att_max_error_deg);
    if results.attitude_penalty > 0
        fprintf(' [PENALTY: +%.1f for exceeding 9 deg baseline]', results.attitude_penalty);
    end
    fprintf('\nOverall Score: %.3f (lower is better)\n', results.performance_score);
end

end
