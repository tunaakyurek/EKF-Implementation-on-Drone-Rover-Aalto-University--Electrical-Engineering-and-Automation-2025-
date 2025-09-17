%% main_random.m - Smooth 3D random-walk simulation with EKF and live animation
% PURPOSE
% Run a full simulation loop that:
% - Keeps EKF algorithms unchanged (sensor-only prediction and selective updates)
% - Drives the "true" drone with a smooth random-walk velocity command
% - Logs states/sensors and plays back a simple animation
% - Optionally computes basic performance plots
%
% MAJOR STEPS
% 1) Load parameters and initialize state/estimate/covariance
% 2) Generate smooth random-walk velocity commands (OU process)
% 3) Convert velocity commands to thrust and gentle attitude/torques
% 4) Integrate true dynamics, simulate sensors, and run EKF
% 5) Log data and periodically print status
% 6) Animate playback and run analysis

clear; clc; close all;

%% PARAMETERS & SETUP
parameters; % loads 'params'
addpath(fullfile(fileparts(mfilename('fullpath')), 'noanim_benchmarks','filters'));

dt = params.Ts.physics;    % physics step (e.g., 0.001 s)
T_end = params.sim_duration;
t = 0:dt:T_end;
N = length(t);

% State dimensions: [pos(3); vel(3); att(3)]
% Exact initial condition (user-tunable)
pos0 = [0; 0; 0];                  % NED (m)
vel0 = [0; 0; 0];                  % NED (m/s)
att0 = [0; 0; 0];                  % [roll pitch yaw] (rad)

x_true = [pos0; vel0; att0];
x_est  = x_true;                   % Start matched to truth for best convergence

% Initial covariance: tight since estimate equals truth
P = diag([0.5^2, 0.5^2, 0.4^2, 0.2^2, 0.2^2, 0.2^2, deg2rad(2)^2, deg2rad(2)^2, deg2rad(3)^2]);

% Histories
x_true_hist = zeros(9, N);
x_est_hist  = zeros(9, N);
x_est_rts_hist  = zeros(9, N);
imu_hist    = zeros(6, N);         % [accel(3); gyro(3)]
gps_hist    = NaN(3, N);
baro_hist   = NaN(1, N);
mag_hist    = NaN(1, N);

% Empty waypoints for plotting API compatibility
waypoints = zeros(3,0);

% Controller tuning (gentle, realistic)
Kp_vel = [0.6; 0.6; 0.8];    % velocity feedback gains
Kp_att = [0.8; 0.8; 0.4];    % attitude P gains  [roll; pitch; yaw]
Kd_att = [0.3; 0.3; 0.15];   % attitude D gains

prev_att_err = zeros(3,1);
max_tilt = deg2rad(10);      % align with model clamp for realism

% Random-walk (OU process) parameters for target velocity (NED)
lambda_v = 0.6;              % mean-reversion rate
sigma_v  = [0.8; 0.8; 0.5];  % noise intensity (m/s/sqrt(s))
v_max    = [3; 3; 1.5];      % max commanded velocity (|m/s|)
vel_cmd_target = zeros(3,1); % OU state

% Warm-up (hover) duration to let EKF settle before random-walk
warmup_T = 2.0;                         % seconds
warmup_steps = min(N, round(warmup_T/dt));

% Online fixed-lag RTS smoother buffer (operates at IMU event rate)
lag_seconds = 1.0;
lag_steps = max(1, round(lag_seconds/max(params.Ts.IMU, eps)));
smooth_buf = struct('x', zeros(9, lag_steps), 'P', zeros(9,9,lag_steps), ...
                    'x_pred', zeros(9, lag_steps), 'P_pred', zeros(9,9,lag_steps), ...
                    'F', zeros(9,9,lag_steps));
% Running smoothed state (delayed)
x_rts = x_est; P_rts = P;

fprintf('=== Random-Walk EKF Simulation ===\n');
fprintf('dt: %.1f ms, duration: %.1f s, steps: %d\n\n', dt*1000, T_end, N);

%% SIMULATION LOOP
for k = 1:N
    current_time = t(k);

    %% 1) Command generation (hover for warm-up, then OU random-walk)
    if k <= warmup_steps
        % Hover during warm-up
        vel_cmd = [0; 0; 0];
    else
        % Smooth random-walk target generation (Ornstein–Uhlenbeck)
        dW = sqrt(dt) * randn(3,1);
        vel_cmd_target = vel_cmd_target + (-lambda_v .* vel_cmd_target) * dt + sigma_v .* dW;
        % Clamp per-axis for realism
        vel_cmd = max(min(vel_cmd_target, v_max), -v_max);
    end

    %% 2) Convert velocity command to thrust and gentle attitude
    % Use estimator state for "pilot's" perception
    vel_est = x_est(4:6);
    att_est = x_est(7:9);

    % Desired acceleration (with velocity feedback)
    vel_error = vel_cmd - vel_est;
    accel_des = Kp_vel .* vel_error;            % NED frame
    accel_des(3) = accel_des(3) + abs(params.g(3)); % gravity compensation

    % Thrust and desired attitude from desired acceleration
    accel_norm = norm(accel_des);
    if accel_norm < 1e-6
        accel_des = [0; 0; abs(params.g(3))];
        accel_norm = norm(accel_des);
    end

    z_body_des = accel_des / accel_norm;

    % Compute desired roll/pitch from desired thrust direction
    roll_des = asin(max(min(-z_body_des(2), sin(max_tilt)), -sin(max_tilt)));
    if abs(cos(roll_des)) > 1e-6
        pitch_des = asin(max(min(z_body_des(1) / cos(roll_des), sin(max_tilt)), -sin(max_tilt)));
    else
        pitch_des = 0;
    end

    % Yaw: align with horizontal velocity command when moving, else hold
    if norm(vel_cmd(1:2)) > 0.3
        yaw_des = atan2(vel_cmd(2), vel_cmd(1));
    else
        yaw_des = att_est(3);
    end

    att_ref = [roll_des; pitch_des; yaw_des];

    % Attitude PD for torques (guard against estimator spikes)
    att_err = att_ref - att_est;
    att_err(3) = atan2(sin(att_err(3)), cos(att_err(3))); % wrap yaw
    att_derr = (att_err - prev_att_err) / dt;
    prev_att_err = att_err;

    tau_cmd = Kp_att .* att_err + Kd_att .* att_derr;   % desired angular accelerations (approx)
    % Rate-limit commanded angular acceleration
    max_alpha = deg2rad(200); % rad/s^2
    tau_cmd = max(min(tau_cmd, max_alpha), -max_alpha);
    tau = params.I * tau_cmd;                           % torque = I * alpha

    % Thrust magnitude
    thrust = params.mass * accel_norm;
    % Limits
    max_thrust = 2.0 * params.mass * abs(params.g(3));
    min_thrust = 0.1 * params.mass * abs(params.g(3));
    thrust = max(min(thrust, max_thrust), min_thrust);

    max_torque = 0.08; % Nm gentle limit for stability
    tau = max(min(tau, max_torque), -max_torque);

    u = [thrust; tau(:)];

    %% 3) True dynamics integration (baseline model)
    x_dot = drone_dynamics(current_time, x_true, u, params);
    x_true = x_true + x_dot * dt;
    x_true(7:9) = wrapToPi(x_true(7:9));

    if any(~isfinite(x_true)) || any(~isfinite(u))
        error('Simulation diverged at t=%.3f s', current_time);
    end

    %% STEP 4: SENSOR SIMULATION AND EKF ESTIMATION
    % Generate synthetic sensor measurements and run EKF estimation
    % This implements the sensor-only EKF approach (no control input usage)
    
    % Generate synthetic sensor measurements from true state
    sensors = sensor_model(x_true, params, current_time);
    imu_meas = [sensors.accel; sensors.gyro];  % Combine accel and gyro for EKF

    %% EKF PREDICTION STEP (IMU Rate - 250 Hz)
    % Run EKF prediction at IMU sampling rate for high-frequency state estimation
    if mod(k-1, round(params.Ts.IMU/dt)) == 0
        % Execute EKF prediction step: x̂(k|k-1) = f(x̂(k-1|k-1), IMU)
        [x_est, P, F_ekf, x_pred, P_pred] = ekf9_sensor_only_step(x_est, P, imu_meas, params, params.Ts.IMU);
        
        % Feed data to fixed-lag RTS smoother for improved estimates
        smooth_buf = push_filter_frame(smooth_buf, x_est, P, x_pred, P_pred, F_ekf);
        [x_rts, P_rts, smooth_buf] = rts_fixed_lag_step(smooth_buf);
        
        % Log IMU measurements for analysis
        imu_hist(:,k) = imu_meas;
    end

    %% EKF MEASUREMENT UPDATES (Multi-rate with Individual Predictions)
    % Each sensor update performs its own prediction to ensure proper timing
    % This maintains the original working behavior while being more explicit
    
    %% GPS UPDATE (10 Hz)
    if mod(k-1, round(params.Ts.GPS/dt)) == 0
        % GPS measurement update with prediction: x̂(k|k) = x̂(k|k-1) + K·(z_GPS - H·x̂(k|k-1))
        [x_est, P] = ekf_sensor_only(x_est, P, imu_meas, sensors.gps, params, params.Ts.GPS, 'GPS');
        gps_hist(:,k) = sensors.gps;  % Log GPS measurements
    end

    %% BAROMETER UPDATE (50 Hz)
    if mod(k-1, round(params.Ts.Baro/dt)) == 0
        % Barometer measurement update with prediction: altitude = -z_NED + noise
        [x_est, P] = ekf_sensor_only(x_est, P, imu_meas, sensors.baro, params, params.Ts.Baro, 'Baro');
        baro_hist(k) = sensors.baro;  % Log barometer measurements
    end

    %% MAGNETOMETER UPDATE (50 Hz)
    if mod(k-1, round(params.Ts.Mag/dt)) == 0
        % Magnetometer measurement update with prediction: yaw = ψ + noise (with angle wrapping)
        [x_est, P] = ekf_sensor_only(x_est, P, imu_meas, sensors.mag, params, params.Ts.Mag, 'Mag');
        mag_hist(k) = sensors.mag;  % Log magnetometer measurements
    end

    %% STEP 5: DATA LOGGING AND STORAGE
    % Store current states and estimates for post-processing analysis
    % This enables performance evaluation and visualization of EKF performance
    
    % Log true state (ground truth for comparison)
    x_true_hist(:,k) = x_true;
    
    % Log EKF state estimate (real-time estimates)
    x_est_hist(:,k)  = x_est;
    
    % Initialize RTS smoother output if needed
    if k == 1 && all(x_rts == 0)
        x_rts = x_est;
    end
    
    % Log RTS smoothed estimates (delayed but improved estimates)
    x_est_rts_hist(:,k) = x_rts;

    % Lightweight progress
    if mod(k, round(2/dt)) == 1
        pos = x_true(1:3);
        fprintf('t=%5.1fs  pos=[%6.1f %6.1f %6.1f]  |v_cmd|=%.2f m/s\n', current_time, pos(1), pos(2), pos(3), norm(vel_cmd));
    end
end

%% REAL-TIME ANIMATION (playback)
animate_drone(t, x_true_hist, x_est_hist, gps_hist, waypoints);

%% EKF ACCURACY & SENSOR FUSION ANALYSIS
try
    analyze_sensor_fusion_performance(t, x_true_hist, x_est_hist, imu_hist, gps_hist, baro_hist, mag_hist);
    set(gcf, 'Name', 'EKF-9 (Raw Estimates)', 'NumberTitle', 'off');
catch ME
    warning('Analysis plotting failed: %s', ME.message);
end

% Offline spike suppression + zero-phase smoothing
try
    x_est_offline = post_smooth_estimates(t, x_est_hist);
    analyze_sensor_fusion_performance(t, x_true_hist, x_est_offline, imu_hist, gps_hist, baro_hist, mag_hist);
    set(gcf, 'Name', 'EKF-9 + Offline Spike Suppression + Zero-Phase Smoothing', 'NumberTitle', 'off');
catch, end

fprintf('\n=== Random-Walk Simulation Complete ===\n');


