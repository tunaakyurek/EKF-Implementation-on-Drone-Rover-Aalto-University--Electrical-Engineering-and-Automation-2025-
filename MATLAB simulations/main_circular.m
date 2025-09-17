%% main_circular.m - Circular trajectory simulation with EKF-9 and analysis
% PURPOSE
% - Reuse the same EKF/sensors/dynamics as random-walk, but command a
%   sustained circular motion in the horizontal plane with gentle climbs.

clear; clc; close all;

%% PARAMETERS & SETUP
parameters; % loads 'params'
addpath(fullfile(fileparts(mfilename('fullpath')), 'noanim_benchmarks','filters'));

dt = params.Ts.physics;
T_end = params.sim_duration;
t = 0:dt:T_end; N = length(t);

% Initial conditions
pos0 = [0; 0; 0]; vel0 = [0; 0; 0]; att0 = [0; 0; 0];
x_true = [pos0; vel0; att0];
x_est  = x_true;
P = diag([0.5^2, 0.5^2, 0.4^2, 0.2^2, 0.2^2, 0.2^2, deg2rad(2)^2, deg2rad(2)^2, deg2rad(3)^2]);

% Logs
x_true_hist = zeros(9,N); x_est_hist = zeros(9,N);
imu_hist = zeros(6,N); gps_hist = NaN(3,N); baro_hist = NaN(1,N); mag_hist = NaN(1,N);
waypoints = zeros(3,0);

% Controller gains (gentle)
Kp_vel = [0.8; 0.8; 0.8];
Kp_att = [0.8; 0.8; 0.4];
Kd_att = [0.3; 0.3; 0.15];
prev_att_err = zeros(3,1);
max_tilt = deg2rad(15);

% Circular command parameters
radius = 10.0;           % meters
speed  = 3.0;            % m/s tangential
omega  = speed / max(radius,1e-3); % rad/s
vz_cmd = 0.0;            % constant climb rate (m/s)

fprintf('=== Circular EKF Simulation ===\n');

for k = 1:N
    tk = t(k);

    %% 1) Velocity command for a horizontal circle
    vel_cmd = [ -radius*omega*sin(omega*tk); radius*omega*cos(omega*tk); vz_cmd ];
    % Clip to practical limits
    vel_cmd = max(min(vel_cmd, [3;3;1.5]), -[3;3;1.5]);

    %% 2) Convert velocity command to thrust and desired attitude
    vel_est = x_est(4:6); att_est = x_est(7:9);
    vel_error = vel_cmd - vel_est;
    accel_des = Kp_vel .* vel_error;           % NED
    accel_des(3) = accel_des(3) + abs(params.g(3));

    a_norm = norm(accel_des);
    if a_norm < 1e-6
        accel_des = [0;0;abs(params.g(3))]; a_norm = norm(accel_des);
    end
    z_body_des = accel_des / a_norm;

    roll_des = asin(max(min(-z_body_des(2), sin(max_tilt)), -sin(max_tilt)));
    if abs(cos(roll_des)) > 1e-6
        pitch_des = asin(max(min(z_body_des(1)/cos(roll_des), sin(max_tilt)), -sin(max_tilt)));
    else
        pitch_des = 0;
    end

    % Yaw follows tangent of the circle
    yaw_des = atan2(vel_cmd(2), vel_cmd(1));
    att_ref = [roll_des; pitch_des; yaw_des];

    att_err = att_ref - att_est; att_err(3) = atan2(sin(att_err(3)), cos(att_err(3)));
    att_derr = (att_err - prev_att_err) / dt; prev_att_err = att_err;
    tau_cmd = Kp_att .* att_err + Kd_att .* att_derr;
    max_alpha = deg2rad(200); tau_cmd = max(min(tau_cmd, max_alpha), -max_alpha);
    tau = params.I * tau_cmd;

    thrust = params.mass * a_norm;
    max_thrust = 2.0 * params.mass * abs(params.g(3));
    min_thrust = 0.1 * params.mass * abs(params.g(3));
    thrust = max(min(thrust, max_thrust), min_thrust);
    max_torque = 0.08; tau = max(min(tau, max_torque), -max_torque);
    u = [thrust; tau(:)];

    %% 3) Truth dynamics
    x_dot = drone_dynamics(tk, x_true, u, params);
    x_true = x_true + x_dot * dt; x_true(7:9) = wrapToPi(x_true(7:9));

    %% 4) Sensors and EKF
    sensors = sensor_model(x_true, params, tk);
    imu_meas = [sensors.accel; sensors.gyro];

    if mod(k-1, round(params.Ts.IMU/dt)) == 0
        [x_est, P, F_ekf, x_pred, P_pred] = ekf9_sensor_only_step(x_est, P, imu_meas, params, params.Ts.IMU);
        imu_hist(:,k) = imu_meas; %#ok<*SAGROW>
    end
    if mod(k-1, round(params.Ts.GPS/dt)) == 0
        [x_est, P] = ekf_sensor_only(x_est, P, imu_meas, sensors.gps, params, 0, 'GPS');
        gps_hist(:,k) = sensors.gps;
    end
    if mod(k-1, round(params.Ts.Baro/dt)) == 0
        [x_est, P] = ekf_sensor_only(x_est, P, imu_meas, sensors.baro, params, 0, 'Baro');
        baro_hist(k) = sensors.baro;
    end
    if mod(k-1, round(params.Ts.Mag/dt)) == 0
        [x_est, P] = ekf_sensor_only(x_est, P, imu_meas, sensors.mag, params, 0, 'Mag');
        mag_hist(k) = sensors.mag;
    end

    %% 5) Log
    x_true_hist(:,k) = x_true; x_est_hist(:,k) = x_est;
end

%% Optional animation
animate_drone(t, x_true_hist, x_est_hist, gps_hist, waypoints);

%% Analysis windows
try
    analyze_sensor_fusion_performance(t, x_true_hist, x_est_hist, imu_hist, gps_hist, baro_hist, mag_hist);
    set(gcf, 'Name', 'EKF-9 (Raw Estimates) - Circular', 'NumberTitle', 'off');
catch, end

try
    x_est_offline = post_smooth_estimates(t, x_est_hist);
    analyze_sensor_fusion_performance(t, x_true_hist, x_est_offline, imu_hist, gps_hist, baro_hist, mag_hist);
    set(gcf, 'Name', 'EKF-9 + Offline Spike Suppression + Zero-Phase Smoothing - Circular', 'NumberTitle', 'off');
catch, end

fprintf('\n=== Circular Simulation Complete ===\n');


