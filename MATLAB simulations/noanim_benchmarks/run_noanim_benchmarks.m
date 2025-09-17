%% run_noanim_benchmarks.m - Real-time, no-animation sensor fusion benchmarks
% PURPOSE
% - Run a single truth simulation and evaluate multiple Bayesian filters in parallel:
%   1) EKF-9 (existing sensor-only EKF)
%   2) EKF-9 + fixed-lag RTS smoother
%   3) EKF-15 with IMU bias states (accel + gyro)
%   4) UKF-9 with similar mechanization
%   5) ESKF-15 (error-state Kalman filter with biases)
% - No visualization/animation. Summary figures via analyze_sensor_fusion_performance.
%
% USAGE
%   From repo root or this folder, run:
%     run_noanim_benchmarks
%
% NOTES
% - Reuses parent `parameters.m`, `sensor_model.m`, `drone_dynamics.m`, `drone_dynamics_imu.m`.
% - Filter implementations live in `noanim_benchmarks/filters`.

clear; clc; close all;

% Add parent for dynamics and params, and this folder for filters
addpath('..');
addpath(fullfile(fileparts(mfilename('fullpath')), 'filters'));

%% PARAMETERS & SETUP
parameters; % loads 'params'

dt = params.Ts.physics;    % physics step
T_end = params.sim_duration;
t = 0:dt:T_end;
N = length(t);

% Fine-tune noises specifically for benchmark stability and accuracy
% - Slightly larger process noise to allow faster correction
% - Slightly lower GPS measurement noise reflecting high-quality module
params.Q = diag([0.05 0.05 0.05, 0.30 0.30 0.30, 0.08 0.08 0.10]);
params.GPS.sigma_xy = 0.5;                % m
params.GPS.sigma_z  = 1.0;                % m
params.R_gps = diag([params.GPS.sigma_xy^2, params.GPS.sigma_xy^2, params.GPS.sigma_z^2]);
% Keep baro/mag as is

% Tilt-update gating and logging config (tuned)
tilt_accel_tol = 5.0;              % |a|-|g| tolerance (m/s^2)
tilt_rate_gate = deg2rad(100);     % rate gate (rad/s)
tilt_R_deg = 3.0;                  % nominal tilt measurement noise (deg)

% Apply recommendations to params for use inside filters
params.Tilt = struct('accel_tol', tilt_accel_tol, 'rate_gate', tilt_rate_gate, 'R_deg', tilt_R_deg);

% Optional scaling based on recommendations
R_mag_scale = 1.00;   % from recommendations
Q_att_scale = 1.50;   % scale attitude process noise
params.R_mag = params.R_mag * R_mag_scale;
params.Q(7:9,7:9) = params.Q(7:9,7:9) * Q_att_scale;

% Truth state (9): [pos; vel; att]
pos0 = [0; 0; 0];
vel0 = [0; 0; 0];
att0 = [0; 0; 0];
x_true = [pos0; vel0; att0];

% EKF-9 (reuse existing)
x9 = x_true; P9 = diag([0.5^2, 0.5^2, 0.4^2, 0.2^2, 0.2^2, 0.2^2, deg2rad(2)^2, deg2rad(2)^2, deg2rad(3)^2]);

% EKF-9 + Fixed-Lag smoother buffer
lag_seconds = 1.0;  % fixed smoothing lag
lag_steps = max(1, round(lag_seconds/params.Ts.IMU));
smooth_buf = struct('x', zeros(9, lag_steps), 'P', zeros(9,9,lag_steps), ...
                    'x_pred', zeros(9, lag_steps), 'P_pred', zeros(9,9,lag_steps), ...
                    'F', zeros(9,9,lag_steps));
% smoothed output mirrors EKF-9 but delayed
x9s = x9; P9s = P9;

% EKF-15 (add accel/gyro biases)
x15 = [x_true; zeros(6,1)];
P15 = blkdiag(P9, diag([0.02 0.02 0.02 0.0005 0.0005 0.0005]));

% UKF-9
xu9 = x_true; Pu9 = P9;

% ESKF-15 (nominal + error covariance)
xn = [x_true; zeros(6,1)];  % nominal with biases
Pe = blkdiag(P9, 1e-2*eye(6));

% Logs (truth and each filter)
x_true_hist = zeros(9, N);
x9_hist  = zeros(9, N);
x9s_hist = zeros(9, N);
x15_hist = zeros(15, N);
xu9_hist = zeros(9, N);
xn_hist  = zeros(15, N);

imu_hist  = zeros(6, N);
gps_hist  = NaN(3, N);
baro_hist = NaN(1, N);
mag_hist  = NaN(1, N);

% Tuning/logging arrays (per-step)
accel_norm_log   = NaN(1,N);
gyro_rate_log    = NaN(1,N);
abs_a_minus_g_log = NaN(1,N);
tilt_used_ekf9   = false(1,N);
tilt_used_ukf9   = false(1,N);
gps_innov_norm_log = NaN(1,N);
baro_innov_log     = NaN(1,N);
mag_innov_log      = NaN(1,N);
pos_err_norm_log   = NaN(1,N);   % EKF-9 position error norm
roll_err_deg_log   = NaN(1,N);   % EKF-9
pitch_err_deg_log  = NaN(1,N);
yaw_err_deg_log    = NaN(1,N);

% Random-walk velocity command (Ornstein-Uhlenbeck)
lambda_v = 0.6;
sigma_v  = [0.8; 0.8; 0.5];
v_max    = [3; 3; 1.5];
vel_cmd_target = zeros(3,1);

Kp_vel = [0.6; 0.6; 0.8];
Kp_att = [0.8; 0.8; 0.4];
Kd_att = [0.3; 0.3; 0.15];
prev_att_err = zeros(3,1);
max_tilt = deg2rad(10);

warmup_T = 2.0;
warmup_steps = min(N, round(warmup_T/dt));

fprintf('=== No-Animation Sensor Fusion Benchmarks ===\n');
fprintf('dt: %.1f ms, duration: %.1f s, steps: %d\n\n', dt*1000, T_end, N);

%% SIM LOOP
for k = 1:N
    current_time = t(k);

    % 1) Command generation
    if k <= warmup_steps
        vel_cmd = [0;0;0];
    else
        dW = sqrt(dt) * randn(3,1);
        vel_cmd_target = vel_cmd_target + (-lambda_v .* vel_cmd_target) * dt + sigma_v .* dW;
        vel_cmd = max(min(vel_cmd_target, v_max), -v_max);
    end

    % 2) Convert velocity command to thrust and attitude
    vel_est = x9(4:6);   % use EKF-9 estimate for controller reference
    att_est = x9(7:9);
    vel_error = vel_cmd - vel_est;
    accel_des = Kp_vel .* vel_error;            % NED
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

    % 3) Truth propagation
    x_dot = drone_dynamics(current_time, x_true, u, params);
    x_true = x_true + x_dot * dt; x_true(7:9) = wrapToPi(x_true(7:9));
    if any(~isfinite(x_true)), error('Diverged'); end

    % 4) Sensors
    sensors = sensor_model(x_true, params, current_time);
    imu_meas = [sensors.accel; sensors.gyro];
    % Log gates
    a_norm = norm(imu_meas(1:3));
    g_norm = norm(params.g);
    accel_norm_log(k) = a_norm;
    gyro_rate_log(k) = norm(imu_meas(4:6));
    abs_a_minus_g_log(k) = abs(a_norm - g_norm);

    %% 5) Filter steps (respect sensor rates)
    % All filters predict on IMU event; updates per sensor types
    imu_event  = mod(k-1, round(params.Ts.IMU/dt))  == 0;
    gps_event  = mod(k-1, round(params.Ts.GPS/dt))  == 0;
    baro_event = mod(k-1, round(params.Ts.Baro/dt)) == 0;
    mag_event  = mod(k-1, round(params.Ts.Mag/dt))  == 0;

    if imu_event
        % EKF-9
        [x9, P9, F_ekf9, x9_pred, P9_pred] = ekf9_sensor_only_step(x9, P9, imu_meas, params, params.Ts.IMU);
        % Smoother buffer push
        smooth_buf = push_filter_frame(smooth_buf, x9, P9, x9_pred, P9_pred, F_ekf9);

        % Optional: accelerometer-based tilt update with gating
        if abs_a_minus_g_log(k) <= tilt_accel_tol && gyro_rate_log(k) <= tilt_rate_gate
            [x9, P9] = ekf_sensor_only(x9, P9, imu_meas, [], params, 0, 'AccelTilt');
            tilt_used_ekf9(k) = true;
        end

        % EKF-15
        [x15, P15] = ekf15_bias_step(x15, P15, imu_meas, [], params, params.Ts.IMU, 'IMU');

        % UKF-9
        [xu9, Pu9] = ukf9_step(xu9, Pu9, imu_meas, [], params, params.Ts.IMU, 'IMU');
        if abs_a_minus_g_log(k) <= tilt_accel_tol && gyro_rate_log(k) <= tilt_rate_gate
            [xu9, Pu9] = ukf9_step(xu9, Pu9, imu_meas, [], params, 0, 'AccelTilt');
            tilt_used_ukf9(k) = true;
        end

        % ESKF-15
        [xn, Pe] = eskf15_step(xn, Pe, imu_meas, [], params, params.Ts.IMU, 'IMU');

        imu_hist(:,k) = imu_meas;
    end

    if gps_event
        z = sensors.gps;
        % IMPORTANT: avoid double-predict at sensor events → use dt = 0 for pure update
        [x9, P9]   = ekf_sensor_only(x9, P9, imu_meas, z, params, 0, 'GPS');
        [x15, P15] = ekf15_bias_step(x15, P15, imu_meas, z, params, 0, 'GPS');
        [xu9, Pu9] = ukf9_step(xu9, Pu9, imu_meas, z, params, 0, 'GPS');
        [xn, Pe]   = eskf15_step(xn, Pe, imu_meas, z, params, 0, 'GPS');
        gps_hist(:,k) = z;
        % Innovations (approx) and position error
        gps_innov_norm_log(k) = norm(z - x_true(1:3));
    end

    if baro_event
        z = sensors.baro;
        [x9, P9]   = ekf_sensor_only(x9, P9, imu_meas, z, params, 0, 'Baro');
        [x15, P15] = ekf15_bias_step(x15, P15, imu_meas, z, params, 0, 'Baro');
        [xu9, Pu9] = ukf9_step(xu9, Pu9, imu_meas, z, params, 0, 'Baro');
        [xn, Pe]   = eskf15_step(xn, Pe, imu_meas, z, params, 0, 'Baro');
        baro_hist(k) = z;
        baro_innov_log(k) = z - (-x_true(3));
    end

    if mag_event
        z = sensors.mag;
        [x9, P9]   = ekf_sensor_only(x9, P9, imu_meas, z, params, 0, 'Mag');
        [x15, P15] = ekf15_bias_step(x15, P15, imu_meas, z, params, 0, 'Mag');
        [xu9, Pu9] = ukf9_step(xu9, Pu9, imu_meas, z, params, 0, 'Mag');
        [xn, Pe]   = eskf15_step(xn, Pe, imu_meas, z, params, 0, 'Mag');
        mag_hist(k) = z;
        yaw_err_deg_log(k) = rad2deg(atan2(sin(z - x_true(9)), cos(z - x_true(9))));
    end

    % Fixed-lag smoother output (delayed copy of EKF-9)
    if imu_event
        [x9s, P9s, smooth_buf] = rts_fixed_lag_step(smooth_buf);
    end

    % Logs
    x_true_hist(:,k) = x_true;
    x9_hist(:,k)  = x9;
    x9s_hist(:,k) = x9s;
    x15_hist(:,k) = x15;
    xu9_hist(:,k) = xu9;
    xn_hist(:,k)  = xn;

    % Error logs for EKF-9
    pos_err_norm_log(k) = norm(x9(1:3) - x_true(1:3));
    roll_err_deg_log(k)  = rad2deg(x9(7) - x_true(7));
    pitch_err_deg_log(k) = rad2deg(x9(8) - x_true(8));

    if mod(k, round(2/dt)) == 1
        pos = x_true(1:3);
        fprintf('t=%5.1fs  pos=[%6.1f %6.1f %6.1f]  |v_cmd|=%.2f m/s\n', current_time, pos(1), pos(2), pos(3), norm(vel_cmd));
    end
end

fprintf('\n=== Benchmark Complete ===\n');

%% ANALYSIS per filter (reuse existing analysis; show separate figures)
try
    figs_before = findall(0,'Type','figure');
    analyze_sensor_fusion_performance(t, x_true_hist, x9_hist, imu_hist, gps_hist, baro_hist, mag_hist);
    figs_after = findall(0,'Type','figure');
    new_figs = setdiff(figs_after, figs_before);
    label = 'EKF-9';
    if isempty(new_figs)
        f1 = gcf; set(f1,'Name',label,'NumberTitle','off');
        set_fig_title(f1, label);
        set_fig_square_center(f1, 900, 900);
    else
        for iF = 1:numel(new_figs)
            f1 = new_figs(iF); set(f1,'Name',label,'NumberTitle','off');
            set_fig_title(f1, label);
            set_fig_square_center(f1, 900, 900);
        end
    end
catch, end

try
    figs_before = findall(0,'Type','figure');
    analyze_sensor_fusion_performance(t, x_true_hist, x9s_hist, imu_hist, gps_hist, baro_hist, mag_hist);
    figs_after = findall(0,'Type','figure');
    new_figs = setdiff(figs_after, figs_before);
    label = 'EKF-9 + Fixed-Lag RTS';
    if isempty(new_figs)
        f2 = gcf; set(f2,'Name',label,'NumberTitle','off');
        set_fig_title(f2, label);
        set_fig_square_center(f2, 900, 900);
    else
        for iF = 1:numel(new_figs)
            f2 = new_figs(iF); set(f2,'Name',label,'NumberTitle','off');
            set_fig_title(f2, label);
            set_fig_square_center(f2, 900, 900);
        end
    end
catch, end

try
    figs_before = findall(0,'Type','figure');
    analyze_sensor_fusion_performance(t, x_true_hist, x15_hist(1:9,:), imu_hist, gps_hist, baro_hist, mag_hist);
    figs_after = findall(0,'Type','figure');
    new_figs = setdiff(figs_after, figs_before);
    label = 'EKF-15 (with biases)';
    if isempty(new_figs)
        f3 = gcf; set(f3,'Name',label,'NumberTitle','off');
        set_fig_title(f3, label);
        set_fig_square_center(f3, 900, 900);
    else
        for iF = 1:numel(new_figs)
            f3 = new_figs(iF); set(f3,'Name',label,'NumberTitle','off');
            set_fig_title(f3, label);
            set_fig_square_center(f3, 900, 900);
        end
    end
catch, end

try
    figs_before = findall(0,'Type','figure');
    analyze_sensor_fusion_performance(t, x_true_hist, xu9_hist, imu_hist, gps_hist, baro_hist, mag_hist);
    figs_after = findall(0,'Type','figure');
    new_figs = setdiff(figs_after, figs_before);
    label = 'UKF-9';
    if isempty(new_figs)
        f4 = gcf; set(f4,'Name',label,'NumberTitle','off');
        set_fig_title(f4, label);
        set_fig_square_center(f4, 900, 900);
    else
        for iF = 1:numel(new_figs)
            f4 = new_figs(iF); set(f4,'Name',label,'NumberTitle','off');
            set_fig_title(f4, label);
            set_fig_square_center(f4, 900, 900);
        end
    end
catch, end

try
    figs_before = findall(0,'Type','figure');
    analyze_sensor_fusion_performance(t, x_true_hist, xn_hist(1:9,:), imu_hist, gps_hist, baro_hist, mag_hist);
    figs_after = findall(0,'Type','figure');
    new_figs = setdiff(figs_after, figs_before);
    label = 'ESKF-15';
    if isempty(new_figs)
        f5 = gcf; set(f5,'Name',label,'NumberTitle','off');
        set_fig_title(f5, label);
        set_fig_square_center(f5, 900, 900);
    else
        for iF = 1:numel(new_figs)
            f5 = new_figs(iF); set(f5,'Name',label,'NumberTitle','off');
            set_fig_title(f5, label);
            set_fig_square_center(f5, 900, 900);
        end
    end
catch, end

%% Write tuning log CSV
try
    log_tbl = table(t(:), accel_norm_log(:), gyro_rate_log(:), abs_a_minus_g_log(:), ...
        tilt_used_ekf9(:), tilt_used_ukf9(:), gps_innov_norm_log(:), baro_innov_log(:), mag_innov_log(:), ...
        pos_err_norm_log(:), roll_err_deg_log(:), pitch_err_deg_log(:), yaw_err_deg_log(:), ...
        'VariableNames', {'time_s','accel_norm','gyro_rate','abs_a_minus_g','tilt_used_ekf9','tilt_used_ukf9', ...
        'gps_innov_norm','baro_innov','mag_innov','pos_err_norm','roll_err_deg','pitch_err_deg','yaw_err_deg'});
    log_path = fullfile(fileparts(mfilename('fullpath')), 'tuning_log.csv');
    writetable(log_tbl, log_path);
    fprintf('Wrote tuning log: %s\n', log_path);
    % Also write a small JSON of the gates used for traceability
    gates = struct('tilt_accel_tol', tilt_accel_tol, 'tilt_rate_gate_deg_s', rad2deg(tilt_rate_gate), 'tilt_R_deg', tilt_R_deg);
    json_path = fullfile(fileparts(mfilename('fullpath')), 'tuning_gates.json');
    fid = fopen(json_path, 'w'); if fid>0, fprintf(fid, '{"tilt_accel_tol": %.6f, "tilt_rate_gate_deg_s": %.6f, "tilt_R_deg": %.6f}', gates.tilt_accel_tol, gates.tilt_rate_gate_deg_s, gates.tilt_R_deg); fclose(fid); end
    fprintf('Wrote gates: %s\n', json_path);
catch ME
    fprintf('Failed to write tuning logs: %s\n', ME.message);
end

%% Helper: set figure to square size and center on primary monitor
function set_fig_square_center(f, w, h)
    if isempty(f) || ~ishandle(f), return; end
    try
        set(f,'Units','pixels');
        mp = get(0,'MonitorPositions');
        if isempty(mp)
            ss = get(0,'ScreenSize'); sw = ss(3); sh = ss(4);
        else
            sw = mp(1,3); sh = mp(1,4);
        end
        x = max(1, round((sw - w)/2));
        y = max(1, round((sh - h)/2));
        set(f, 'Position', [x y w h]);
    catch
        % Fallback: just set size
        try, set(f,'Position',[100 100 w h]); end
    end
end

function set_fig_title(f, label)
    if isempty(f) || ~ishandle(f), return; end
    try
        % Prefer sgtitle on given figure handle
        try
            sgtitle(f, label);
        catch
            % Older MATLAB: make f current and call sgtitle(text)
            figure(f); sgtitle(label);
        end
    catch
        % Fallback: annotation-based supertitle
        try
            ann = annotation(f,'textbox',[0 0.96 1 0.04], 'String', label, ...
                'EdgeColor','none','HorizontalAlignment','center', ...
                'FontWeight','bold','Interpreter','none'); %#ok<NASGU>
        catch
            % Last resort: title on the first axes
            ax = findall(f,'Type','axes'); if ~isempty(ax), title(ax(1), label); end
        end
    end
end

%% Helper: push a frame into smoother buffer
function buf = push_filter_frame(buf, x, P, x_pred, P_pred, F)
    buf.x      = circshift(buf.x,      [0, 1]); buf.x(:,1) = x;
    buf.P      = circshift(buf.P,      [0, 0, 1]); buf.P(:,:,1) = P;
    buf.x_pred = circshift(buf.x_pred, [0, 1]); buf.x_pred(:,1) = x_pred;
    buf.P_pred = circshift(buf.P_pred, [0, 0, 1]); buf.P_pred(:,:,1) = P_pred;
    buf.F      = circshift(buf.F,      [0, 0, 1]); buf.F(:,:,1) = F;
end
