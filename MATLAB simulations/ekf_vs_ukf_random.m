%% ekf_vs_ukf_random.m - EKF vs UKF Comparison with Random-Walk Simulation
% PURPOSE
% Run the same simulation as main_random.m but compare EKF vs UKF performance:
% - Uses identical setup as main_random.m (proven working simulation)
% - Runs both EKF and UKF in parallel on the same sensor data
% - Generates comprehensive comparison plots and metrics
% - Shows which filter performs better in realistic conditions
%
% MAJOR STEPS
% 1) Load parameters and initialize both filters identically
% 2) Generate smooth random-walk velocity commands (same as main_random.m)
% 3) Integrate true dynamics and simulate sensors
% 4) Run both EKF and UKF on the same sensor data
% 5) Log both filter results and generate comparison analysis

clear; clc; close all;

%% PARAMETERS & SETUP
parameters; % loads 'params'
addpath(fullfile(fileparts(mfilename('fullpath')), 'noanim_benchmarks','filters'));

dt = params.Ts.physics;    % physics step (e.g., 0.001 s)
T_end = params.sim_duration;
t = 0:dt:T_end;
N = length(t);

fprintf('=== EKF vs UKF Comparison Simulation ===\n');
fprintf('dt: %.1f ms, duration: %.1f s, steps: %d\n\n', dt*1000, T_end, N);

% State dimensions: [pos(3); vel(3); att(3)]
% Exact initial condition (user-tunable)
pos0 = [0; 0; 0];                  % NED (m)
vel0 = [0; 0; 0];                  % NED (m/s)
att0 = [0; 0; 0];                  % [roll pitch yaw] (rad)

x_true = [pos0; vel0; att0];

% Initialize both filters identically
x_est_ekf = x_true;               % EKF estimate
x_est_ukf = x_true;               % UKF estimate

% Initial covariance: tight since estimate equals truth
P_ekf = diag([0.5^2, 0.5^2, 0.4^2, 0.2^2, 0.2^2, 0.2^2, deg2rad(2)^2, deg2rad(2)^2, deg2rad(3)^2]);
P_ukf = P_ekf;  % Same initial uncertainty

% Histories for both filters
x_true_hist = zeros(9, N);
x_est_ekf_hist = zeros(9, N);
x_est_ukf_hist = zeros(9, N);
imu_hist = zeros(6, N);         % [accel(3); gyro(3)]
gps_hist = NaN(3, N);
baro_hist = NaN(1, N);
mag_hist = NaN(1, N);

% Performance tracking
ekf_computation_times = zeros(N, 1);
ukf_computation_times = zeros(N, 1);
ekf_innovations = cell(N, 1);
ukf_innovations = cell(N, 1);

% Empty waypoints for plotting API compatibility
waypoints = zeros(3,0);

% Controller tuning (gentle, realistic) - same as main_random.m
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

% Warm-up (hover) duration to let filters settle before random-walk
warmup_T = 2.0;                         % seconds
warmup_steps = min(N, round(warmup_T/dt));

fprintf('Initialization complete. Starting simulation...\n\n');

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
    % Use EKF estimate for control (as would be done in practice)
    vel_est = x_est_ekf(4:6);
    att_est = x_est_ekf(7:9);

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

    %% 4) Sensors (same data for both filters)
    sensors = sensor_model(x_true, params, current_time);
    imu_meas = [sensors.accel; sensors.gyro];

    %% 5) EKF Processing
    ekf_start_time = tic;
    
    % EKF predict at IMU rate
    if mod(k-1, round(params.Ts.IMU/dt)) == 0
        try
            [x_est_ekf, P_ekf, F_ekf, x_pred_ekf, P_pred_ekf] = ekf9_sensor_only_step(x_est_ekf, P_ekf, imu_meas, params, params.Ts.IMU);
            imu_hist(:,k) = imu_meas;
        catch ME
            fprintf('EKF IMU step failed at t=%.3f: %s\n', current_time, ME.message);
        end
    end

    % EKF GPS update
    if mod(k-1, round(params.Ts.GPS/dt)) == 0
        try
            [x_est_ekf, P_ekf] = ekf_sensor_only(x_est_ekf, P_ekf, imu_meas, sensors.gps, params, 0, 'GPS');
            gps_hist(:,k) = sensors.gps;
        catch ME
            fprintf('EKF GPS update failed at t=%.3f: %s\n', current_time, ME.message);
        end
    end

    % EKF Barometer update
    if mod(k-1, round(params.Ts.Baro/dt)) == 0
        try
            [x_est_ekf, P_ekf] = ekf_sensor_only(x_est_ekf, P_ekf, imu_meas, sensors.baro, params, 0, 'Baro');
            baro_hist(k) = sensors.baro;
        catch ME
            fprintf('EKF Baro update failed at t=%.3f: %s\n', current_time, ME.message);
        end
    end

    % EKF Magnetometer update
    if mod(k-1, round(params.Ts.Mag/dt)) == 0
        try
            [x_est_ekf, P_ekf] = ekf_sensor_only(x_est_ekf, P_ekf, imu_meas, sensors.mag, params, 0, 'Mag');
            mag_hist(k) = sensors.mag;
        catch ME
            fprintf('EKF Mag update failed at t=%.3f: %s\n', current_time, ME.message);
        end
    end
    
    ekf_computation_times(k) = toc(ekf_start_time);

    %% 6) UKF Processing (same sensor data)
    ukf_start_time = tic;
    
    % UKF predict at IMU rate
    if mod(k-1, round(params.Ts.IMU/dt)) == 0
        try
            [x_est_ukf, P_ukf] = ukf9_step(x_est_ukf, P_ukf, imu_meas, 'Predict', params.Ts.IMU, params);
        catch ME
            fprintf('UKF IMU step failed at t=%.3f: %s\n', current_time, ME.message);
        end
    end

    % UKF GPS update
    if mod(k-1, round(params.Ts.GPS/dt)) == 0
        try
            [x_est_ukf, P_ukf] = ukf9_step(x_est_ukf, P_ukf, sensors.gps, 'GPS', 0, params);
        catch ME
            fprintf('UKF GPS update failed at t=%.3f: %s\n', current_time, ME.message);
        end
    end

    % UKF Barometer update
    if mod(k-1, round(params.Ts.Baro/dt)) == 0
        try
            [x_est_ukf, P_ukf] = ukf9_step(x_est_ukf, P_ukf, sensors.baro, 'Baro', 0, params);
        catch ME
            fprintf('UKF Baro update failed at t=%.3f: %s\n', current_time, ME.message);
        end
    end

    % UKF Magnetometer update
    if mod(k-1, round(params.Ts.Mag/dt)) == 0
        try
            [x_est_ukf, P_ukf] = ukf9_step(x_est_ukf, P_ukf, sensors.mag, 'Mag', 0, params);
        catch ME
            fprintf('UKF Mag update failed at t=%.3f: %s\n', current_time, ME.message);
        end
    end
    
    ukf_computation_times(k) = toc(ukf_start_time);

    %% 7) Log both filter results
    x_true_hist(:,k) = x_true;
    x_est_ekf_hist(:,k) = x_est_ekf;
    x_est_ukf_hist(:,k) = x_est_ukf;

    % Lightweight progress
    if mod(k, round(2/dt)) == 1
        pos = x_true(1:3);
        fprintf('t=%5.1fs  pos=[%6.1f %6.1f %6.1f]  |v_cmd|=%.2f m/s\n', current_time, pos(1), pos(2), pos(3), norm(vel_cmd));
    end
end

fprintf('\nSimulation complete. Analyzing results...\n\n');

%% PERFORMANCE ANALYSIS
% Calculate errors
ekf_pos_errors = sqrt(sum((x_true_hist(1:3,:) - x_est_ekf_hist(1:3,:)).^2, 1));
ukf_pos_errors = sqrt(sum((x_true_hist(1:3,:) - x_est_ukf_hist(1:3,:)).^2, 1));

ekf_vel_errors = sqrt(sum((x_true_hist(4:6,:) - x_est_ekf_hist(4:6,:)).^2, 1));
ukf_vel_errors = sqrt(sum((x_true_hist(4:6,:) - x_est_ukf_hist(4:6,:)).^2, 1));

ekf_att_errors = abs(atan2(sin(x_true_hist(7:9,:) - x_est_ekf_hist(7:9,:)), cos(x_true_hist(7:9,:) - x_est_ekf_hist(7:9,:))));
ukf_att_errors = abs(atan2(sin(x_true_hist(7:9,:) - x_est_ukf_hist(7:9,:)), cos(x_true_hist(7:9,:) - x_est_ukf_hist(7:9,:))));

% Performance metrics
ekf_pos_rmse = sqrt(mean(ekf_pos_errors.^2));
ukf_pos_rmse = sqrt(mean(ukf_pos_errors.^2));

ekf_vel_rmse = sqrt(mean(ekf_vel_errors.^2));
ukf_vel_rmse = sqrt(mean(ukf_vel_errors.^2));

ekf_att_rmse = sqrt(mean(ekf_att_errors(:).^2));
ukf_att_rmse = sqrt(mean(ukf_att_errors(:).^2));

ekf_total_time = sum(ekf_computation_times);
ukf_total_time = sum(ukf_computation_times);

% Improvements
pos_improvement = (ekf_pos_rmse - ukf_pos_rmse) / ekf_pos_rmse * 100;
vel_improvement = (ekf_vel_rmse - ukf_vel_rmse) / ekf_vel_rmse * 100;
att_improvement = (ekf_att_rmse - ukf_att_rmse) / ekf_att_rmse * 100;
time_overhead = (ukf_total_time - ekf_total_time) / ekf_total_time * 100;

%% RESULTS SUMMARY
fprintf('=== EKF vs UKF PERFORMANCE COMPARISON ===\n\n');
fprintf('Metric                | EKF      | UKF      | UKF Improvement\n');
fprintf('----------------------|----------|----------|----------------\n');
fprintf('Position RMSE (m)     | %8.3f | %8.3f | %8.1f%%\n', ekf_pos_rmse, ukf_pos_rmse, pos_improvement);
fprintf('Velocity RMSE (m/s)   | %8.3f | %8.3f | %8.1f%%\n', ekf_vel_rmse, ukf_vel_rmse, vel_improvement);
fprintf('Attitude RMSE (deg)   | %8.3f | %8.3f | %8.1f%%\n', rad2deg(ekf_att_rmse), rad2deg(ukf_att_rmse), att_improvement);
fprintf('Computation Time (s)  | %8.3f | %8.3f | %8.1f%% overhead\n', ekf_total_time, ukf_total_time, time_overhead);

fprintf('\n');
if pos_improvement > 0 && vel_improvement > 0
    fprintf('🎉 UKF shows BETTER performance than EKF!\n');
elseif pos_improvement > -5 && vel_improvement > -5
    fprintf('📊 UKF shows COMPARABLE performance to EKF\n');
else
    fprintf('📈 EKF shows better performance than UKF\n');
end

if time_overhead < 50
    fprintf('⚡ UKF computational overhead is acceptable (%.1f%%)\n', time_overhead);
else
    fprintf('⚠️  UKF has significant computational overhead (%.1f%%)\n', time_overhead);
end

%% COMPREHENSIVE COMPARISON PLOTS
fprintf('\nGenerating comparison plots...\n');

% Main comparison figure
figure('Name', 'EKF vs UKF Comprehensive Comparison', 'Position', [50, 50, 1600, 1000]);

% Position errors over time
subplot(3, 4, 1);
plot(t, ekf_pos_errors, 'b-', 'LineWidth', 1.5, 'DisplayName', 'EKF');
hold on;
plot(t, ukf_pos_errors, 'r--', 'LineWidth', 1.5, 'DisplayName', 'UKF');
xlabel('Time (s)');
ylabel('Position Error (m)');
title('Position Estimation Error');
legend('Location', 'best');
grid on;

% Velocity errors over time
subplot(3, 4, 2);
plot(t, ekf_vel_errors, 'b-', 'LineWidth', 1.5, 'DisplayName', 'EKF');
hold on;
plot(t, ukf_vel_errors, 'r--', 'LineWidth', 1.5, 'DisplayName', 'UKF');
xlabel('Time (s)');
ylabel('Velocity Error (m/s)');
title('Velocity Estimation Error');
legend('Location', 'best');
grid on;

% Attitude errors (yaw only for clarity)
subplot(3, 4, 3);
plot(t, rad2deg(ekf_att_errors(3,:)), 'b-', 'LineWidth', 1.5, 'DisplayName', 'EKF');
hold on;
plot(t, rad2deg(ukf_att_errors(3,:)), 'r--', 'LineWidth', 1.5, 'DisplayName', 'UKF');
xlabel('Time (s)');
ylabel('Yaw Error (deg)');
title('Yaw Estimation Error');
legend('Location', 'best');
grid on;

% 3D trajectory comparison
subplot(3, 4, 4);
plot3(x_true_hist(1,:), x_true_hist(2,:), x_true_hist(3,:), 'k-', 'LineWidth', 2, 'DisplayName', 'True');
hold on;
plot3(x_est_ekf_hist(1,:), x_est_ekf_hist(2,:), x_est_ekf_hist(3,:), 'b--', 'LineWidth', 1.5, 'DisplayName', 'EKF');
plot3(x_est_ukf_hist(1,:), x_est_ukf_hist(2,:), x_est_ukf_hist(3,:), 'r:', 'LineWidth', 1.5, 'DisplayName', 'UKF');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Trajectory Comparison');
legend('Location', 'best');
grid on;
axis equal;

% X position comparison
subplot(3, 4, 5);
plot(t, x_true_hist(1,:), 'k-', 'LineWidth', 2, 'DisplayName', 'True');
hold on;
plot(t, x_est_ekf_hist(1,:), 'b--', 'LineWidth', 1.5, 'DisplayName', 'EKF');
plot(t, x_est_ukf_hist(1,:), 'r:', 'LineWidth', 1.5, 'DisplayName', 'UKF');
xlabel('Time (s)'); ylabel('X Position (m)');
title('X Position Estimates');
legend('Location', 'best');
grid on;

% Y position comparison
subplot(3, 4, 6);
plot(t, x_true_hist(2,:), 'k-', 'LineWidth', 2, 'DisplayName', 'True');
hold on;
plot(t, x_est_ekf_hist(2,:), 'b--', 'LineWidth', 1.5, 'DisplayName', 'EKF');
plot(t, x_est_ukf_hist(2,:), 'r:', 'LineWidth', 1.5, 'DisplayName', 'UKF');
xlabel('Time (s)'); ylabel('Y Position (m)');
title('Y Position Estimates');
legend('Location', 'best');
grid on;

% Z position comparison
subplot(3, 4, 7);
plot(t, x_true_hist(3,:), 'k-', 'LineWidth', 2, 'DisplayName', 'True');
hold on;
plot(t, x_est_ekf_hist(3,:), 'b--', 'LineWidth', 1.5, 'DisplayName', 'EKF');
plot(t, x_est_ukf_hist(3,:), 'r:', 'LineWidth', 1.5, 'DisplayName', 'UKF');
xlabel('Time (s)'); ylabel('Z Position (m)');
title('Z Position Estimates');
legend('Location', 'best');
grid on;

% Performance metrics bar chart
subplot(3, 4, 8);
metrics = {'Pos RMSE', 'Vel RMSE', 'Att RMSE'};
ekf_values = [ekf_pos_rmse, ekf_vel_rmse, rad2deg(ekf_att_rmse)];
ukf_values = [ukf_pos_rmse, ukf_vel_rmse, rad2deg(ukf_att_rmse)];

x = 1:length(metrics);
width = 0.35;
bar(x - width/2, ekf_values, width, 'FaceColor', 'blue', 'DisplayName', 'EKF');
hold on;
bar(x + width/2, ukf_values, width, 'FaceColor', 'red', 'DisplayName', 'UKF');
set(gca, 'XTickLabel', metrics);
ylabel('RMSE');
title('Performance Metrics');
legend('Location', 'best');
grid on;

% Computational time comparison
subplot(3, 4, 9);
computation_data = [ekf_total_time, ukf_total_time];
computation_labels = {'EKF', 'UKF'};
bar(computation_data, 'FaceColor', [0.7, 0.7, 0.7]);
set(gca, 'XTickLabel', computation_labels);
ylabel('Total Computation Time (s)');
title('Computational Performance');
grid on;

% Error distributions
subplot(3, 4, 10);
histogram(ekf_pos_errors, 30, 'FaceAlpha', 0.6, 'DisplayName', 'EKF');
hold on;
histogram(ukf_pos_errors, 30, 'FaceAlpha', 0.6, 'DisplayName', 'UKF');
xlabel('Position Error (m)');
ylabel('Frequency');
title('Position Error Distribution');
legend('Location', 'best');
grid on;

% Computation time evolution
subplot(3, 4, 11);
plot(t, ekf_computation_times*1000, 'b-', 'LineWidth', 1, 'DisplayName', 'EKF');
hold on;
plot(t, ukf_computation_times*1000, 'r--', 'LineWidth', 1, 'DisplayName', 'UKF');
xlabel('Time (s)');
ylabel('Computation Time (ms)');
title('Per-Step Computation Time');
legend('Location', 'best');
grid on;

% Summary performance
subplot(3, 4, 12);
improvement_data = [pos_improvement, vel_improvement, att_improvement];
improvement_labels = {'Position', 'Velocity', 'Attitude'};
colors = improvement_data;
colors(colors >= 0) = 1;  % Green for improvements
colors(colors < 0) = 0;   % Red for degradation
bar(improvement_data, 'FaceColor', 'flat', 'CData', [colors', 1-colors', zeros(length(colors), 1)]);
set(gca, 'XTickLabel', improvement_labels);
ylabel('UKF Improvement (%)');
title('UKF vs EKF Improvements');
grid on;
ylim([min(improvement_data)-5, max(improvement_data)+5]);

sgtitle('EKF vs UKF Performance Comparison (Random-Walk Simulation)', 'FontSize', 16, 'FontWeight', 'bold');

% Save plots
savefig('EKF_vs_UKF_Random_Comparison.fig');
saveas(gcf, 'EKF_vs_UKF_Random_Comparison.png');

% Create animation comparison
fprintf('\nGenerating animation with both estimates...\n');
try
    % Modified animation function call with both estimates
    figure('Name', 'EKF vs UKF Animation Comparison', 'Position', [100, 100, 1200, 800]);
    
    % Plot final trajectories
    subplot(2, 2, [1, 3]);
    plot3(x_true_hist(1,:), x_true_hist(2,:), x_true_hist(3,:), 'k-', 'LineWidth', 3, 'DisplayName', 'True Trajectory');
    hold on;
    plot3(x_est_ekf_hist(1,:), x_est_ekf_hist(2,:), x_est_ekf_hist(3,:), 'b--', 'LineWidth', 2, 'DisplayName', 'EKF Estimate');
    plot3(x_est_ukf_hist(1,:), x_est_ukf_hist(2,:), x_est_ukf_hist(3,:), 'r:', 'LineWidth', 2, 'DisplayName', 'UKF Estimate');
    
    % Mark start and end points
    plot3(x_true_hist(1,1), x_true_hist(2,1), x_true_hist(3,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    plot3(x_true_hist(1,end), x_true_hist(2,end), x_true_hist(3,end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'End');
    
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('Complete Trajectory Comparison');
    legend('Location', 'best');
    grid on;
    axis equal;
    view(3);
    
    % Error evolution
    subplot(2, 2, 2);
    semilogy(t, ekf_pos_errors, 'b-', 'LineWidth', 2, 'DisplayName', 'EKF');
    hold on;
    semilogy(t, ukf_pos_errors, 'r--', 'LineWidth', 2, 'DisplayName', 'UKF');
    xlabel('Time (s)'); ylabel('Position Error (m)');
    title('Position Error Evolution (Log Scale)');
    legend('Location', 'best');
    grid on;
    
    % Final statistics
    subplot(2, 2, 4);
    text(0.1, 0.8, sprintf('EKF Position RMSE: %.3f m', ekf_pos_rmse), 'FontSize', 12);
    text(0.1, 0.7, sprintf('UKF Position RMSE: %.3f m', ukf_pos_rmse), 'FontSize', 12);
    text(0.1, 0.6, sprintf('Position Improvement: %.1f%%', pos_improvement), 'FontSize', 12, 'FontWeight', 'bold');
    text(0.1, 0.5, sprintf('EKF Computation: %.3f s', ekf_total_time), 'FontSize', 12);
    text(0.1, 0.4, sprintf('UKF Computation: %.3f s', ukf_total_time), 'FontSize', 12);
    text(0.1, 0.3, sprintf('Time Overhead: %.1f%%', time_overhead), 'FontSize', 12, 'FontWeight', 'bold');
    
    if pos_improvement > 0
        text(0.1, 0.1, '🎉 UKF WINS!', 'FontSize', 16, 'FontWeight', 'bold', 'Color', 'green');
    else
        text(0.1, 0.1, '📊 EKF WINS!', 'FontSize', 16, 'FontWeight', 'bold', 'Color', 'blue');
    end
    
    xlim([0, 1]); ylim([0, 1]);
    title('Performance Summary');
    axis off;
    
    savefig('EKF_vs_UKF_Final_Comparison.fig');
    saveas(gcf, 'EKF_vs_UKF_Final_Comparison.png');
    
catch ME
    fprintf('Animation failed: %s\n', ME.message);
end

% Save all results
save('EKF_vs_UKF_Random_Results.mat', 't', 'x_true_hist', 'x_est_ekf_hist', 'x_est_ukf_hist', ...
     'ekf_pos_rmse', 'ukf_pos_rmse', 'ekf_vel_rmse', 'ukf_vel_rmse', ...
     'ekf_att_rmse', 'ukf_att_rmse', 'ekf_total_time', 'ukf_total_time', ...
     'pos_improvement', 'vel_improvement', 'att_improvement', 'time_overhead');

fprintf('\n=== EKF vs UKF Comparison Complete ===\n');
fprintf('Results saved to: EKF_vs_UKF_Random_Results.mat\n');
fprintf('Plots saved as: EKF_vs_UKF_Random_Comparison.png/.fig\n\n');

if pos_improvement > 5
    fprintf('✅ UKF shows significant improvement over EKF (%.1f%% better)!\n', pos_improvement);
    fprintf('   Ready for RL integration with UKF state estimation.\n');
elseif pos_improvement > 0
    fprintf('✅ UKF shows modest improvement over EKF (%.1f%% better).\n', pos_improvement);
    fprintf('   Both filters are suitable for RL integration.\n');
else
    fprintf('📊 EKF shows better performance than UKF.\n');
    fprintf('   Recommend using EKF for RL integration.\n');
end
