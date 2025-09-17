%% fixed_ukf_test.m - Corrected UKF vs EKF Test
% PURPOSE
% Fix the function call errors and use very conservative UKF parameters

clear; clc; close all;

fprintf('=== FIXED UKF vs EKF Test ===\n\n');

%% 1. Load parameters and make UKF VERY conservative
parameters;

% ULTRA-CONSERVATIVE UKF settings to prevent divergence
params.Q_ukf = params.Q * 5.0;           % Much higher process noise
params.innovation_gate_gps = 50.0;       % Very relaxed gates
params.innovation_gate_baro = 20.0;      
params.innovation_gate_mag = deg2rad(90);
params.adaptive_noise = false;           % Disable adaptive features

% Add path
addpath(fullfile(fileparts(mfilename('fullpath')), 'noanim_benchmarks','filters'));

fprintf('Using ULTRA-CONSERVATIVE UKF settings:\n');
fprintf('  Q scaling: 5.0x\n');
fprintf('  GPS gate: %.1f m\n', params.innovation_gate_gps);
fprintf('  Baro gate: %.1f m\n', params.innovation_gate_baro);
fprintf('  Mag gate: %.1f deg\n', rad2deg(params.innovation_gate_mag));

%% 2. Short simulation setup (30s only for testing)
dt = params.Ts.physics;
T_end = 30;  % Short test
t = 0:dt:T_end;
N = length(t);

% Initial conditions
x_true = [0; 0; 0; 0; 0; 0; 0; 0; 0];
x_est_ekf = x_true;
x_est_ukf = x_true;
P_ekf = 0.1 * eye(9);
P_ukf = 0.1 * eye(9);

% Storage
x_true_hist = zeros(9, N);
x_est_ekf_hist = zeros(9, N);
x_est_ukf_hist = zeros(9, N);

% Simple controller
Kp_vel = [0.6; 0.6; 0.8];
prev_att_err = zeros(3,1);

fprintf('\nStarting 30s test simulation...\n');

%% 3. Fixed simulation loop
for k = 1:N
    current_time = t(k);

    % Simple velocity command
    if current_time < 5
        vel_cmd = [0; 0; 0];
    else
        vel_cmd = [1; 0.5; 0];  % Simple constant command
    end

    % Simple control
    vel_est = x_est_ekf(4:6);
    vel_error = vel_cmd - vel_est;
    accel_des = Kp_vel .* vel_error;
    accel_des(3) = accel_des(3) + abs(params.g(3));
    
    thrust = params.mass * norm(accel_des);
    thrust = max(min(thrust, 2*params.mass*abs(params.g(3))), 0.5*params.mass*abs(params.g(3)));
    
    u = [thrust; 0; 0; 0];  % Simple thrust-only control

    % True dynamics
    x_dot = drone_dynamics(current_time, x_true, u, params);
    x_true = x_true + x_dot * dt;
    x_true(7:9) = wrapToPi(x_true(7:9));

    % Sensors
    sensors = sensor_model(x_true, params, current_time);
    imu_meas = [sensors.accel; sensors.gyro];

    % === EKF (working version) ===
    if mod(k-1, round(params.Ts.IMU/dt)) == 0
        try
            [x_est_ekf, P_ekf] = ekf9_sensor_only_step(x_est_ekf, P_ekf, imu_meas, params, params.Ts.IMU);
        catch
        end
    end
    if mod(k-1, round(params.Ts.GPS/dt)) == 0
        try
            [x_est_ekf, P_ekf] = ekf_sensor_only(x_est_ekf, P_ekf, imu_meas, sensors.gps, params, 0, 'GPS');
        catch
        end
    end

    % === FIXED UKF ===
    % IMU prediction (CORRECTED function call)
    if mod(k-1, round(params.Ts.IMU/dt)) == 0
        try
            % CORRECT signature: use 'IMU' sensor type with empty measurement for prediction
            [x_est_ukf, P_ukf] = ukf9_step(x_est_ukf, P_ukf, imu_meas, [], params, params.Ts.IMU, 'IMU');
        catch ME
            fprintf('UKF IMU failed at t=%.3f: %s\n', current_time, ME.message);
        end
    end
    
    % GPS update (CORRECTED function call)
    if mod(k-1, round(params.Ts.GPS/dt)) == 0
        try
            % CORRECT signature: GPS measurement with 0 dt for update
            [x_est_ukf, P_ukf] = ukf9_step(x_est_ukf, P_ukf, imu_meas, sensors.gps, params, 0, 'GPS');
        catch ME
            fprintf('UKF GPS failed at t=%.3f: %s\n', current_time, ME.message);
        end
    end
    
    % Baro update (CORRECTED function call)
    if mod(k-1, round(params.Ts.Baro/dt)) == 0
        try
            [x_est_ukf, P_ukf] = ukf9_step(x_est_ukf, P_ukf, imu_meas, sensors.baro, params, 0, 'Baro');
        catch ME
            fprintf('UKF Baro failed at t=%.3f: %s\n', current_time, ME.message);
        end
    end

    % Store results
    x_true_hist(:,k) = x_true;
    x_est_ekf_hist(:,k) = x_est_ekf;
    x_est_ukf_hist(:,k) = x_est_ukf;

    % Progress
    if mod(k, round(5/dt)) == 1
        ekf_err = norm(x_est_ekf(1:3) - x_true(1:3));
        ukf_err = norm(x_est_ukf(1:3) - x_true(1:3));
        fprintf('t=%5.1fs  EKF_err=%.2fm  UKF_err=%.2fm\n', current_time, ekf_err, ukf_err);
        
        % Stop if UKF diverges too much
        if ukf_err > 50
            fprintf('⚠️  UKF diverged too much (%.1fm), stopping test\n', ukf_err);
            break;
        end
    end
end

%% 4. Results
fprintf('\n=== FIXED UKF TEST RESULTS ===\n');

% Calculate final errors
ekf_pos_errors = sqrt(sum((x_true_hist(1:3,1:k) - x_est_ekf_hist(1:3,1:k)).^2, 1));
ukf_pos_errors = sqrt(sum((x_true_hist(1:3,1:k) - x_est_ukf_hist(1:3,1:k)).^2, 1));

ekf_final_error = mean(ekf_pos_errors(end-50:end));  % Last 50 points
ukf_final_error = mean(ukf_pos_errors(end-50:end));

fprintf('Final Position Errors:\n');
fprintf('  EKF: %.2f m\n', ekf_final_error);
fprintf('  UKF: %.2f m\n', ukf_final_error);

if ukf_final_error < 5.0
    fprintf('\n✅ SUCCESS: UKF is now stable! (< 5m error)\n');
    fprintf('📈 UKF can be used for RL training\n');
elseif ukf_final_error < 20.0
    fprintf('\n⚠️  UKF improved but still drifting (%.1fm error)\n', ukf_final_error);
    fprintf('🔧 Need more conservative parameters or use EKF for RL\n');
else
    fprintf('\n❌ UKF still diverging (%.1fm error)\n', ukf_final_error);
    fprintf('💡 Recommend using EKF for RL training\n');
end

%% 5. Detailed Error Analysis (Enhanced)
fprintf('\n=== DETAILED ERROR ANALYSIS ===\n');

% Calculate all error types
ekf_pos_errors = sqrt(sum((x_true_hist(1:3,1:k) - x_est_ekf_hist(1:3,1:k)).^2, 1));
ukf_pos_errors = sqrt(sum((x_true_hist(1:3,1:k) - x_est_ukf_hist(1:3,1:k)).^2, 1));

ekf_vel_errors = sqrt(sum((x_true_hist(4:6,1:k) - x_est_ekf_hist(4:6,1:k)).^2, 1));
ukf_vel_errors = sqrt(sum((x_true_hist(4:6,1:k) - x_est_ukf_hist(4:6,1:k)).^2, 1));

ekf_att_errors = abs(atan2(sin(x_true_hist(7:9,1:k) - x_est_ekf_hist(7:9,1:k)), cos(x_true_hist(7:9,1:k) - x_est_ekf_hist(7:9,1:k))));
ukf_att_errors = abs(atan2(sin(x_true_hist(7:9,1:k) - x_est_ukf_hist(7:9,1:k)), cos(x_true_hist(7:9,1:k) - x_est_ukf_hist(7:9,1:k))));

% Enhanced RMSE calculations
ekf_vel_rmse = sqrt(mean(ekf_vel_errors.^2));
ukf_vel_rmse = sqrt(mean(ukf_vel_errors.^2));

ekf_att_rmse = sqrt(mean(ekf_att_errors(:).^2));
ukf_att_rmse = sqrt(mean(ukf_att_errors(:).^2));

% Improvement metrics
vel_improvement = (ekf_vel_rmse - ukf_vel_rmse) / ekf_vel_rmse * 100;
att_improvement = (ekf_att_rmse - ukf_att_rmse) / ekf_att_rmse * 100;

fprintf('Extended Performance Metrics:\n');
fprintf('  Velocity RMSE - EKF: %.3f m/s, UKF: %.3f m/s (%.1f%% improvement)\n', ekf_vel_rmse, ukf_vel_rmse, vel_improvement);
fprintf('  Attitude RMSE - EKF: %.3f deg, UKF: %.3f deg (%.1f%% improvement)\n', rad2deg(ekf_att_rmse), rad2deg(ukf_att_rmse), att_improvement);

%% 6. Comprehensive Analysis Plots (12-subplot detailed analysis)
figure('Name', 'Comprehensive UKF vs EKF Analysis', 'Position', [50, 50, 1600, 1200]);

% Position errors over time
subplot(3, 4, 1);
plot(t(1:k), ekf_pos_errors, 'b-', 'LineWidth', 1.5, 'DisplayName', 'EKF');
hold on;
plot(t(1:k), ukf_pos_errors, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Fixed UKF');
xlabel('Time (s)'); ylabel('Position Error (m)');
title('Position Estimation Error');
legend('Location', 'best'); grid on;

% Velocity errors over time
subplot(3, 4, 2);
plot(t(1:k), ekf_vel_errors, 'b-', 'LineWidth', 1.5, 'DisplayName', 'EKF');
hold on;
plot(t(1:k), ukf_vel_errors, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Fixed UKF');
xlabel('Time (s)'); ylabel('Velocity Error (m/s)');
title('Velocity Estimation Error');
legend('Location', 'best'); grid on;

% Attitude errors (yaw only for clarity)
subplot(3, 4, 3);
plot(t(1:k), rad2deg(ekf_att_errors(3,:)), 'b-', 'LineWidth', 1.5, 'DisplayName', 'EKF');
hold on;
plot(t(1:k), rad2deg(ukf_att_errors(3,:)), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Fixed UKF');
xlabel('Time (s)'); ylabel('Yaw Error (deg)');
title('Yaw Estimation Error');
legend('Location', 'best'); grid on;

% 3D trajectory comparison
subplot(3, 4, 4);
plot3(x_true_hist(1,1:k), x_true_hist(2,1:k), x_true_hist(3,1:k), 'k-', 'LineWidth', 2, 'DisplayName', 'True');
hold on;
plot3(x_est_ekf_hist(1,1:k), x_est_ekf_hist(2,1:k), x_est_ekf_hist(3,1:k), 'b--', 'LineWidth', 1.5, 'DisplayName', 'EKF');
plot3(x_est_ukf_hist(1,1:k), x_est_ukf_hist(2,1:k), x_est_ukf_hist(3,1:k), 'r:', 'LineWidth', 1.5, 'DisplayName', 'Fixed UKF');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Trajectory Comparison');
legend('Location', 'best'); grid on; axis equal;

% X position comparison
subplot(3, 4, 5);
plot(t(1:k), x_true_hist(1,1:k), 'k-', 'LineWidth', 2, 'DisplayName', 'True');
hold on;
plot(t(1:k), x_est_ekf_hist(1,1:k), 'b--', 'LineWidth', 1.5, 'DisplayName', 'EKF');
plot(t(1:k), x_est_ukf_hist(1,1:k), 'r:', 'LineWidth', 1.5, 'DisplayName', 'Fixed UKF');
xlabel('Time (s)'); ylabel('X Position (m)');
title('X Position Estimates');
legend('Location', 'best'); grid on;

% Y position comparison
subplot(3, 4, 6);
plot(t(1:k), x_true_hist(2,1:k), 'k-', 'LineWidth', 2, 'DisplayName', 'True');
hold on;
plot(t(1:k), x_est_ekf_hist(2,1:k), 'b--', 'LineWidth', 1.5, 'DisplayName', 'EKF');
plot(t(1:k), x_est_ukf_hist(2,1:k), 'r:', 'LineWidth', 1.5, 'DisplayName', 'Fixed UKF');
xlabel('Time (s)'); ylabel('Y Position (m)');
title('Y Position Estimates');
legend('Location', 'best'); grid on;

% Z position comparison
subplot(3, 4, 7);
plot(t(1:k), x_true_hist(3,1:k), 'k-', 'LineWidth', 2, 'DisplayName', 'True');
hold on;
plot(t(1:k), x_est_ekf_hist(3,1:k), 'b--', 'LineWidth', 1.5, 'DisplayName', 'EKF');
plot(t(1:k), x_est_ukf_hist(3,1:k), 'r:', 'LineWidth', 1.5, 'DisplayName', 'Fixed UKF');
xlabel('Time (s)'); ylabel('Z Position (m)');
title('Z Position Estimates');
legend('Location', 'best'); grid on;

% Performance metrics bar chart
subplot(3, 4, 8);
metrics = {'Pos RMSE', 'Vel RMSE', 'Att RMSE'};
ekf_values = [ekf_final_error, ekf_vel_rmse, rad2deg(ekf_att_rmse)];
ukf_values = [ukf_final_error, ukf_vel_rmse, rad2deg(ukf_att_rmse)];

x = 1:length(metrics);
width = 0.35;
bar(x - width/2, ekf_values, width, 'FaceColor', 'blue', 'DisplayName', 'EKF');
hold on;
bar(x + width/2, ukf_values, width, 'FaceColor', 'red', 'DisplayName', 'Fixed UKF');
set(gca, 'XTickLabel', metrics);
ylabel('RMSE');
title('Performance Metrics');
legend('Location', 'best'); grid on;

% Error distributions
subplot(3, 4, 9);
histogram(ekf_pos_errors, 20, 'FaceAlpha', 0.6, 'DisplayName', 'EKF', 'FaceColor', 'blue');
hold on;
histogram(ukf_pos_errors, 20, 'FaceAlpha', 0.6, 'DisplayName', 'Fixed UKF', 'FaceColor', 'red');
xlabel('Position Error (m)');
ylabel('Frequency');
title('Position Error Distribution');
legend('Location', 'best'); grid on;

% Log scale error evolution
subplot(3, 4, 10);
semilogy(t(1:k), ekf_pos_errors, 'b-', 'LineWidth', 1.5, 'DisplayName', 'EKF');
hold on;
semilogy(t(1:k), ukf_pos_errors, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Fixed UKF');
xlabel('Time (s)'); ylabel('Position Error (m, log scale)');
title('Error Evolution (Log Scale)');
legend('Location', 'best'); grid on;

% Improvement comparison
subplot(3, 4, 11);
pos_improvement_local = (ekf_final_error - ukf_final_error) / ekf_final_error * 100;
improvement_data = [pos_improvement_local, vel_improvement, att_improvement];
improvement_labels = {'Position', 'Velocity', 'Attitude'};
colors = improvement_data;
colors(colors >= 0) = 1;  % Green for improvements
colors(colors < 0) = 0;   % Red for degradation
bar_colors = [colors', 1-colors', zeros(length(colors), 1)];
bar(improvement_data, 'FaceColor', 'flat', 'CData', bar_colors);
set(gca, 'XTickLabel', improvement_labels);
ylabel('UKF Improvement (%)');
title('Fixed UKF vs EKF Improvements');
grid on;

% Summary text
subplot(3, 4, 12);
if ukf_final_error < 3
    status_text = 'UKF STABLE ✅';
else
    status_text = 'UKF UNSTABLE ⚠️';
end

summary_text = {
    sprintf('Position RMSE:');
    sprintf('  EKF: %.3f m', ekf_final_error);
    sprintf('  UKF: %.3f m', ukf_final_error);
    sprintf('  Improvement: %.1f%%', pos_improvement_local);
    '';
    sprintf('Velocity RMSE:');
    sprintf('  EKF: %.3f m/s', ekf_vel_rmse);
    sprintf('  UKF: %.3f m/s', ukf_vel_rmse);
    sprintf('  Improvement: %.1f%%', vel_improvement);
    '';
    sprintf('Status: %s', status_text)
};

text(0.05, 0.95, summary_text, 'FontSize', 10, 'VerticalAlignment', 'top', 'FontName', 'FixedWidth');
xlim([0, 1]); ylim([0, 1]);
title('Performance Summary');
axis off;

sgtitle('Comprehensive Fixed UKF vs EKF Analysis', 'FontSize', 16, 'FontWeight', 'bold');

% Save comprehensive plots
savefig('Fixed_UKF_Comprehensive_Analysis.fig');
saveas(gcf, 'Fixed_UKF_Comprehensive_Analysis.png');

fprintf('\n✓ Results plot saved as Fixed_UKF_Test_Results.png\n');

%% 6. Decision
fprintf('\n=== RECOMMENDATION ===\n');
if ukf_final_error < 2.0
    fprintf('🎯 UKF is working well! Use UKF for RL training.\n');
    fprintf('   Command: cd rl_obstacle_avoidance; run(''train_rl_system.m'');\n');
elseif ukf_final_error < 10.0
    fprintf('📊 UKF is stable but EKF is better. Choose based on preference.\n');
    fprintf('   EKF recommended for RL: cd rl_obstacle_avoidance; run(''train_rl_system.m'');\n');
else
    fprintf('🚀 UKF needs more work. Proceed with proven EKF for RL training.\n');
    fprintf('   Command: cd rl_obstacle_avoidance; run(''train_rl_system.m'');\n');
end
