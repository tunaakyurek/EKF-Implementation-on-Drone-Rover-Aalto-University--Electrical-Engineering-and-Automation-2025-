%% EKF-ONLY PARAMETER TUNING DEMONSTRATION
% This script demonstrates rapid EKF parameter tuning without animations
% Focuses only on the 9-state EKF for maximum speed

% Add necessary paths for EKF functions
addpath('noanim_benchmarks/filters');

clear; clc; close all;

fprintf('=== EKF-ONLY PARAMETER TUNING DEMO ===\n\n');

%% Test 1: Default Parameters (Baseline)
fprintf('Test 1: Baseline EKF Performance\n');
fprintf('--------------------------------\n');
results_baseline = rapid_test_ekf('duration', 60, 'verbose', true);

%% Test 2: Increased Attitude Process Noise (Better Attitude Tracking)
fprintf('\n\nTest 2: Increased Attitude Process Noise (Q_att_scale = 2.0)\n');
fprintf('------------------------------------------------------------\n');
results_high_Q_att = rapid_test_ekf('duration', 60, 'Q_att_scale', 2.0, 'verbose', true);

%% Test 3: Decreased Magnetometer Noise (More Trust in Mag)
fprintf('\n\nTest 3: Decreased Magnetometer Noise (R_mag_scale = 0.7)\n');
fprintf('--------------------------------------------------------\n');
results_low_mag = rapid_test_ekf('duration', 60, 'R_mag_scale', 0.7, 'verbose', true);

%% Test 4: Combined Attitude Tuning
fprintf('\n\nTest 4: Combined Attitude Tuning (Q_att=1.5, R_mag=0.8)\n');
fprintf('-----------------------------------------------------\n');
results_att_tune = rapid_test_ekf('duration', 60, 'Q_att_scale', 1.5, 'R_mag_scale', 0.8, 'verbose', true);

%% Compare Results
fprintf('\n\n=== COMPARISON RESULTS ===\n');
fprintf('%-20s | %-8s | %-8s | %-8s | %-8s | %-8s\n', 'Test', 'Pos RMSE', 'Vel RMSE', 'Att RMSE', 'Penalty', 'Score');
fprintf('%-20s-+-%-8s-+-%-8s-+-%-8s-+-%-8s-+-%-8s\n', repmat('-',1,20), repmat('-',1,8), repmat('-',1,8), repmat('-',1,8), repmat('-',1,8), repmat('-',1,8));
fprintf('%-20s | %-8.3f | %-8.3f | %-8.2f | %-8.1f | %-8.3f\n', 'Baseline', results_baseline.pos_rmse, results_baseline.vel_rmse, results_baseline.att_rmse_deg, results_baseline.attitude_penalty, results_baseline.performance_score);
fprintf('%-20s | %-8.3f | %-8.3f | %-8.2f | %-8.1f | %-8.3f\n', 'High Q_att', results_high_Q_att.pos_rmse, results_high_Q_att.vel_rmse, results_high_Q_att.att_rmse_deg, results_high_Q_att.attitude_penalty, results_high_Q_att.performance_score);
fprintf('%-20s | %-8.3f | %-8.3f | %-8.2f | %-8.1f | %-8.3f\n', 'Low Mag Noise', results_low_mag.pos_rmse, results_low_mag.vel_rmse, results_low_mag.att_rmse_deg, results_low_mag.attitude_penalty, results_low_mag.performance_score);
fprintf('%-20s | %-8.3f | %-8.3f | %-8.2f | %-8.1f | %-8.3f\n', 'Attitude Tuned', results_att_tune.pos_rmse, results_att_tune.vel_rmse, results_att_tune.att_rmse_deg, results_att_tune.attitude_penalty, results_att_tune.performance_score);

%% Find Best Configuration
all_results = [results_baseline, results_high_Q_att, results_low_mag, results_att_tune];
[~, best_idx] = min([all_results.performance_score]);
test_names = {'Baseline', 'High Q_att', 'Low Mag Noise', 'Attitude Tuned'};

fprintf('\nBest Configuration: %s (Score: %.3f)\n', test_names{best_idx}, all_results(best_idx).performance_score);

%% Quick Optimization Example
fprintf('\n\n=== QUICK OPTIMIZATION EXAMPLE ===\n');
fprintf('Running 10 random parameter combinations...\n');

best_score = inf;
best_params = [];
for i = 1:10
    % Random parameters within reasonable bounds (focus on attitude tuning)
    Q_scale = 0.5 + rand() * 1.5;      % [0.5, 2.0]
    R_gps_scale = 0.6 + rand() * 0.8;  % [0.6, 1.4]
    R_baro_scale = 0.7 + rand() * 0.6; % [0.7, 1.3]
    R_mag_scale = 0.5 + rand() * 1.0;  % [0.5, 1.5] - wider range for attitude tuning
    Q_att_scale = 0.5 + rand() * 2.0;  % [0.5, 2.5] - focus on attitude process noise
    P_scale = 0.8 + rand() * 0.4;      % [0.8, 1.2]
    
    try
        results = rapid_test_ekf('duration', 30, ...
            'Q_scale', Q_scale, ...
            'R_gps_scale', R_gps_scale, ...
            'R_baro_scale', R_baro_scale, ...
            'R_mag_scale', R_mag_scale, ...
            'Q_att_scale', Q_att_scale, ...
            'P_scale', P_scale, ...
            'verbose', false);
        
        if results.performance_score < best_score
            best_score = results.performance_score;
            best_params = [Q_scale, R_gps_scale, R_baro_scale, R_mag_scale, Q_att_scale, P_scale];
        end
        
        fprintf('Test %d: Score = %.3f (Q=%.2f, R_gps=%.2f, R_baro=%.2f, R_mag=%.2f, Q_att=%.2f, P=%.2f)\n', ...
            i, results.performance_score, Q_scale, R_gps_scale, R_baro_scale, R_mag_scale, Q_att_scale, P_scale);
    catch
        fprintf('Test %d: Failed\n', i);
    end
end

if ~isempty(best_params)
    fprintf('\nBest random parameters found:\n');
    fprintf('  Q_scale: %.3f\n', best_params(1));
    fprintf('  R_gps_scale: %.3f\n', best_params(2));
    fprintf('  R_baro_scale: %.3f\n', best_params(3));
    fprintf('  R_mag_scale: %.3f\n', best_params(4));
    fprintf('  Q_att_scale: %.3f\n', best_params(5));
    fprintf('  P_scale: %.3f\n', best_params(6));
    fprintf('  Best score: %.3f\n', best_score);
end

fprintf('\n=== TUNING COMPLETE ===\n');
fprintf('Use rapid_test_ekf() for custom parameter testing\n');
fprintf('Use optimize_ekf_parameters() for automated optimization\n');
