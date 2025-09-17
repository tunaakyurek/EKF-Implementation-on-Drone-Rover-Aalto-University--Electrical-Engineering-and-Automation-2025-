%% Test Original Baseline Parameters
% This tests the true original parameters documented in the comments

clear; clc;

fprintf('=== TESTING TRUE ORIGINAL BASELINE PARAMETERS ===\n\n');

% Load current parameters
parameters;

% Store current values to restore later
current_Q = params.Q;
current_R_gps = params.R_gps;
current_R_baro = params.R_baro;
current_R_mag = params.R_mag;
current_gps_sigma_xy = params.GPS.sigma_xy;
current_gps_sigma_z = params.GPS.sigma_z;
current_baro_sigma_z = params.Baro.sigma_z;
current_mag_sigma_deg = params.Mag.sigma_deg;
current_innovation_gates = [params.innovation_gate_gps, params.innovation_gate_baro, params.innovation_gate_mag, params.innovation_gate_imu];

% Apply TRUE original baseline parameters
fprintf('Applying true original baseline parameters...\n');

% Original Q matrix
params.Q = diag([0.02 0.02 0.02, 0.10 0.10 0.12, 0.05 0.05 0.06]);

% Original sensor noise values
params.GPS.sigma_xy = 0.8;     % meters (horizontal)
params.GPS.sigma_z = 1.6;      % meters (vertical)
params.Baro.sigma_z = 0.35;    % meters
params.Mag.sigma_deg = 2.0;    % degrees

% Original R matrices (recalculated with original sensor noise)
params.R_gps = diag([params.GPS.sigma_xy^2, params.GPS.sigma_xy^2, params.GPS.sigma_z^2]);
params.R_baro = (params.Baro.sigma_z)^2;
params.R_mag = (params.Mag.sigma_deg * pi/180)^2;

% Original innovation gates
params.innovation_gate_gps = 8.0;   % meters
params.innovation_gate_baro = 4.0;  % meters
params.innovation_gate_mag = deg2rad(30); % radians
params.innovation_gate_imu = 15.0;  % m/s²

fprintf('Original parameters applied:\n');
fprintf('  Q matrix: diag([0.02 0.02 0.02, 0.10 0.10 0.12, 0.05 0.05 0.06])\n');
fprintf('  GPS noise: σ_xy=%.1f, σ_z=%.1f\n', params.GPS.sigma_xy, params.GPS.sigma_z);
fprintf('  Baro noise: σ_z=%.2f\n', params.Baro.sigma_z);
fprintf('  Mag noise: σ=%.1f deg\n', params.Mag.sigma_deg);
fprintf('  Innovation gates: GPS=%.1f, Baro=%.1f, Mag=%.1f deg, IMU=%.1f\n', ...
    params.innovation_gate_gps, params.innovation_gate_baro, rad2deg(params.innovation_gate_mag), params.innovation_gate_imu);

% Run test
fprintf('\nRunning 20-second test with true original parameters...\n');
results_original_baseline = rapid_test_ekf('duration', 20, 'verbose', true);

% Restore current parameters
fprintf('\nRestoring current parameters...\n');
params.Q = current_Q;
params.R_gps = current_R_gps;
params.R_baro = current_R_baro;
params.R_mag = current_R_mag;
params.GPS.sigma_xy = current_gps_sigma_xy;
params.GPS.sigma_z = current_gps_sigma_z;
params.Baro.sigma_z = current_baro_sigma_z;
params.Mag.sigma_deg = current_mag_sigma_deg;
params.innovation_gate_gps = current_innovation_gates(1);
params.innovation_gate_baro = current_innovation_gates(2);
params.innovation_gate_mag = current_innovation_gates(3);
params.innovation_gate_imu = current_innovation_gates(4);

fprintf('\n=== TRUE ORIGINAL BASELINE RESULTS ===\n');
fprintf('Position RMSE: %.3f m\n', results_original_baseline.pos_rmse);
fprintf('Velocity RMSE: %.3f m/s\n', results_original_baseline.vel_rmse);
fprintf('Attitude RMSE: %.2f deg\n', results_original_baseline.att_rmse_deg);
fprintf('Overall Score: %.3f\n', results_original_baseline.performance_score);

% Save results for comparison
save('original_baseline_results.mat', 'results_original_baseline');

fprintf('\nResults saved to original_baseline_results.mat\n');
fprintf('Current parameters restored.\n');
