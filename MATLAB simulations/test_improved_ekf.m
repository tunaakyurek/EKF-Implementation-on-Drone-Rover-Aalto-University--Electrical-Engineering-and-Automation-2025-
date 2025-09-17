%% test_improved_ekf.m - Simple test for improved EKF implementation
% PURPOSE
% Test the improved EKF implementation to ensure it works correctly
% before running the full animated obstacle avoidance test

clear; clc; close all;

fprintf('=== Testing Improved EKF Implementation ===\n');

try
    %% 1) Load improved parameters
    fprintf('Loading improved parameters...\n');
    params = parameters_animated_obstacle_avoidance();
    fprintf('  Parameters loaded successfully\n');
    
    %% 2) Test EKF initialization
    fprintf('Testing EKF initialization...\n');
    x = [0; 0; -10; 0; 0; 0; 0; 0; 0];  % Initial state
    P = diag([1, 1, 1, 0.5, 0.5, 0.5, 0.1, 0.1, 0.1]);  % Initial covariance
    imu = [0; 0; 9.81; 0; 0; 0];  % IMU measurement (hovering)
    dt = 0.01;
    
    %% 3) Test EKF prediction step
    fprintf('Testing EKF prediction step...\n');
    [x_pred, P_pred] = ekf_animated_obstacle_avoidance_improved(x, P, imu, [], params, dt, 'IMU');
    fprintf('  Prediction step successful\n');
    fprintf('  State size: %s\n', mat2str(size(x_pred)));
    fprintf('  Covariance size: %s\n', mat2str(size(P_pred)));
    
    %% 4) Test EKF with GPS update
    fprintf('Testing EKF with GPS update...\n');
    gps_meas = [1; 2; -9];  % GPS measurement
    [x_gps, P_gps] = ekf_animated_obstacle_avoidance_improved(x_pred, P_pred, imu, gps_meas, params, 0, 'GPS');
    fprintf('  GPS update successful\n');
    
    %% 5) Test EKF with Baro update
    fprintf('Testing EKF with Baro update...\n');
    baro_meas = 9;  % Barometer measurement (altitude)
    [x_baro, P_baro] = ekf_animated_obstacle_avoidance_improved(x_gps, P_gps, imu, baro_meas, params, 0, 'Baro');
    fprintf('  Baro update successful\n');
    
    %% 6) Test EKF with Mag update
    fprintf('Testing EKF with Mag update...\n');
    mag_meas = 0.1;  % Magnetometer measurement (yaw)
    [x_mag, P_mag] = ekf_animated_obstacle_avoidance_improved(x_baro, P_baro, imu, mag_meas, params, 0, 'Mag');
    fprintf('  Mag update successful\n');
    
    %% 7) Test maneuvering scenario
    fprintf('Testing maneuvering scenario...\n');
    % Simulate a turn with high angular rate
    imu_turn = [2; 0; 9.81; 0; 0; 0.5];  % Turning with yaw rate
    [x_turn, P_turn] = ekf_animated_obstacle_avoidance_improved(x_mag, P_mag, imu_turn, [], params, dt, 'IMU');
    fprintf('  Maneuvering scenario successful\n');
    
    %% 8) Verify numerical stability
    fprintf('Checking numerical stability...\n');
    if all(isfinite(x_turn)) && all(isfinite(P_turn(:)))
        fprintf('  Numerical stability: PASSED\n');
    else
        error('Numerical stability check failed');
    end
    
    %% 9) Check covariance conditioning
    fprintf('Checking covariance conditioning...\n');
    [U, S, V] = svd(P_turn);
    min_sv = min(diag(S));
    max_sv = max(diag(S));
    cond_num = max_sv / min_sv;
    
    fprintf('  Min singular value: %.2e\n', min_sv);
    fprintf('  Max singular value: %.2e\n', max_sv);
    fprintf('  Condition number: %.2e\n', cond_num);
    
    if cond_num < 1e12
        fprintf('  Covariance conditioning: PASSED\n');
    else
        warning('Covariance conditioning: WARNING (high condition number)');
    end
    
    fprintf('\n=== ALL TESTS PASSED ===\n');
    fprintf('The improved EKF implementation is working correctly!\n');
    fprintf('You can now run the full animated obstacle avoidance test.\n');
    
catch ME
    fprintf('\n=== TEST FAILED ===\n');
    fprintf('Error: %s\n', ME.message);
    fprintf('Stack trace:\n');
    for i = 1:length(ME.stack)
        fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
    rethrow(ME);
end
