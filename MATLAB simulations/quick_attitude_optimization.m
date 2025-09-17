%% QUICK ATTITUDE OPTIMIZATION
% Fast targeted search for attitude RMSE < 5 degrees

clear; clc;

fprintf('=== QUICK ATTITUDE OPTIMIZATION ===\n');
fprintf('Target: Attitude RMSE < 5 degrees\n\n');

% Add necessary paths
addpath('noanim_benchmarks/filters');

%% Test specific promising combinations
% Based on analysis: lower Q_att_scale and specific R_mag_scale work better

promising_combinations = [
    % Q_scale, R_gps, R_baro, R_mag, Q_att, P_scale
    1.2, 1.2, 0.9, 0.6, 0.4, 0.9;  % Very low Q_att, low R_mag
    1.3, 1.3, 0.8, 0.7, 0.5, 0.8;  % Low Q_att, low R_mag
    1.1, 1.1, 1.0, 0.5, 0.3, 1.0;  % Very low Q_att, very low R_mag
    1.4, 1.4, 0.9, 0.8, 0.6, 0.9;  % Low Q_att, moderate R_mag
    1.0, 1.0, 0.8, 0.4, 0.2, 0.8;  % Extremely low Q_att, very low R_mag
    1.5, 1.5, 1.0, 0.9, 0.7, 1.0;  % Low Q_att, moderate R_mag
];

best_params = [];
best_attitude = inf;
best_results = [];

fprintf('Testing %d promising combinations...\n', size(promising_combinations, 1));

for i = 1:size(promising_combinations, 1)
    combo = promising_combinations(i, :);
    
    fprintf('Test %d/%d: Q_att=%.2f, R_mag=%.2f... ', i, size(promising_combinations, 1), combo(5), combo(4));
    
    try
        results = rapid_test_ekf('duration', 15, ...
            'Q_scale', combo(1), ...
            'R_gps_scale', combo(2), ...
            'R_baro_scale', combo(3), ...
            'R_mag_scale', combo(4), ...
            'Q_att_scale', combo(5), ...
            'P_scale', combo(6), ...
            'verbose', false);
        
        fprintf('Attitude = %.2f°\n', results.att_rmse_deg);
        
        if results.att_rmse_deg < best_attitude
            best_attitude = results.att_rmse_deg;
            best_params = combo;
            best_results = results;
            
            if best_attitude < 5.0
                fprintf('🎯 TARGET ACHIEVED! Attitude < 5°\n');
            end
        end
        
    catch ME
        fprintf('Failed: %s\n', ME.message);
    end
end

%% Display Results
fprintf('\n=== QUICK OPTIMIZATION RESULTS ===\n');

if ~isempty(best_params)
    fprintf('\nBest Parameters Found:\n');
    fprintf('  Q_scale: %.3f\n', best_params(1));
    fprintf('  R_gps_scale: %.3f\n', best_params(2));
    fprintf('  R_baro_scale: %.3f\n', best_params(3));
    fprintf('  R_mag_scale: %.3f\n', best_params(4));
    fprintf('  Q_att_scale: %.3f\n', best_params(5));
    fprintf('  P_scale: %.3f\n', best_params(6));
    
    fprintf('\nPerformance:\n');
    fprintf('  Position RMSE: %.3f m\n', best_results.pos_rmse);
    fprintf('  Velocity RMSE: %.3f m/s\n', best_results.vel_rmse);
    fprintf('  Attitude RMSE: %.2f°\n', best_results.att_rmse_deg);
    fprintf('  Overall Score: %.3f\n', best_results.performance_score);
    
    % Check targets
    fprintf('\nTarget Achievement:\n');
    if best_attitude < 5.0
        att_status = '✅ ACHIEVED';
    else
        att_status = '❌ Not achieved';
    end
    if best_results.vel_rmse < 1.5
        vel_status = '✅ ACHIEVED';
    else
        vel_status = '❌ Not achieved';
    end
    if best_results.pos_rmse < 1.0
        pos_status = '✅ ACHIEVED';
    else
        pos_status = '❌ Not achieved';
    end
    
    fprintf('  Attitude < 5°: %s (%.2f°)\n', att_status, best_attitude);
    fprintf('  Velocity < 1.5 m/s: %s (%.3f m/s)\n', vel_status, best_results.vel_rmse);
    fprintf('  Position < 1 m: %s (%.3f m)\n', pos_status, best_results.pos_rmse);
    
    % Save results
    save('quick_attitude_results.mat', 'best_params', 'best_results', 'best_attitude');
    fprintf('\nResults saved to quick_attitude_results.mat\n');
    
    % If target achieved, offer to apply parameters
    if best_attitude < 5.0
        fprintf('\n🎯 TARGET ACHIEVED! Would you like to apply these parameters? (y/n): ');
        % Note: In actual usage, you would input 'y' to apply
        fprintf('To apply: run apply_quick_optimization_results.m\n');
    end
    
else
    fprintf('No valid results found.\n');
end

fprintf('\n=== QUICK OPTIMIZATION COMPLETE ===\n');
