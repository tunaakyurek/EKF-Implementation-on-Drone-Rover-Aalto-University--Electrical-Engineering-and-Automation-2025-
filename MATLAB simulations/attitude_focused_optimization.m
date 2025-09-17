%% ATTITUDE-FOCUSED EKF OPTIMIZATION
% Specifically targets attitude RMSE < 5 degrees
% Uses targeted parameter ranges based on analysis

clear; clc;

fprintf('=== ATTITUDE-FOCUSED EKF OPTIMIZATION ===\n');
fprintf('Target: Attitude RMSE < 5 degrees\n\n');

% Add necessary paths
addpath('noanim_benchmarks/filters');

%% Current best parameters (baseline)
current_best = [1.377, 1.370, 0.956, 1.115, 0.926, 0.880];
current_attitude = 8.27; % degrees

fprintf('Current best attitude: %.2f°\n', current_attitude);
fprintf('Target: < 5.0°\n\n');

%% Attitude-focused parameter exploration
% Based on analysis: lower Q_att_scale and specific R_mag_scale ranges work better

best_params = [];
best_attitude = inf;
best_results = [];

fprintf('Testing attitude-focused parameter combinations...\n');

% Focus on Q_att_scale (most important for attitude) and R_mag_scale
Q_att_values = [0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9];  % Lower values for better attitude
R_mag_values = [0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4];  % Wider range
Q_scale_values = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5];  % Moderate range
R_gps_values = [1.0, 1.1, 1.2, 1.3, 1.4];  % Moderate range
R_baro_values = [0.8, 0.9, 1.0, 1.1];  % Conservative range
P_scale_values = [0.8, 0.9, 1.0, 1.1];  % Conservative range

test_count = 0;
total_tests = length(Q_att_values) * length(R_mag_values) * length(Q_scale_values) * length(R_gps_values) * length(R_baro_values) * length(P_scale_values);

fprintf('Total combinations to test: %d\n', total_tests);
fprintf('This will take a while...\n\n');

for q_att = Q_att_values
    for r_mag = R_mag_values
        for q_scale = Q_scale_values
            for r_gps = R_gps_values
                for r_baro = R_baro_values
                    for p_scale = P_scale_values
                        test_count = test_count + 1;
                        
                        if mod(test_count, 100) == 0
                            fprintf('Progress: %d/%d (%.1f%%) - Best attitude so far: %.2f°\n', ...
                                test_count, total_tests, 100*test_count/total_tests, best_attitude);
                        end
                        
                        try
                            % Test this combination
                            results = rapid_test_ekf('duration', 15, ...
                                'Q_scale', q_scale, ...
                                'R_gps_scale', r_gps, ...
                                'R_baro_scale', r_baro, ...
                                'R_mag_scale', r_mag, ...
                                'Q_att_scale', q_att, ...
                                'P_scale', p_scale, ...
                                'verbose', false);
                            
                            % Check if this is the best attitude performance
                            if results.att_rmse_deg < best_attitude
                                best_attitude = results.att_rmse_deg;
                                best_params = [q_scale, r_gps, r_baro, r_mag, q_att, p_scale];
                                best_results = results;
                                
                                fprintf('NEW BEST: Attitude = %.2f° (Q_att=%.2f, R_mag=%.2f)\n', ...
                                    best_attitude, q_att, r_mag);
                                
                                % If we achieve target, we can stop early
                                if best_attitude < 5.0
                                    fprintf('TARGET ACHIEVED! Attitude < 5°\n');
                                    break;
                                end
                            end
                            
                        catch
                            % Skip failed tests
                        end
                    end
                    if best_attitude < 5.0, break; end
                end
                if best_attitude < 5.0, break; end
            end
            if best_attitude < 5.0, break; end
        end
        if best_attitude < 5.0, break; end
    end
    if best_attitude < 5.0, break; end
end

%% Display Results
fprintf('\n=== ATTITUDE-FOCUSED OPTIMIZATION RESULTS ===\n');
fprintf('Tests completed: %d/%d\n', test_count, total_tests);

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
    
    % Improvement over current best
    improvement = current_attitude - best_attitude;
    fprintf('\nImprovement over current best: %.2f° (%.1f%% better)\n', improvement, 100*improvement/current_attitude);
    
    % Save results
    save('attitude_focused_results.mat', 'best_params', 'best_results', 'best_attitude');
    fprintf('\nResults saved to attitude_focused_results.mat\n');
    
else
    fprintf('No valid results found.\n');
end

fprintf('\n=== OPTIMIZATION COMPLETE ===\n');
