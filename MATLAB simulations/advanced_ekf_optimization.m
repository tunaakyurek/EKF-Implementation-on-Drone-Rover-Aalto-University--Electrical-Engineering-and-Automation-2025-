%% ADVANCED EKF OPTIMIZATION - Target-Driven Parameter Tuning
% This script uses multiple optimization methods to achieve specific performance targets:
% - Attitude RMSE < 5 degrees
% - Velocity RMSE < 1.5 m/s (already achieved)
% - Position RMSE < 1 m (already achieved)

clear; clc; close all;

fprintf('=== ADVANCED EKF OPTIMIZATION ===\n');
fprintf('Targets: Attitude < 5°, Velocity < 1.5 m/s, Position < 1 m\n\n');

% Add necessary paths
addpath('noanim_benchmarks/filters');

%% Define Performance Targets
targets.attitude_rmse = 5.0;    % degrees
targets.velocity_rmse = 1.5;    % m/s
targets.position_rmse = 1.0;    % m

%% Enhanced Parameter Bounds (Focused on Attitude Improvement)
% Expanded bounds with more granular control for attitude tuning
param_names = {'Q_scale', 'R_gps_scale', 'R_baro_scale', 'R_mag_scale', 'Q_att_scale', 'P_scale'};

% Method 1: Conservative bounds (around current best)
bounds_conservative = [
    0.8,  1.8;   % Q_scale
    0.8,  1.6;   % R_gps_scale
    0.7,  1.3;   % R_baro_scale
    0.8,  1.4;   % R_mag_scale
    0.5,  1.2;   % Q_att_scale (key for attitude)
    0.7,  1.3;   % P_scale
];

% Method 2: Aggressive bounds (wider exploration)
bounds_aggressive = [
    0.5,  2.5;   % Q_scale
    0.5,  2.0;   % R_gps_scale
    0.5,  1.8;   % R_baro_scale
    0.5,  2.0;   % R_mag_scale
    0.2,  1.5;   % Q_att_scale (wider range for attitude)
    0.5,  1.8;   % P_scale
];

% Method 3: Attitude-focused bounds
bounds_attitude_focused = [
    1.0,  1.8;   % Q_scale (moderate)
    1.0,  1.8;   % R_gps_scale (moderate)
    0.8,  1.2;   % R_baro_scale (conservative)
    0.6,  1.4;   % R_mag_scale (wider for attitude)
    0.3,  0.8;   % Q_att_scale (lower for better attitude)
    0.8,  1.2;   % P_scale (conservative)
];

%% Objective Function with Target Penalties
function score = advanced_objective_function(params, targets, duration)
    try
        % Extract parameters
        Q_scale = params(1);
        R_gps_scale = params(2);
        R_baro_scale = params(3);
        R_mag_scale = params(4);
        Q_att_scale = params(5);
        P_scale = params(6);
        
        % Run simulation
        results = rapid_test_ekf('duration', duration, ...
            'Q_scale', Q_scale, ...
            'R_gps_scale', R_gps_scale, ...
            'R_baro_scale', R_baro_scale, ...
            'R_mag_scale', R_mag_scale, ...
            'Q_att_scale', Q_att_scale, ...
            'P_scale', P_scale, ...
            'verbose', false);
        
        % Base score
        base_score = results.performance_score;
        
        % Target penalties (heavily penalize missing targets)
        attitude_penalty = 0;
        velocity_penalty = 0;
        position_penalty = 0;
        
        if results.att_rmse_deg > targets.attitude_rmse
            attitude_penalty = (results.att_rmse_deg - targets.attitude_rmse) * 10; % Heavy penalty
        end
        
        if results.vel_rmse > targets.velocity_rmse
            velocity_penalty = (results.vel_rmse - targets.velocity_rmse) * 5;
        end
        
        if results.pos_rmse > targets.position_rmse
            position_penalty = (results.pos_rmse - targets.position_rmse) * 3;
        end
        
        % Bonus for achieving targets
        target_bonus = 0;
        if results.att_rmse_deg <= targets.attitude_rmse
            target_bonus = target_bonus - 2; % Bonus for good attitude
        end
        if results.vel_rmse <= targets.velocity_rmse
            target_bonus = target_bonus - 1;
        end
        if results.pos_rmse <= targets.position_rmse
            target_bonus = target_bonus - 1;
        end
        
        score = base_score + attitude_penalty + velocity_penalty + position_penalty + target_bonus;
        
    catch ME
        score = 1000; % High penalty for failed simulations
    end
end

%% Method 1: Multi-Start Optimization
fprintf('=== METHOD 1: Multi-Start Optimization ===\n');
best_params_method1 = [];
best_score_method1 = inf;
best_results_method1 = [];

for start = 1:10
    fprintf('Multi-start %d/10...\n', start);
    
    % Random starting point within conservative bounds
    x0 = zeros(1, 6);
    for i = 1:6
        x0(i) = bounds_conservative(i,1) + rand() * (bounds_conservative(i,2) - bounds_conservative(i,1));
    end
    
    try
        options = optimoptions('fmincon', 'Display', 'off', 'MaxIterations', 20);
        [params, score] = fmincon(@(x) advanced_objective_function(x, targets, 15), x0, [], [], [], [], ...
            bounds_conservative(:,1), bounds_conservative(:,2), [], options);
        
        if score < best_score_method1
            best_score_method1 = score;
            best_params_method1 = params;
            
            % Get detailed results
            results = rapid_test_ekf('duration', 20, ...
                'Q_scale', params(1), 'R_gps_scale', params(2), 'R_baro_scale', params(3), ...
                'R_mag_scale', params(4), 'Q_att_scale', params(5), 'P_scale', params(6), ...
                'verbose', false);
            best_results_method1 = results;
        end
    catch
        % Skip failed optimizations
    end
end

%% Method 2: Genetic Algorithm (if available)
fprintf('\n=== METHOD 2: Genetic Algorithm ===\n');
best_params_method2 = [];
best_score_method2 = inf;
best_results_method2 = [];

if exist('ga', 'file')
    try
        options = optimoptions('ga', 'Display', 'off', 'MaxGenerations', 20, 'PopulationSize', 20);
        [params, score] = ga(@(x) advanced_objective_function(x, targets, 10), 6, [], [], [], [], ...
            bounds_aggressive(:,1), bounds_aggressive(:,2), [], options);
        
        best_score_method2 = score;
        best_params_method2 = params;
        
        % Get detailed results
        results = rapid_test_ekf('duration', 20, ...
            'Q_scale', params(1), 'R_gps_scale', params(2), 'R_baro_scale', params(3), ...
            'R_mag_scale', params(4), 'Q_att_scale', params(5), 'P_scale', params(6), ...
            'verbose', false);
        best_results_method2 = results;
        
        fprintf('Genetic Algorithm completed.\n');
    catch
        fprintf('Genetic Algorithm not available or failed.\n');
    end
else
    fprintf('Genetic Algorithm not available (requires Global Optimization Toolbox).\n');
end

%% Method 3: Attitude-Focused Random Search
fprintf('\n=== METHOD 3: Attitude-Focused Random Search ===\n');
best_params_method3 = [];
best_score_method3 = inf;
best_results_method3 = [];

for i = 1:50
    if mod(i, 10) == 0
        fprintf('Random search %d/50...\n', i);
    end
    
    % Generate random parameters within attitude-focused bounds
    params = zeros(1, 6);
    for j = 1:6
        params(j) = bounds_attitude_focused(j,1) + rand() * (bounds_attitude_focused(j,2) - bounds_attitude_focused(j,1));
    end
    
    score = advanced_objective_function(params, targets, 15);
    
    if score < best_score_method3
        best_score_method3 = score;
        best_params_method3 = params;
        
        % Get detailed results
        results = rapid_test_ekf('duration', 20, ...
            'Q_scale', params(1), 'R_gps_scale', params(2), 'R_baro_scale', params(3), ...
            'R_mag_scale', params(4), 'Q_att_scale', params(5), 'P_scale', params(6), ...
            'verbose', false);
        best_results_method3 = results;
    end
end

%% Method 4: Grid Search (Coarse)
fprintf('\n=== METHOD 4: Coarse Grid Search ===\n');
best_params_method4 = [];
best_score_method4 = inf;
best_results_method4 = [];

% Coarse grid around current best parameters
current_best = [1.377, 1.370, 0.956, 1.115, 0.926, 0.880];
grid_ranges = [0.1, 0.1, 0.05, 0.1, 0.1, 0.1]; % ±range around current best

for i = 1:3
    for j = 1:3
        for k = 1:3
            for l = 1:3
                for m = 1:3
                    for n = 1:3
                        params = current_best + ([(i-2), (j-2), (k-2), (l-2), (m-2), (n-2)] .* grid_ranges);
                        
                        % Check bounds
                        if all(params >= bounds_conservative(:,1)') && all(params <= bounds_conservative(:,2)')
                            score = advanced_objective_function(params, targets, 10);
                            
                            if score < best_score_method4
                                best_score_method4 = score;
                                best_params_method4 = params;
                                
                                % Get detailed results
                                results = rapid_test_ekf('duration', 20, ...
                                    'Q_scale', params(1), 'R_gps_scale', params(2), 'R_baro_scale', params(3), ...
                                    'R_mag_scale', params(4), 'Q_att_scale', params(5), 'P_scale', params(6), ...
                                    'verbose', false);
                                best_results_method4 = results;
                            end
                        end
                    end
                end
            end
        end
    end
end

%% Compare All Methods
fprintf('\n=== OPTIMIZATION RESULTS COMPARISON ===\n');

methods = {'Multi-Start', 'Genetic Algorithm', 'Attitude-Focused Random', 'Grid Search'};
best_scores = [best_score_method1, best_score_method2, best_score_method3, best_score_method4];
best_params_all = {best_params_method1, best_params_method2, best_params_method3, best_params_method4};
best_results_all = {best_results_method1, best_results_method2, best_results_method3, best_results_method4};

% Find overall best
[overall_best_score, best_method_idx] = min(best_scores);
overall_best_params = best_params_all{best_method_idx};
overall_best_results = best_results_all{best_method_idx};

fprintf('\nMethod Comparison:\n');
for i = 1:length(methods)
    if ~isempty(best_params_all{i})
        fprintf('%s: Score = %.3f, Att = %.2f°, Vel = %.3f m/s, Pos = %.3f m\n', ...
            methods{i}, best_scores(i), best_results_all{i}.att_rmse_deg, ...
            best_results_all{i}.vel_rmse, best_results_all{i}.pos_rmse);
    else
        fprintf('%s: No valid results\n', methods{i});
    end
end

%% Display Best Results
fprintf('\n=== BEST OVERALL RESULTS ===\n');
fprintf('Best Method: %s\n', methods{best_method_idx});
fprintf('Best Parameters:\n');
for i = 1:length(param_names)
    fprintf('  %s: %.3f\n', param_names{i}, overall_best_params(i));
end

fprintf('\nPerformance:\n');
if overall_best_results.pos_rmse <= targets.position_rmse
    pos_status = '✅';
else
    pos_status = '❌';
end
if overall_best_results.vel_rmse <= targets.velocity_rmse
    vel_status = '✅';
else
    vel_status = '❌';
end
if overall_best_results.att_rmse_deg <= targets.attitude_rmse
    att_status = '✅';
else
    att_status = '❌';
end

fprintf('  Position RMSE: %.3f m (target: < %.1f m) %s\n', ...
    overall_best_results.pos_rmse, targets.position_rmse, pos_status);
fprintf('  Velocity RMSE: %.3f m/s (target: < %.1f m/s) %s\n', ...
    overall_best_results.vel_rmse, targets.velocity_rmse, vel_status);
fprintf('  Attitude RMSE: %.2f° (target: < %.1f°) %s\n', ...
    overall_best_results.att_rmse_deg, targets.attitude_rmse, att_status);
fprintf('  Overall Score: %.3f\n', overall_best_results.performance_score);

%% Save Results
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
filename = sprintf('advanced_optimization_results_%s.mat', timestamp);
save(filename, 'overall_best_params', 'overall_best_results', 'best_method_idx', 'methods', 'targets');
fprintf('\nResults saved to: %s\n', filename);

%% Create Report
report_filename = sprintf('Advanced_Optimization_Report_%s.md', timestamp);
fid = fopen(report_filename, 'w');

fprintf(fid, '# Advanced EKF Optimization Report\n');
fprintf(fid, 'Generated: %s\n\n', datestr(now));
fprintf(fid, '## Optimization Targets\n');
fprintf(fid, '- Attitude RMSE < %.1f°\n', targets.attitude_rmse);
fprintf(fid, '- Velocity RMSE < %.1f m/s\n', targets.velocity_rmse);
fprintf(fid, '- Position RMSE < %.1f m\n\n', targets.position_rmse);

fprintf(fid, '## Best Results\n');
fprintf(fid, '**Method:** %s\n\n', methods{best_method_idx});
fprintf(fid, '**Parameters:**\n');
for i = 1:length(param_names)
    fprintf(fid, '- %s: %.3f\n', param_names{i}, overall_best_params(i));
end

fprintf(fid, '\n**Performance:**\n');
fprintf(fid, '- Position RMSE: %.3f m %s\n', overall_best_results.pos_rmse, ...
    overall_best_results.pos_rmse <= targets.position_rmse ? '✅' : '❌');
fprintf(fid, '- Velocity RMSE: %.3f m/s %s\n', overall_best_results.vel_rmse, ...
    overall_best_results.vel_rmse <= targets.velocity_rmse ? '✅' : '❌');
fprintf(fid, '- Attitude RMSE: %.2f° %s\n', overall_best_results.att_rmse_deg, ...
    overall_best_results.att_rmse_deg <= targets.attitude_rmse ? '✅' : '❌');
fprintf(fid, '- Overall Score: %.3f\n', overall_best_results.performance_score);

fclose(fid);
fprintf('Report saved to: %s\n', report_filename);

fprintf('\n=== OPTIMIZATION COMPLETE ===\n');
