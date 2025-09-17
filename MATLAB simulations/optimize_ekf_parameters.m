function best_params = optimize_ekf_parameters(varargin)
%% EKF-9 PARAMETER OPTIMIZATION - Automated Parameter Tuning
% Uses optimization algorithms to find the best EKF-9 parameters
% Focuses ONLY on the 9-state EKF for maximum speed

% Add necessary paths for EKF functions
addpath('noanim_benchmarks/filters');
%
% USAGE:
%   best_params = optimize_ekf_parameters()                    % Default optimization
%   best_params = optimize_ekf_parameters('method', 'grid')    % Grid search
%   best_params = optimize_ekf_parameters('method', 'fmincon') % Gradient-based
%   best_params = optimize_ekf_parameters('duration', 60)      % Full performance tests
%
% OPTIMIZATION METHODS:
%   'grid'     - Grid search over parameter space (thorough but slow)
%   'fmincon'  - MATLAB's constrained optimization (faster)
%   'random'   - Random sampling (fastest, good for exploration)
%
% PARAMETERS TO OPTIMIZE (EKF-9 only):
%   Q_scale     - Process noise scaling [0.1, 3.0]
%   R_gps_scale - GPS measurement noise scaling [0.5, 2.0]
%   R_baro_scale- Barometer noise scaling [0.5, 2.0]
%   R_mag_scale - Magnetometer noise scaling [0.5, 2.0]
%   P_scale     - Initial covariance scaling [0.5, 2.0]

%% Parse input parameters
p = inputParser;
addParameter(p, 'method', 'fmincon', @(x) ismember(x, {'grid', 'fmincon', 'random'}));
addParameter(p, 'duration', 60, @(x) isnumeric(x) && x > 0);
addParameter(p, 'max_evaluations', 50, @(x) isnumeric(x) && x > 0);
addParameter(p, 'verbose', true, @islogical);
addParameter(p, 'save_results', true, @islogical);
parse(p, varargin{:});

method = p.Results.method;
duration = p.Results.duration;
max_eval = p.Results.max_evaluations;
verbose = p.Results.verbose;
save_results = p.Results.save_results;

if verbose
    fprintf('=== EKF PARAMETER OPTIMIZATION ===\n');
    fprintf('Method: %s, Duration: %.1fs, Max evaluations: %d\n', method, duration, max_eval);
end

%% Define parameter bounds
% [Q_scale, R_gps_scale, R_baro_scale, R_mag_scale, Q_att_scale, P_scale]
param_names = {'Q_scale', 'R_gps_scale', 'R_baro_scale', 'R_mag_scale', 'Q_att_scale', 'P_scale'};
param_bounds = [
    0.1,  3.0;  % Q_scale
    0.5,  2.0;  % R_gps_scale
    0.5,  2.0;  % R_baro_scale
    0.5,  2.0;  % R_mag_scale
    0.1,  5.0;  % Q_att_scale (attitude process noise - wider range for attitude tuning)
    0.5,  2.0;  % P_scale
];

%% Objective function
function score = objective_function(params)
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
        
        % Return performance score (lower is better)
        score = results.performance_score;
        
    catch ME
        % If simulation fails, return high penalty
        score = 1000;
        if verbose
            fprintf('Simulation failed: %s\n', ME.message);
        end
    end
end

%% Run optimization based on method
switch method
    case 'grid'
        best_params = optimize_grid_search();
    case 'fmincon'
        best_params = optimize_fmincon();
    case 'random'
        best_params = optimize_random_search();
end

%% Save results
if save_results
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    filename = sprintf('ekf_optimization_%s_%s.mat', method, timestamp);
    save(filename, 'best_params', 'method', 'duration', 'max_eval');
    if verbose
        fprintf('Results saved to: %s\n', filename);
    end
end

%% Grid Search Implementation
function best_params = optimize_grid_search()
    if verbose
        fprintf('Running grid search optimization...\n');
    end
    
    % Define grid points (coarse grid for speed)
    n_points = 3; % Points per parameter
    param_grids = cell(1, 6);
    for i = 1:6
        param_grids{i} = linspace(param_bounds(i,1), param_bounds(i,2), n_points);
    end
    
    best_score = inf;
    best_params = [];
    evaluation_count = 0;
    
    % Generate all combinations
    [Q_grid, R_gps_grid, R_baro_grid, R_mag_grid, Q_att_grid, P_grid] = ndgrid(...
        param_grids{1}, param_grids{2}, param_grids{3}, param_grids{4}, param_grids{5}, param_grids{6});
    
    total_combinations = numel(Q_grid);
    if verbose
        fprintf('Testing %d parameter combinations...\n', total_combinations);
    end
    
    for i = 1:total_combinations
        if evaluation_count >= max_eval
            break;
        end
        
        params = [Q_grid(i), R_gps_grid(i), R_baro_grid(i), R_mag_grid(i), Q_att_grid(i), P_grid(i)];
        score = objective_function(params);
        evaluation_count = evaluation_count + 1;
        
        if score < best_score
            best_score = score;
            best_params = params;
        end
        
        if verbose && mod(i, 10) == 0
            fprintf('Progress: %d/%d, Best score: %.3f\n', i, total_combinations, best_score);
        end
    end
end

%% FMINCON Implementation
function best_params = optimize_fmincon()
    if verbose
        fprintf('Running fmincon optimization...\n');
    end
    
    % Initial guess (center of parameter space)
    x0 = mean(param_bounds, 2);
    
    % Lower and upper bounds
    lb = param_bounds(:,1);
    ub = param_bounds(:,2);
    
    % Options
    options = optimoptions('fmincon', ...
        'Display', 'iter', ...
        'MaxFunctionEvaluations', max_eval, ...
        'OptimalityTolerance', 1e-3, ...
        'StepTolerance', 1e-6);
    
    % Run optimization
    [best_params, best_score, exitflag] = fmincon(@objective_function, x0, [], [], [], [], lb, ub, [], options);
    
    if verbose
        fprintf('Optimization completed. Exit flag: %d, Best score: %.3f\n', exitflag, best_score);
    end
end

%% Random Search Implementation
function best_params = optimize_random_search()
    if verbose
        fprintf('Running random search optimization...\n');
    end
    
    best_score = inf;
    best_params = [];
    
    for i = 1:max_eval
        % Generate random parameters within bounds
        params = zeros(1, 6);
        for j = 1:6
            params(j) = param_bounds(j,1) + rand() * (param_bounds(j,2) - param_bounds(j,1));
        end
        
        score = objective_function(params);
        
        if score < best_score
            best_score = score;
            best_params = params;
        end
        
        if verbose && mod(i, 10) == 0
            fprintf('Evaluation %d/%d, Best score: %.3f\n', i, max_eval, best_score);
        end
    end
end

%% Display final results
if verbose
    fprintf('\n=== OPTIMIZATION RESULTS ===\n');
    fprintf('Best parameters found:\n');
    for i = 1:length(param_names)
        fprintf('  %s: %.3f\n', param_names{i}, best_params(i));
    end
    
    % Run final test with best parameters
    fprintf('\nRunning final validation test...\n');
    final_results = rapid_test_ekf('duration', duration, ...
        'Q_scale', best_params(1), ...
        'R_gps_scale', best_params(2), ...
        'R_baro_scale', best_params(3), ...
        'R_mag_scale', best_params(4), ...
        'Q_att_scale', best_params(5), ...
        'P_scale', best_params(6), ...
        'verbose', true);
    
    fprintf('\nFinal Performance:\n');
    fprintf('  Position RMSE: %.3f m\n', final_results.pos_rmse);
    fprintf('  Velocity RMSE: %.3f m/s\n', final_results.vel_rmse);
    fprintf('  Attitude RMSE: %.2f deg\n', final_results.att_rmse_deg);
    fprintf('  Overall Score: %.3f\n', final_results.performance_score);
end

end
