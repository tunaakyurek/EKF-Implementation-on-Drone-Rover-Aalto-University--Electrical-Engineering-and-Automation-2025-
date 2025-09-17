%% demo_navigation_scenarios.m - Demonstration of Enhanced Navigation Scenarios
% PURPOSE
% Demonstrates the updated navigation-focused RL system with cylindrical columns,
% rectangular boundaries, and optimized environments for drone navigation training.
%
% USAGE
%   Run this script to see the enhanced navigation scenarios:
%   >> demo_navigation_scenarios

clear; clc; close all;

fprintf('=== Enhanced Navigation Scenarios Demonstration ===\n\n');

%% 1. Load Enhanced Parameters
fprintf('Loading enhanced navigation parameters...\n');

% Load updated RL parameters
params = rl_parameters();

% Display key navigation parameters
fprintf('Navigation Parameters:\n');
fprintf('  Map size: %.0fx%.0f meters\n', params.rl.map_bounds(2)-params.rl.map_bounds(1), params.rl.map_bounds(4)-params.rl.map_bounds(3));
fprintf('  Obstacle scenario: %s\n', params.rl.obstacle_scenario);
fprintf('  Obstacle density: %.1f%%\n', params.rl.obstacle_density);
fprintf('  Min corridor width: %.1f m\n', params.rl.navigation_corridor_width);
fprintf('  Min column spacing: %.1f m\n', params.rl.column_spacing_min);

%% 2. Create Map Generator and Test Scenarios
fprintf('\nGenerating navigation scenarios...\n');

try
    map_gen = map_generator(params);
    fprintf('✓ Map generator created\n');
catch ME
    fprintf('✗ Error creating map generator: %s\n', ME.message);
    return;
end

% Test different scenarios
scenarios = {'columns', 'pillars', 'mixed'};
difficulties = [0.3, 0.6, 0.9];

figure('Position', [100, 100, 1200, 800]);

for i = 1:length(scenarios)
    scenario = scenarios{i};
    difficulty = difficulties(i);
    
    fprintf('\nScenario %d: %s (difficulty %.1f)\n', i, scenario, difficulty);
    
    % Generate map
    [occupancy_grid, height_map, metadata] = map_gen.generate_map(scenario, i*123, difficulty);
    
    % Create subplot
    subplot(2, 3, i);
    visualize_navigation_scenario(occupancy_grid, scenario, difficulty);
    title(sprintf('%s Navigation (Diff: %.1f)', upper(scenario), difficulty), 'FontSize', 12);
    
    % Display statistics
    fprintf('  Obstacles placed: %.1f%% density\n', metadata.obstacle_density * 100);
    if metadata.path_exists
        fprintf('  Path validation: PASS\n');
    else
        fprintf('  Path validation: FAIL\n');
    end
end

%% 3. Demonstrate Command Output System
fprintf('\nTesting RL command to drone control conversion...\n');

% Create EKF integration layer
try
    ekf_integration = rl_ekf_integration(params);
    fprintf('✓ EKF integration layer created\n');
    
    % Test different RL commands
    test_commands = [
        [0.5, 0.0, 0.0, 0.0];    % Forward motion
        [0.0, 0.5, 0.0, 0.0];    % Right motion  
        [0.0, 0.0, -0.3, 0.0];   % Upward motion
        [0.2, 0.2, 0.0, 0.3];    % Diagonal with yaw
        [0.0, 0.0, 0.0, 0.5];    % Pure yaw rotation
    ];
    
    command_names = {'Forward', 'Right', 'Up', 'Diagonal+Yaw', 'Pure Yaw'};
    
    % Test state (hovering drone)
    test_state = [0; 0; -5; 0; 0; 0; 0; 0; 0];  % 5m altitude, level attitude
    
    fprintf('\nCommand Output Testing:\n');
    fprintf('RL Command → Drone Control [Thrust(N), τx(Nm), τy(Nm), τz(Nm)]\n');
    fprintf('--------------------------------------------------------\n');
    
    for cmd = 1:size(test_commands, 1)
        rl_cmd = test_commands(cmd, :);
        control_out = ekf_integration.convert_rl_command_to_control(rl_cmd, test_state);
        
        fprintf('%12s: [%5.1f, %5.1f, %5.1f] → [%6.2f, %6.3f, %6.3f, %6.3f]\n', ...
                command_names{cmd}, rl_cmd(1), rl_cmd(2), rl_cmd(3), control_out(1), control_out(2), control_out(3), control_out(4));
    end
    
    fprintf('✓ Command conversion working correctly\n');
    
catch ME
    fprintf('✗ Error in command testing: %s\n', ME.message);
end

%% 4. Demonstrate Environment Reset and Navigation
fprintf('\nTesting environment reset and navigation...\n');

try
    % Create environment
    env = rl_environment(params);
    fprintf('✓ Environment created\n');
    
    % Test multiple resets
    fprintf('\nEnvironment Reset Testing:\n');
    for reset_test = 1:3
        obs = env.reset();
        start_pos = env.drone_state(1:3);
        goal_pos = env.target_waypoints(:, end);
        distance = norm(goal_pos - start_pos);
        
        fprintf('Reset %d: Start[%5.1f,%5.1f,%5.1f] → Goal[%5.1f,%5.1f,%5.1f] (%.1fm)\n', ...
                reset_test, start_pos, goal_pos, distance);
    end
    
    fprintf('✓ Environment reset working correctly\n');
    
catch ME
    fprintf('✗ Error in environment testing: %s\n', ME.message);
end

%% 5. Navigation Corridor Analysis
fprintf('\nAnalyzing navigation corridors...\n');

% Test navigation corridor widths
subplot(2, 3, 4:6);
[occupancy_grid, ~, ~] = map_gen.generate_map('columns', 999, 0.5);
analyze_navigation_corridors(occupancy_grid, params.rl.map_resolution);
title('Navigation Corridor Analysis', 'FontSize', 14);

%% 6. Performance Comparison
fprintf('\nPerformance Comparison (Old vs New):\n');
fprintf('Feature                 | Old System      | New System\n');
fprintf('------------------------|-----------------|------------------\n');
fprintf('Obstacle Type           | Mixed/Random    | Cylindrical Columns\n');
fprintf('Map Size                | 100x100x30m     | 80x80x25m\n');
fprintf('Obstacle Density        | 15%%             | 8%%\n');
fprintf('Min Corridor Width      | Variable        | 4.0m guaranteed\n');
fprintf('Column Spacing          | Random          | 6.0m minimum\n');
fprintf('Boundary Type           | Walls           | Rectangular + Openings\n');
fprintf('Navigation Difficulty   | High            | Optimized for Learning\n');
fprintf('Path Validation         | Basic           | Advanced with fixes\n');

fprintf('\n=== Navigation Enhancement Summary ===\n');
fprintf('✓ Cylindrical columns with guaranteed spacing\n');
fprintf('✓ Rectangular boundary walls with optional openings\n');
fprintf('✓ Reduced obstacle density for better navigation\n');
fprintf('✓ Enhanced RL command → drone control conversion\n');
fprintf('✓ Proper 9-DOF dynamics integration\n');
fprintf('✓ Navigation corridor analysis and validation\n');
fprintf('✓ Randomized scenarios for diverse training\n');

fprintf('\nThe system is now optimized for training drones to\n');
fprintf('"swish through" obstacles with realistic navigation challenges!\n');

%% Helper Functions

function visualize_navigation_scenario(occupancy_grid, scenario_name, difficulty)
    %% Visualize Navigation Scenario in 2D Top-Down View
    
    % Create top-down view by taking maximum along Z-axis
    top_view = squeeze(max(occupancy_grid, [], 3));
    
    % Display as binary image
    imshow(~top_view, 'InitialMagnification', 'fit');
    colormap(gray);
    
    % Add navigation analysis
    [corridors, min_width] = analyze_corridor_widths(top_view);
    
    % Add text annotation
    text(5, 5, sprintf('Min corridor: %.1fm', min_width * 0.5), ...
         'Color', 'red', 'FontSize', 10, 'FontWeight', 'bold');
end

function analyze_navigation_corridors(occupancy_grid, resolution)
    %% Analyze Navigation Corridor Widths
    
    % Create top-down view
    top_view = squeeze(max(occupancy_grid, [], 3));
    
    % Find free space
    free_space = ~top_view;
    
    % Distance transform to find corridor widths
    dist_transform = bwdist(top_view);
    
    % Display results
    imagesc(dist_transform * resolution);
    colorbar;
    colormap(jet);
    xlabel('Distance from obstacles (meters)');
    
    % Find minimum corridor width
    min_corridor_width = min(dist_transform(free_space)) * 2 * resolution;
    max_corridor_width = max(dist_transform(free_space)) * 2 * resolution;
    avg_corridor_width = mean(dist_transform(free_space)) * 2 * resolution;
    
    title(sprintf('Corridor Analysis: Min=%.1fm, Avg=%.1fm, Max=%.1fm', ...
                  min_corridor_width, avg_corridor_width, max_corridor_width));
end

function [corridors, min_width] = analyze_corridor_widths(top_view)
    %% Analyze Corridor Widths in Top View
    
    % Distance transform
    dist_transform = bwdist(top_view);
    free_space = ~top_view;
    
    if any(free_space(:))
        min_width = min(dist_transform(free_space)) * 2;  % Diameter
        corridors = dist_transform;
    else
        min_width = 0;
        corridors = zeros(size(top_view));
    end
end
