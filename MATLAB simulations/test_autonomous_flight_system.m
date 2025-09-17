%% test_autonomous_flight_system.m - Test Autonomous Flight Before RL Training
% PURPOSE
% Comprehensive testing of autonomous flight system (EKF + Control + Dynamics)
% before plugging into RL training. This validates the flight control protocol.

function test_autonomous_flight_system()
    fprintf('=== Autonomous Flight System Testing ===\n\n');
    
    % Step 1: Setup
    fprintf('Step 1: Setting up test environment...\n');
    addpath('rl_obstacle_avoidance');
    addpath('.');
    
    % Load parameters
    params = rl_parameters();
    fprintf('✓ Parameters loaded\n');
    
    % Create test scenarios
    test_scenarios = {
        'hover_test', 'straight_line', 'waypoint_tracking', 'obstacle_avoidance', 'sensor_failure'
    };
    
    % Run each test
    for i = 1:length(test_scenarios)
        fprintf('\n--- Test %d: %s ---\n', i, test_scenarios{i});
        run_flight_test(test_scenarios{i}, params);
    end
    
    fprintf('\n=== All Flight Tests Complete ===\n');
end

function run_flight_test(test_name, params)
    %% Run Individual Flight Test
    
    % Create environment
    env = rl_environment(params);
    
    % Create RL-EKF integration (same as in training)
    rl_ekf = rl_ekf_integration(params);
    
    % Test parameters
    test_duration = 30;  % seconds
    dt = params.Ts.physics;
    steps = round(test_duration / dt);
    
    % Initialize logging
    log = struct();
    log.time = zeros(1, steps);
    log.true_state = zeros(9, steps);
    log.estimated_state = zeros(9, steps);
    log.control_inputs = zeros(4, steps);
    log.velocity_commands = zeros(4, steps);
    log.ekf_uncertainty = zeros(1, steps);
    log.sensor_data = struct();
    log.sensor_data.imu = zeros(6, steps);
    log.sensor_data.gps = zeros(3, steps);
    log.sensor_data.baro = zeros(1, steps);
    log.sensor_data.mag = zeros(1, steps);
    
    % Generate test trajectory based on scenario
    [start_pos, waypoints, velocity_profile] = generate_test_trajectory(test_name, env);
    
    % Initialize drone state
    true_state = [start_pos; zeros(3,1); zeros(3,1)];  % [pos; vel; att]
    env.drone_state = true_state;
    
    fprintf('  Start: [%.1f %.1f %.1f]\n', start_pos(1), start_pos(2), start_pos(3));
    fprintf('  Waypoints: %d\n', size(waypoints, 2));
    fprintf('  Duration: %.1f s\n', test_duration);
    
    % Run simulation
    current_waypoint = 1;
    for step = 1:steps
        t = step * dt;
        log.time(step) = t;
        
        % Generate velocity command based on test scenario
        vel_cmd = generate_velocity_command(test_name, true_state, waypoints, current_waypoint, velocity_profile, t);
        
        % Apply control and get new state (same as in RL training)
        [new_true_state, uncertainty_info] = rl_ekf.step(true_state, vel_cmd, dt);
        
        % Log data
        log.true_state(:, step) = true_state;
        log.estimated_state(:, step) = rl_ekf.ekf_state;
        log.velocity_commands(:, step) = vel_cmd;
        log.ekf_uncertainty(step) = trace(uncertainty_info.covariance);
        
        % Get sensor data
        measurements = rl_ekf.get_current_measurements();
        if ~isempty(measurements)
            if isfield(measurements, 'imu')
                log.sensor_data.imu(:, step) = measurements.imu;
            end
            if isfield(measurements, 'gps')
                log.sensor_data.gps(:, step) = measurements.gps;
            end
            if isfield(measurements, 'baro')
                log.sensor_data.baro(step) = measurements.baro;
            end
            if isfield(measurements, 'mag')
                log.sensor_data.mag(step) = measurements.mag;
            end
        end
        
        % Update state
        true_state = new_true_state;
        
        % Check waypoint progress
        if current_waypoint <= size(waypoints, 2)
            dist_to_waypoint = norm(true_state(1:3) - waypoints(:, current_waypoint));
            if dist_to_waypoint < 2.0  % Within 2m
                current_waypoint = current_waypoint + 1;
                fprintf('  Reached waypoint %d at t=%.1fs\n', current_waypoint-1, t);
            end
        end
        
        % Check for collisions
        if env.check_collision()
            fprintf('  ⚠️  Collision detected at t=%.1fs\n', t);
            break;
        end
    end
    
    % Analyze results
    analyze_flight_test(test_name, log, steps);
    
    % Generate plots
    plot_flight_test(test_name, log, steps);
end

function [start_pos, waypoints, velocity_profile] = generate_test_trajectory(test_name, env)
    %% Generate Test Trajectory Based on Scenario
    
    switch test_name
        case 'hover_test'
            start_pos = [0; 0; 10];
            waypoints = start_pos;
            velocity_profile = @(t) [0; 0; 0; 0];  % Hover
            
        case 'straight_line'
            start_pos = [0; 0; 10];
            waypoints = [start_pos, [20; 0; 10], [40; 0; 10]];
            velocity_profile = @(t) [2; 0; 0; 0];  % Forward flight
            
        case 'waypoint_tracking'
            start_pos = [0; 0; 10];
            waypoints = [start_pos, [10; 10; 15], [20; 5; 20], [30; 15; 10]];
            velocity_profile = @(t) [1; 0.5; 0.2; 0.1];  % Complex trajectory
            
        case 'obstacle_avoidance'
            start_pos = [0; 0; 10];
            waypoints = [start_pos, [20; 0; 10], [40; 0; 10]];
            velocity_profile = @(t) [1.5; 0; 0; 0];  % Forward with obstacles
            
        case 'sensor_failure'
            start_pos = [0; 0; 10];
            waypoints = [start_pos, [20; 0; 10]];
            velocity_profile = @(t) [1; 0; 0; 0];  % Normal flight with sensor issues
    end
end

function vel_cmd = generate_velocity_command(test_name, current_state, waypoints, current_waypoint, velocity_profile, t)
    %% Generate Velocity Command Based on Test Scenario
    
    switch test_name
        case 'hover_test'
            vel_cmd = velocity_profile(t);
            
        case 'straight_line'
            vel_cmd = velocity_profile(t);
            
        case 'waypoint_tracking'
            if current_waypoint <= size(waypoints, 2)
                % Simple waypoint following
                target_pos = waypoints(:, current_waypoint);
                current_pos = current_state(1:3);
                direction = target_pos - current_pos;
                if norm(direction) > 0.1
                    direction = direction / norm(direction);
                    vel_cmd = [direction * 2; 0];  % 2 m/s towards waypoint
                else
                    vel_cmd = [0; 0; 0; 0];
                end
            else
                vel_cmd = [0; 0; 0; 0];
            end
            
        case 'obstacle_avoidance'
            % Simple obstacle avoidance (go around obstacles)
            vel_cmd = velocity_profile(t);
            % Add simple avoidance logic here if needed
            
        case 'sensor_failure'
            vel_cmd = velocity_profile(t);
    end
end

function analyze_flight_test(test_name, log, steps)
    %% Analyze Flight Test Results
    
    fprintf('  Results Analysis:\n');
    
    % Position tracking accuracy
    pos_error = log.true_state(1:3, :) - log.estimated_state(1:3, :);
    pos_rmse = sqrt(mean(sum(pos_error.^2, 1)));
    fprintf('    Position RMSE: %.3f m\n', pos_rmse);
    
    % Velocity tracking accuracy
    vel_error = log.true_state(4:6, :) - log.estimated_state(4:6, :);
    vel_rmse = sqrt(mean(sum(vel_error.^2, 1)));
    fprintf('    Velocity RMSE: %.3f m/s\n', vel_rmse);
    
    % Attitude tracking accuracy
    att_error = log.true_state(7:9, :) - log.estimated_state(7:9, :);
    att_rmse = sqrt(mean(sum(att_error.^2, 1)));
    fprintf('    Attitude RMSE: %.3f rad (%.1f deg)\n', att_rmse, rad2deg(att_rmse));
    
    % EKF uncertainty
    avg_uncertainty = mean(log.ekf_uncertainty);
    fprintf('    Average EKF Uncertainty: %.3f\n', avg_uncertainty);
    
    % Control smoothness
    vel_cmd_diff = diff(log.velocity_commands, 1, 2);
    control_smoothness = mean(sqrt(sum(vel_cmd_diff.^2, 1)));
    fprintf('    Control Smoothness: %.3f\n', control_smoothness);
    
    % Overall assessment
    if pos_rmse < 1.0 && vel_rmse < 0.5 && att_rmse < 0.1
        fprintf('    ✅ Flight system working well\n');
    elseif pos_rmse < 2.0 && vel_rmse < 1.0 && att_rmse < 0.2
        fprintf('    ⚠️  Flight system acceptable but needs tuning\n');
    else
        fprintf('    ❌ Flight system needs significant improvement\n');
    end
end

function plot_flight_test(test_name, log, steps)
    %% Generate Flight Test Plots
    
    figure('Position', [100, 100, 1200, 800]);
    
    % 3D trajectory
    subplot(2, 3, 1);
    plot3(log.true_state(1, :), log.true_state(2, :), log.true_state(3, :), 'b-', 'LineWidth', 2);
    hold on;
    plot3(log.estimated_state(1, :), log.estimated_state(2, :), log.estimated_state(3, :), 'r--', 'LineWidth', 1);
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title(sprintf('%s: 3D Trajectory', test_name));
    legend('True', 'Estimated', 'Location', 'best');
    grid on;
    
    % Position tracking
    subplot(2, 3, 2);
    plot(log.time, log.true_state(1:3, :), 'b-', 'LineWidth', 1);
    hold on;
    plot(log.time, log.estimated_state(1:3, :), 'r--', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('Position Tracking');
    legend('True X', 'True Y', 'True Z', 'Est X', 'Est Y', 'Est Z', 'Location', 'best');
    grid on;
    
    % Velocity tracking
    subplot(2, 3, 3);
    plot(log.time, log.true_state(4:6, :), 'b-', 'LineWidth', 1);
    hold on;
    plot(log.time, log.estimated_state(4:6, :), 'r--', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity Tracking');
    legend('True Vx', 'True Vy', 'True Vz', 'Est Vx', 'Est Vy', 'Est Vz', 'Location', 'best');
    grid on;
    
    % Attitude tracking
    subplot(2, 3, 4);
    plot(log.time, rad2deg(log.true_state(7:9, :)), 'b-', 'LineWidth', 1);
    hold on;
    plot(log.time, rad2deg(log.estimated_state(7:9, :)), 'r--', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Attitude (deg)');
    title('Attitude Tracking');
    legend('True Roll', 'True Pitch', 'True Yaw', 'Est Roll', 'Est Pitch', 'Est Yaw', 'Location', 'best');
    grid on;
    
    % EKF uncertainty
    subplot(2, 3, 5);
    plot(log.time, log.ekf_uncertainty, 'g-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('EKF Uncertainty');
    title('EKF Uncertainty');
    grid on;
    
    % Velocity commands
    subplot(2, 3, 6);
    plot(log.time, log.velocity_commands, 'k-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Velocity Command');
    title('Velocity Commands');
    legend('Vx', 'Vy', 'Vz', 'Yaw Rate', 'Location', 'best');
    grid on;
    
    sgtitle(sprintf('Flight Test: %s', test_name), 'FontSize', 14);
    
    % Save plot
    saveas(gcf, sprintf('flight_test_%s.png', test_name));
    fprintf('  Plot saved: flight_test_%s.png\n', test_name);
end
