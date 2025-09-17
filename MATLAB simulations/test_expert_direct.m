%% test_expert_direct.m - Direct Expert System Test
% PURPOSE
% Test a direct path-following approach that actually moves toward the goal.

function test_expert_direct()
    fprintf('=== Direct Expert System Test ===\n\n');
    
    % Setup
    addpath('rl_obstacle_avoidance');
    addpath('.');
    
    % Load parameters
    run('parameters.m');
    params = rl_parameters();
    
    % Generate simple obstacle map
    map_gen = expert_map_generator();
    obstacle_map = map_gen.generate_column_obstacles(0.1);  % 10% density
    
    % Define start and goal positions
    start_pos = [-10; -10; 5];
    goal_pos = [10; 10; 5];
    
    % Test parameters
    duration = 20;  % seconds
    dt = params.Ts.physics;
    steps = round(duration / dt);
    
    % Initial state
    true_state = [start_pos; zeros(6, 1)];  % [pos; vel; att]
    
    % Logging
    time = zeros(1, steps);
    true_pos = zeros(3, steps);
    control_inputs = zeros(4, steps);
    
    fprintf('Testing direct path following for %.1f seconds...\n', duration);
    fprintf('Start: [%.1f %.1f %.1f], Goal: [%.1f %.1f %.1f]\n', ...
            start_pos(1), start_pos(2), start_pos(3), ...
            goal_pos(1), goal_pos(2), goal_pos(3));
    
    for step = 1:steps
        t = step * dt;
        time(step) = t;
        
        % Direct path following control
        [control_inputs(:, step), path_info] = compute_direct_control(true_state, goal_pos, obstacle_map);
        
        % Log true state
        true_pos(:, step) = true_state(1:3);
        
        % Simple dynamics simulation
        true_state = simulate_direct_dynamics(true_state, control_inputs(:, step), dt, params);
        
        % Check if goal reached
        goal_distance = norm(true_state(1:3) - goal_pos);
        if goal_distance < 2.0
            fprintf('Goal reached at t=%.1fs (distance: %.2fm)\n', t, goal_distance);
            break;
        end
        
        % Print progress every 5 seconds
        if mod(t, 5) < dt
            fprintf('  t=%.1fs: pos=[%.1f %.1f %.1f], goal_dist=%.1fm, thrust=%.1fN\n', ...
                    t, true_state(1), true_state(2), true_state(3), ...
                    goal_distance, control_inputs(1, step));
        end
        
        % Safety check
        if goal_distance > 50.0
            fprintf('Safety stop: drone flew too far (%.1fm)\n', goal_distance);
            break;
        end
    end
    
    % Analyze results
    final_pos = true_pos(:, step);
    final_error = norm(final_pos - goal_pos);
    
    fprintf('\nResults:\n');
    fprintf('  Final position: [%.1f %.1f %.1f]\n', final_pos(1), final_pos(2), final_pos(3));
    fprintf('  Goal position: [%.1f %.1f %.1f]\n', goal_pos(1), goal_pos(2), goal_pos(3));
    fprintf('  Final error: %.3f m\n', final_error);
    
    if final_error < 3.0
        fprintf('  ✅ Direct system: SUCCESS\n');
    elseif final_error < 10.0
        fprintf('  ⚠️  Direct system: PARTIAL SUCCESS\n');
    else
        fprintf('  ❌ Direct system: NEEDS IMPROVEMENT\n');
    end
    
    % Plot results
    plot_direct_results(time, true_pos, obstacle_map, start_pos, goal_pos, control_inputs);
end

function [control_input, path_info] = compute_direct_control(current_state, goal_pos, obstacle_map)
    %% Direct Path Following Control
    
    current_pos = current_state(1:3);
    current_vel = current_state(4:6);
    current_att = current_state(7:9);
    
    % Compute direction to goal
    direction = goal_pos - current_pos;
    distance = norm(direction);
    
    if distance > 1e-6
        direction = direction / distance;
    else
        direction = [0; 0; 0];
    end
    
    % Compute desired velocity (direct toward goal)
    max_vel = 1.0;  % m/s
    if distance > 5.0
        desired_vel = direction * max_vel;
    elseif distance > 2.0
        desired_vel = direction * max_vel * (distance / 5.0);
    else
        desired_vel = direction * max_vel * 0.3;
    end
    
    % Compute desired acceleration
    vel_error = desired_vel - current_vel;
    desired_accel = vel_error * 2.0;  % Simple proportional control
    
    % Add gravity compensation
    gravity = [0; 0; 9.81];  % Gravity
    desired_accel = desired_accel + gravity;
    
    % Convert to thrust and attitude commands
    thrust_magnitude = norm(desired_accel) * 0.5;  % Mass scaling
    thrust_magnitude = max(min(thrust_magnitude, 15.0), 5.0);  % 5-15 N range
    
    % Simple attitude control (level flight)
    desired_att = [0; 0; 0];  % Level flight
    att_error = desired_att - current_att;
    torque = att_error * 1.0;  % Simple proportional control
    
    % Generate control input
    control_input = [thrust_magnitude; torque];
    
    % Generate path info
    path_info = struct();
    path_info.target_pos = goal_pos;
    path_info.desired_vel = desired_vel;
    path_info.desired_accel = desired_accel;
    path_info.distance_to_goal = distance;
end

function true_state = simulate_direct_dynamics(current_state, control_inputs, dt, params)
    %% Direct Dynamics Simulation
    
    % Extract control inputs
    thrust = control_inputs(1);
    torque = control_inputs(2:4);
    
    % Extract current state
    pos = current_state(1:3);
    vel = current_state(4:6);
    att = current_state(7:9);
    
    % Position integration
    new_pos = pos + vel * dt;
    
    % Velocity dynamics
    gravity = [0; 0; params.g(3)];  % Gravity vector (positive down in NED)
    
    % Convert thrust to world frame
    R = rotation_matrix(att(1), att(2), att(3));
    thrust_body = [0; 0; -thrust];  % Thrust in body frame (upward)
    thrust_world = R * thrust_body;  % Convert to world frame
    
    % Total acceleration
    accel = gravity + thrust_world / params.mass;
    
    % Damping
    damping = -0.3 * vel;
    accel = accel + damping;
    
    new_vel = vel + accel * dt;
    
    % Attitude dynamics
    if length(torque) == 3 && length(params.I) == 3
        if isvector(params.I)
            I_diag = params.I;
        else
            I_diag = diag(params.I);
        end
        att_rate = torque ./ I_diag;
    else
        att_rate = torque / params.mass;
    end
    
    % Attitude damping
    att_damping = -0.2 * att;
    att_rate = att_rate + att_damping;
    
    new_att = att + att_rate * dt;
    
    % Wrap angles
    new_att(3) = wrapToPi(new_att(3));
    
    % Ensure all vectors are column vectors
    new_pos = new_pos(:);
    new_vel = new_vel(:);
    new_att = new_att(:);
    
    true_state = [new_pos; new_vel; new_att];
end

function plot_direct_results(time, true_pos, obstacle_map, start_pos, goal_pos, control_inputs)
    %% Plot Direct Results
    
    figure('Position', [100, 100, 1400, 1000]);
    
    subplot(3, 3, 1);
    plot3(true_pos(1, :), true_pos(2, :), true_pos(3, :), 'b-', 'LineWidth', 2);
    hold on;
    plot3(start_pos(1), start_pos(2), start_pos(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot3(goal_pos(1), goal_pos(2), goal_pos(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    
    % Plot obstacles
    for i = 1:length(obstacle_map.cylinders)
        cyl = obstacle_map.cylinders{i};
        [X, Y, Z] = cylinder(cyl.radius, 20);
        Z = Z * cyl.height + cyl.center(3);
        X = X + cyl.center(1);
        Y = Y + cyl.center(2);
        surf(X, Y, Z, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    end
    
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('3D Trajectory with Obstacles');
    legend('Path', 'Start', 'Goal', 'Location', 'best');
    grid on;
    
    subplot(3, 3, 2);
    plot(time, true_pos, 'b-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('Position vs Time');
    legend('X', 'Y', 'Z', 'Location', 'best');
    grid on;
    
    subplot(3, 3, 3);
    plot(time, control_inputs(1, :), 'g-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Thrust (N)');
    title('Thrust Command');
    grid on;
    
    subplot(3, 3, 4);
    plot(time, control_inputs(2, :), 'r-', 'LineWidth', 1);
    hold on;
    plot(time, control_inputs(3, :), 'g-', 'LineWidth', 1);
    plot(time, control_inputs(4, :), 'b-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Torque (Nm)');
    title('Torque Commands');
    legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');
    grid on;
    
    subplot(3, 3, 5);
    goal_distance = sqrt(sum((true_pos - goal_pos).^2, 1));
    plot(time, goal_distance, 'k-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Distance to Goal (m)');
    title('Goal Distance');
    grid on;
    
    subplot(3, 3, 6);
    velocity_magnitude = sqrt(sum(diff(true_pos, 1, 2).^2, 1)) / (time(2) - time(1));
    plot(time(2:end), velocity_magnitude, 'm-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity Magnitude');
    grid on;
    
    subplot(3, 3, 7);
    plot(time, sqrt(sum(true_pos.^2, 1)), 'b-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Distance from Origin (m)');
    title('Path Length');
    grid on;
    
    subplot(3, 3, 8);
    plot(time, true_pos(3, :), 'b-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Altitude (m)');
    title('Altitude vs Time');
    grid on;
    
    subplot(3, 3, 9);
    plot(true_pos(1, :), true_pos(2, :), 'b-', 'LineWidth', 2);
    hold on;
    plot(start_pos(1), start_pos(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(goal_pos(1), goal_pos(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    xlabel('X (m)');
    ylabel('Y (m)');
    title('Top View Trajectory');
    legend('Path', 'Start', 'Goal', 'Location', 'best');
    grid on;
    
    sgtitle('Direct Expert System Test', 'FontSize', 14);
    saveas(gcf, 'direct_expert_test.png');
    fprintf('Plot saved: direct_expert_test.png\n');
end
