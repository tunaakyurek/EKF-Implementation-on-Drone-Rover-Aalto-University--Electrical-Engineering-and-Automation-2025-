%% test_tuned_expert.m - Tuned Expert System Test
% PURPOSE
% Test the expert flight controller with properly tuned control parameters.

function test_tuned_expert()
    fprintf('=== Tuned Expert System Test ===\n\n');
    
    % Setup
    addpath('rl_obstacle_avoidance');
    addpath('.');
    
    % Load parameters
    run('parameters.m');
    params = rl_parameters();
    
    % Create expert flight controller with tuned parameters
    expert_controller = expert_flight_controller(params);
    
    % Tune control parameters for stability
    expert_controller.max_velocity = 0.8;        % Reduced from 2.0
    expert_controller.max_acceleration = 0.5;    % Reduced from 1.5
    expert_controller.lookahead_distance = 2.0;  % Reduced from 3.0
    
    % Generate simple obstacle map
    map_gen = expert_map_generator();
    obstacle_map = map_gen.generate_column_obstacles(0.1);  % 10% density
    
    % Define start and goal positions
    start_pos = [-8; -8; 5];
    goal_pos = [8; 8; 5];
    
    % Test parameters
    duration = 40;  % seconds
    dt = params.Ts.physics;
    steps = round(duration / dt);
    
    % Initialize expert system
    expert_controller.set_obstacle_map(obstacle_map);
    expert_controller.set_start_goal(start_pos, goal_pos);
    
    % Initial state
    true_state = [start_pos; zeros(6, 1)];  % [pos; vel; att]
    
    % Logging
    time = zeros(1, steps);
    true_pos = zeros(3, steps);
    expert_pos = zeros(3, steps);
    control_inputs = zeros(4, steps);
    
    fprintf('Testing tuned expert system for %.1f seconds...\n', duration);
    fprintf('Start: [%.1f %.1f %.1f], Goal: [%.1f %.1f %.1f]\n', ...
            start_pos(1), start_pos(2), start_pos(3), ...
            goal_pos(1), goal_pos(2), goal_pos(3));
    
    for step = 1:steps
        t = step * dt;
        time(step) = t;
        
        % Expert path planning and control with tuned gains
        [expert_state, control_inputs(:, step), path_info] = compute_tuned_expert_control(true_state, expert_controller, goal_pos, t);
        
        % Log expert state
        expert_pos(:, step) = expert_state(1:3);
        
        % Log true state
        true_pos(:, step) = true_state(1:3);
        
        % Working dynamics simulation
        true_state = simulate_working_dynamics(true_state, control_inputs(:, step), dt, params);
        
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
        fprintf('  ✅ Tuned expert: SUCCESS\n');
    elseif final_error < 10.0
        fprintf('  ⚠️  Tuned expert: PARTIAL SUCCESS\n');
    else
        fprintf('  ❌ Tuned expert: NEEDS IMPROVEMENT\n');
    end
    
    % Plot results
    plot_tuned_results(time, true_pos, expert_pos, obstacle_map, start_pos, goal_pos, control_inputs);
end

function [expert_state, control_input, path_info] = compute_tuned_expert_control(current_state, expert_controller, goal_pos, current_time)
    %% Tuned Expert Control with Conservative Gains
    
    current_pos = current_state(1:3);
    current_vel = current_state(4:6);
    current_att = current_state(7:9);
    
    % Get planned path from expert controller
    if ~expert_controller.path_planned
        expert_controller.plan_path();
    end
    
    % If no path available, use simple goal seeking
    if isempty(expert_controller.current_path)
        [expert_state, control_input, path_info] = expert_controller.simple_goal_seeking(current_state);
        return;
    end
    
    % Find current target waypoint
    if expert_controller.path_index <= size(expert_controller.current_path, 2)
        target_pos = expert_controller.current_path(:, expert_controller.path_index);
    else
        target_pos = goal_pos;
    end
    
    % Move to next waypoint if close enough
    distance_to_waypoint = norm(current_pos - target_pos);
    if distance_to_waypoint < 2.0 && expert_controller.path_index < size(expert_controller.current_path, 2)
        expert_controller.path_index = expert_controller.path_index + 1;
        if expert_controller.path_index <= size(expert_controller.current_path, 2)
            target_pos = expert_controller.current_path(:, expert_controller.path_index);
        else
            target_pos = goal_pos;
        end
    end
    
    % Compute direction to target
    direction = target_pos - current_pos;
    distance = norm(direction);
    
    if distance > 1e-6
        direction = direction / distance;
    else
        direction = [0; 0; 0];
    end
    
    % Conservative velocity control
    max_vel = expert_controller.max_velocity;
    if distance > 3.0
        desired_vel = direction * max_vel;
    elseif distance > 1.0
        desired_vel = direction * max_vel * (distance / 3.0);
    else
        desired_vel = direction * max_vel * 0.2;
    end
    
    % Conservative acceleration control
    vel_error = desired_vel - current_vel;
    kp = 1.0;  % Reduced from 3.0
    desired_accel = vel_error * kp;
    
    % Add gravity compensation
    gravity = [0; 0; 9.819];  % Gravity
    desired_accel = desired_accel + gravity;
    
    % Convert to thrust and attitude commands
    thrust_magnitude = norm(desired_accel) * 0.5;  % Mass scaling
    thrust_magnitude = max(min(thrust_magnitude, 15.0), 5.0);  % 5-15 N range
    
    % Conservative attitude control
    target_att = [0; 0; 0];  % Level flight
    att_error = target_att - current_att;
    kp_att = [1.0, 1.0, 0.5];  % Reduced gains
    torque = kp_att' .* att_error;
    
    % Limit torque
    max_torque = 0.1;  % Reduced max torque
    torque = max(min(torque, max_torque), -max_torque);
    
    % Generate control input
    control_input = [thrust_magnitude; torque];
    
    % Generate expert state
    expert_state = [target_pos; desired_vel; target_att];
    
    % Generate path info
    path_info = struct();
    path_info.target_pos = target_pos;
    path_info.desired_vel = desired_vel;
    path_info.desired_accel = desired_accel;
    path_info.path_index = expert_controller.path_index;
    path_info.distance_to_goal = distance;
end

function true_state = simulate_working_dynamics(current_state, control_inputs, dt, params)
    %% Working Dynamics Simulation
    
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
    thrust_body = [0; 0; -thrust];  % Thrust in body frame (upward, so negative Z in NED)
    thrust_world = R * thrust_body;  % Convert to world frame
    
    % Total acceleration
    accel = gravity + thrust_world / params.mass;
    
    % Damping
    damping = -0.1 * vel;
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
    att_damping = -0.1 * att;
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

function plot_tuned_results(time, true_pos, expert_pos, obstacle_map, start_pos, goal_pos, control_inputs)
    %% Plot Tuned Results
    
    figure('Position', [100, 100, 1400, 1000]);
    
    subplot(3, 3, 1);
    plot3(true_pos(1, :), true_pos(2, :), true_pos(3, :), 'b-', 'LineWidth', 2);
    hold on;
    plot3(expert_pos(1, :), expert_pos(2, :), expert_pos(3, :), 'r--', 'LineWidth', 1);
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
    legend('True Path', 'Expert Path', 'Start', 'Goal', 'Location', 'best');
    grid on;
    
    subplot(3, 3, 2);
    plot(time, true_pos, 'b-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('Position vs Time');
    legend('X', 'Y', 'Z', 'Location', 'best');
    grid on;
    
    subplot(3, 3, 3);
    plot(time, expert_pos, 'r-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Expert Position (m)');
    title('Expert Position vs Time');
    legend('X', 'Y', 'Z', 'Location', 'best');
    grid on;
    
    subplot(3, 3, 4);
    plot(time, control_inputs(1, :), 'g-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Thrust (N)');
    title('Thrust Command');
    grid on;
    
    subplot(3, 3, 5);
    plot(time, control_inputs(2, :), 'r-', 'LineWidth', 1);
    hold on;
    plot(time, control_inputs(3, :), 'g-', 'LineWidth', 1);
    plot(time, control_inputs(4, :), 'b-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Torque (Nm)');
    title('Torque Commands');
    legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');
    grid on;
    
    subplot(3, 3, 6);
    goal_distance = sqrt(sum((true_pos - goal_pos).^2, 1));
    plot(time, goal_distance, 'k-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Distance to Goal (m)');
    title('Goal Distance');
    grid on;
    
    subplot(3, 3, 7);
    velocity_magnitude = sqrt(sum(diff(true_pos, 1, 2).^2, 1)) / (time(2) - time(1));
    plot(time(2:end), velocity_magnitude, 'm-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity Magnitude');
    grid on;
    
    subplot(3, 3, 8);
    plot(time, sqrt(sum(true_pos.^2, 1)), 'b-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Distance from Origin (m)');
    title('Path Length');
    grid on;
    
    subplot(3, 3, 9);
    plot(true_pos(1, :), true_pos(2, :), 'b-', 'LineWidth', 2);
    hold on;
    plot(expert_pos(1, :), expert_pos(2, :), 'r--', 'LineWidth', 1);
    plot(start_pos(1), start_pos(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(goal_pos(1), goal_pos(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    xlabel('X (m)');
    ylabel('Y (m)');
    title('Top View Trajectory');
    legend('True Path', 'Expert Path', 'Start', 'Goal', 'Location', 'best');
    grid on;
    
    sgtitle('Tuned Expert System Test', 'FontSize', 14);
    saveas(gcf, 'tuned_expert_test.png');
    fprintf('Plot saved: tuned_expert_test.png\n');
end
