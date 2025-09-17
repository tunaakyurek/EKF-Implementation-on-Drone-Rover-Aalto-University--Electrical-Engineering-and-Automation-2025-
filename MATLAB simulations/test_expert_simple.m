%% test_expert_simple.m - Simple Expert System Test
% PURPOSE
% Test the expert flight controller with simplified dynamics and basic obstacle avoidance.

function test_expert_simple()
    fprintf('=== Simple Expert System Test ===\n\n');
    
    % Setup
    addpath('rl_obstacle_avoidance');
    addpath('.');
    
    % Load parameters
    run('parameters.m');
    params = rl_parameters();
    
    % Create expert flight controller
    expert_controller = expert_flight_controller(params);
    
    % Generate simple obstacle map
    map_gen = expert_map_generator();
    obstacle_map = map_gen.generate_column_obstacles(0.2);  % 20% density
    
    % Define start and goal positions
    start_pos = [-20; -20; 10];  % Start position
    goal_pos = [20; 20; 10];     % Goal position
    
    % Test parameters
    duration = 20;  % seconds
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
    
    fprintf('Testing expert obstacle avoidance for %.1f seconds...\n', duration);
    
    for step = 1:steps
        t = step * dt;
        time(step) = t;
        
        % Expert path planning and control
        [expert_state, control_inputs(:, step), path_info] = expert_controller.compute_expert_control(true_state, t);
        
        % Log expert state
        expert_pos(:, step) = expert_state(1:3);
        
        % Log true state
        true_pos(:, step) = true_state(1:3);
        
        % Simple dynamics simulation (hover mode)
        true_state = simulate_simple_dynamics(true_state, control_inputs(:, step), dt, params);
        
        % Check if goal reached
        if norm(true_state(1:3) - goal_pos) < 3.0
            fprintf('Goal reached at t=%.1fs\n', t);
            break;
        end
        
        % Print progress every 5 seconds
        if mod(t, 5) < dt
            fprintf('  t=%.1fs: pos=[%.1f %.1f %.1f], goal_dist=%.1fm\n', ...
                    t, true_state(1), true_state(2), true_state(3), ...
                    norm(true_state(1:3) - goal_pos));
        end
    end
    
    % Analyze results
    final_pos = true_pos(:, step);
    final_error = norm(final_pos - goal_pos);
    
    fprintf('\nResults:\n');
    fprintf('  Final position: [%.1f %.1f %.1f]\n', final_pos(1), final_pos(2), final_pos(3));
    fprintf('  Goal position: [%.1f %.1f %.1f]\n', goal_pos(1), goal_pos(2), goal_pos(3));
    fprintf('  Final error: %.3f m\n', final_error);
    
    if final_error < 5.0
        fprintf('  ✅ Expert system: SUCCESS\n');
    else
        fprintf('  ⚠️  Expert system: NEEDS IMPROVEMENT\n');
    end
    
    % Plot results
    plot_simple_results(time, true_pos, expert_pos, obstacle_map, start_pos, goal_pos);
end

function true_state = simulate_simple_dynamics(current_state, control_inputs, dt, params)
    %% Simple Drone Dynamics Simulation (Improved)
    
    % Extract control inputs
    thrust = control_inputs(1);
    torque = control_inputs(2:4);
    
    % Extract current state
    pos = current_state(1:3);
    vel = current_state(4:6);
    att = current_state(7:9);
    
    % Position integration
    new_pos = pos + vel * dt;
    
    % Improved velocity dynamics with proper thrust application
    gravity = [0; 0; params.g(3)];  % Gravity vector (positive down in NED)
    
    % Convert thrust to world frame using rotation matrix
    R = rotation_matrix(att(1), att(2), att(3));
    thrust_body = [0; 0; -thrust];  % Thrust in body frame (upward)
    thrust_world = R * thrust_body;  % Convert to world frame
    
    % Total acceleration
    accel = gravity + thrust_world / params.mass;
    
    % Add damping for stability
    damping = -0.2 * vel;
    accel = accel + damping;
    
    new_vel = vel + accel * dt;
    
    % Improved attitude dynamics
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
    
    % Add attitude damping
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

function plot_simple_results(time, true_pos, expert_pos, obstacle_map, start_pos, goal_pos)
    %% Plot Simple Results
    
    figure('Position', [100, 100, 1200, 800]);
    
    subplot(2, 3, 1);
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
    
    subplot(2, 3, 2);
    plot(time, true_pos, 'b-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('Position vs Time');
    legend('X', 'Y', 'Z', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 3);
    plot(time, expert_pos, 'r-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Expert Position (m)');
    title('Expert Position vs Time');
    legend('X', 'Y', 'Z', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 4);
    plot(time, sqrt(sum(true_pos.^2, 1)), 'b-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Distance from Origin (m)');
    title('Path Length');
    grid on;
    
    subplot(2, 3, 5);
    plot(time, sqrt(sum(expert_pos.^2, 1)), 'r-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Expert Distance from Origin (m)');
    title('Expert Path Length');
    grid on;
    
    subplot(2, 3, 6);
    plot(time, sqrt(sum((true_pos - expert_pos).^2, 1)), 'k-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Tracking Error (m)');
    title('Expert Tracking Error');
    grid on;
    
    sgtitle('Simple Expert System Test', 'FontSize', 14);
    saveas(gcf, 'simple_expert_test.png');
    fprintf('Plot saved: simple_expert_test.png\n');
end
