%% test_basic_hover.m - Basic Hover Test
% PURPOSE
% Test basic hover control to verify the dynamics simulation works correctly.

function test_basic_hover()
    fprintf('=== Basic Hover Test ===\n\n');
    
    % Setup
    addpath('rl_obstacle_avoidance');
    addpath('.');
    
    % Load parameters
    run('parameters.m');
    params = rl_parameters();
    
    % Test parameters
    duration = 10;  % seconds
    dt = params.Ts.physics;
    steps = round(duration / dt);
    
    % Initial state (hover at origin)
    true_state = [0; 0; 5; 0; 0; 0; 0; 0; 0];  % [pos; vel; att]
    
    % Logging
    time = zeros(1, steps);
    true_pos = zeros(3, steps);
    control_inputs = zeros(4, steps);
    
    fprintf('Testing basic hover for %.1f seconds...\n', duration);
    
    for step = 1:steps
        t = step * dt;
        time(step) = t;
        
        % Simple hover control
        control_inputs(:, step) = compute_hover_control(true_state, params);
        
        % Log true state
        true_pos(:, step) = true_state(1:3);
        
        % Basic dynamics simulation
        true_state = simulate_basic_dynamics(true_state, control_inputs(:, step), dt, params);
        
        % Print progress every 2 seconds
        if mod(t, 2) < dt
            fprintf('  t=%.1fs: pos=[%.1f %.1f %.1f], thrust=%.1fN\n', ...
                    t, true_state(1), true_state(2), true_state(3), control_inputs(1, step));
        end
    end
    
    % Analyze results
    final_pos = true_pos(:, step);
    position_error = norm(final_pos - [0; 0; 5]);
    
    fprintf('\nResults:\n');
    fprintf('  Final position: [%.1f %.1f %.1f]\n', final_pos(1), final_pos(2), final_pos(3));
    fprintf('  Target position: [0.0 0.0 5.0]\n');
    fprintf('  Position error: %.3f m\n', position_error);
    
    if position_error < 1.0
        fprintf('  ✅ Hover control: SUCCESS\n');
    elseif position_error < 3.0
        fprintf('  ⚠️  Hover control: PARTIAL SUCCESS\n');
    else
        fprintf('  ❌ Hover control: NEEDS IMPROVEMENT\n');
    end
    
    % Plot results
    plot_hover_results(time, true_pos, control_inputs);
end

function control_input = compute_hover_control(current_state, params)
    %% Simple Hover Control
    
    current_pos = current_state(1:3);
    current_vel = current_state(4:6);
    current_att = current_state(7:9);
    
    % Target position (hover at origin, 5m altitude)
    target_pos = [0; 0; 5];
    
    % Position error
    pos_error = target_pos - current_pos;
    
    % Simple proportional control
    kp = [0.5, 0.5, 1.0];  % [x, y, z] gains
    desired_vel = kp' .* pos_error;
    
    % Velocity error
    vel_error = desired_vel - current_vel;
    
    % Simple velocity control
    kv = [0.3, 0.3, 0.5];  % Velocity gains
    desired_accel = kv' .* vel_error;
    
    % Add gravity compensation (gravity is positive down in NED)
    gravity = [0; 0; params.g(3)];  % [0; 0; 9.819] - positive down
    desired_accel = desired_accel + gravity;
    
    % Convert to thrust
    thrust_magnitude = norm(desired_accel) * params.mass;
    thrust_magnitude = max(min(thrust_magnitude, 20.0), 5.0);  % 5-20 N range
    
    % Simple attitude control (level flight)
    target_att = [0; 0; 0];
    att_error = target_att - current_att;
    torque = att_error * 0.5;  % Simple proportional control
    
    % Generate control input
    control_input = [thrust_magnitude; torque];
end

function true_state = simulate_basic_dynamics(current_state, control_inputs, dt, params)
    %% Basic Dynamics Simulation
    
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

function plot_hover_results(time, true_pos, control_inputs)
    %% Plot Hover Results
    
    figure('Position', [100, 100, 1200, 800]);
    
    subplot(2, 3, 1);
    plot3(true_pos(1, :), true_pos(2, :), true_pos(3, :), 'b-', 'LineWidth', 2);
    hold on;
    plot3(0, 0, 5, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('3D Hover Trajectory');
    legend('Path', 'Target', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 2);
    plot(time, true_pos, 'b-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('Position vs Time');
    legend('X', 'Y', 'Z', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 3);
    plot(time, control_inputs(1, :), 'g-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Thrust (N)');
    title('Thrust Command');
    grid on;
    
    subplot(2, 3, 4);
    plot(time, control_inputs(2, :), 'r-', 'LineWidth', 1);
    hold on;
    plot(time, control_inputs(3, :), 'g-', 'LineWidth', 1);
    plot(time, control_inputs(4, :), 'b-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Torque (Nm)');
    title('Torque Commands');
    legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 5);
    position_error = sqrt(sum(true_pos.^2, 1));
    plot(time, position_error, 'k-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position Error (m)');
    title('Position Error');
    grid on;
    
    subplot(2, 3, 6);
    velocity_magnitude = sqrt(sum(diff(true_pos, 1, 2).^2, 1)) / (time(2) - time(1));
    plot(time(2:end), velocity_magnitude, 'm-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity Magnitude');
    grid on;
    
    sgtitle('Basic Hover Test', 'FontSize', 14);
    saveas(gcf, 'basic_hover_test.png');
    fprintf('Plot saved: basic_hover_test.png\n');
end
