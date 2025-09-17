%% test_physics.m - Physics Test
% PURPOSE
% Test basic physics to verify the dynamics simulation works correctly.

function test_physics()
    fprintf('=== Physics Test ===\n\n');
    
    % Setup
    addpath('rl_obstacle_avoidance');
    addpath('.');
    
    % Load parameters
    run('parameters.m');
    
    % Test parameters
    dt = 0.01;  % 10ms timestep
    duration = 5;  % seconds
    steps = round(duration / dt);
    
    % Initial state
    true_state = [0; 0; 5; 0; 0; 0; 0; 0; 0];  % [pos; vel; att]
    
    % Logging
    time = zeros(1, steps);
    true_pos = zeros(3, steps);
    
    fprintf('Testing basic physics for %.1f seconds...\n', duration);
    fprintf('Mass: %.3f kg, Gravity: %.3f m/s²\n', params.mass, params.g(3));
    fprintf('Required hover thrust: %.3f N\n', params.mass * params.g(3));
    
    for step = 1:steps
        t = step * dt;
        time(step) = t;
        
        % Simple hover thrust
        hover_thrust = params.mass * params.g(3);  % Exactly enough to hover
        control_input = [hover_thrust; 0; 0; 0];  % [thrust; torque]
        
        % Log true state
        true_pos(:, step) = true_state(1:3);
        
        % Basic dynamics simulation
        true_state = simulate_physics_dynamics(true_state, control_input, dt, params);
        
        % Print progress every second
        if mod(t, 1) < dt
            fprintf('  t=%.1fs: pos=[%.1f %.1f %.1f], thrust=%.1fN\n', ...
                    t, true_state(1), true_state(2), true_state(3), hover_thrust);
        end
    end
    
    % Analyze results
    final_pos = true_pos(:, step);
    position_error = norm(final_pos - [0; 0; 5]);
    
    fprintf('\nResults:\n');
    fprintf('  Final position: [%.1f %.1f %.1f]\n', final_pos(1), final_pos(2), final_pos(3));
    fprintf('  Target position: [0.0 0.0 5.0]\n');
    fprintf('  Position error: %.3f m\n', position_error);
    
    if position_error < 0.5
        fprintf('  ✅ Physics: SUCCESS\n');
    elseif position_error < 2.0
        fprintf('  ⚠️  Physics: PARTIAL SUCCESS\n');
    else
        fprintf('  ❌ Physics: NEEDS IMPROVEMENT\n');
    end
    
    % Plot results
    plot_physics_results(time, true_pos);
end

function true_state = simulate_physics_dynamics(current_state, control_input, dt, params)
    %% Physics Dynamics Simulation
    
    % Extract control inputs
    thrust = control_input(1);
    torque = control_input(2:4);
    
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
    
    % No damping for pure physics test
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
    
    new_att = att + att_rate * dt;
    
    % Wrap angles
    new_att(3) = wrapToPi(new_att(3));
    
    % Ensure all vectors are column vectors
    new_pos = new_pos(:);
    new_vel = new_vel(:);
    new_att = new_att(:);
    
    true_state = [new_pos; new_vel; new_att];
end

function plot_physics_results(time, true_pos)
    %% Plot Physics Results
    
    figure('Position', [100, 100, 1200, 600]);
    
    subplot(2, 3, 1);
    plot3(true_pos(1, :), true_pos(2, :), true_pos(3, :), 'b-', 'LineWidth', 2);
    hold on;
    plot3(0, 0, 5, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('3D Physics Trajectory');
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
    plot(time, true_pos(3, :), 'b-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Altitude (m)');
    title('Altitude vs Time');
    grid on;
    
    subplot(2, 3, 4);
    position_error = sqrt(sum(true_pos.^2, 1));
    plot(time, position_error, 'k-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position Error (m)');
    title('Position Error');
    grid on;
    
    subplot(2, 3, 5);
    velocity_magnitude = sqrt(sum(diff(true_pos, 1, 2).^2, 1)) / (time(2) - time(1));
    plot(time(2:end), velocity_magnitude, 'm-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity Magnitude');
    grid on;
    
    subplot(2, 3, 6);
    plot(true_pos(1, :), true_pos(2, :), 'b-', 'LineWidth', 2);
    hold on;
    plot(0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    xlabel('X (m)');
    ylabel('Y (m)');
    title('Top View Trajectory');
    legend('Path', 'Target', 'Location', 'best');
    grid on;
    
    sgtitle('Physics Test', 'FontSize', 14);
    saveas(gcf, 'physics_test.png');
    fprintf('Plot saved: physics_test.png\n');
end
