%% test_simple_flight_control.m - Test Simple and Stable Flight Control
% PURPOSE
% Test a very simple, stable flight control system that focuses on basic
% hover stability before adding complexity.

function test_simple_flight_control()
    fprintf('=== Simple Flight Control System Testing ===\n\n');
    
    % Setup
    addpath('rl_obstacle_avoidance');
    addpath('.');
    params = rl_parameters();
    
    % Test 1: Basic Hover Control
    fprintf('Test 1: Basic Hover Control\n');
    test_basic_hover(params);
    
    % Test 2: Simple Position Control
    fprintf('\nTest 2: Simple Position Control\n');
    test_simple_position_control(params);
    
    % Test 3: Stability Analysis
    fprintf('\nTest 3: Stability Analysis\n');
    test_stability_analysis(params);
    
    fprintf('\n=== Simple Flight Control Testing Complete ===\n');
end

function test_basic_hover(params)
    %% Test Basic Hover Control with Simple Controller
    
    % Test parameters
    duration = 10;  % seconds
    dt = params.Ts.physics;
    steps = round(duration / dt);
    
    % Initial state (hover at 10m altitude)
    true_state = [0; 0; 10; 0; 0; 0; 0; 0; 0];  % [pos; vel; att]
    target_state = [0; 0; 10; 0; 0; 0; 0; 0; 0];  % Hover target
    
    % Simple controller gains
    kp_z = 0.5;  % Position gain
    kd_z = 0.2;  % Velocity gain
    kp_xy = 0.3;  % XY position gain
    kd_xy = 0.1;  % XY velocity gain
    
    % Logging
    time = zeros(1, steps);
    true_pos = zeros(3, steps);
    true_vel = zeros(3, steps);
    control_inputs = zeros(4, steps);
    
    fprintf('  Testing basic hover control for %.1f seconds...\n', duration);
    
    for step = 1:steps
        t = step * dt;
        time(step) = t;
        
        % Extract current state
        pos = true_state(1:3);
        vel = true_state(4:6);
        att = true_state(7:9);
        
        % Simple proportional control
        pos_error = target_state(1:3) - pos;
        vel_error = target_state(4:6) - vel;
        
        % Z control (altitude)
        thrust = params.mass * abs(params.g(3)) + kp_z * pos_error(3) + kd_z * vel_error(3);
        thrust = max(min(thrust, 2.0 * params.mass * abs(params.g(3))), 0.5 * params.mass * abs(params.g(3)));
        
        % XY control (position)
        torque_x = kp_xy * pos_error(2) + kd_xy * vel_error(2);  % Roll for Y movement
        torque_y = -kp_xy * pos_error(1) - kd_xy * vel_error(1);  % Pitch for X movement
        torque_z = 0;  % No yaw control for now
        
        % Limit torques
        max_torque = 0.02;
        torque_x = max(min(torque_x, max_torque), -max_torque);
        torque_y = max(min(torque_y, max_torque), -max_torque);
        torque_z = max(min(torque_z, max_torque), -max_torque);
        
        control_inputs(:, step) = [thrust; torque_x; torque_y; torque_z];
        
        % Log current state
        true_pos(:, step) = pos;
        true_vel(:, step) = vel;
        
        % Simulate dynamics
        true_state = simulate_simple_drone_dynamics(true_state, control_inputs(:, step), dt, params);
    end
    
    % Analyze results
    pos_drift = norm(true_pos(:, end) - true_pos(:, 1));
    pos_std = std(true_pos, 0, 2);
    vel_std = std(true_vel, 0, 2);
    
    fprintf('  Position drift: %.3f m\n', pos_drift);
    fprintf('  Position std: [%.3f %.3f %.3f] m\n', pos_std(1), pos_std(2), pos_std(3));
    fprintf('  Velocity std: [%.3f %.3f %.3f] m/s\n', vel_std(1), vel_std(2), vel_std(3));
    
    % Performance assessment
    if pos_drift < 1.0 && max(pos_std) < 0.5 && max(vel_std) < 0.2
        fprintf('  ✅ Basic hover control: EXCELLENT\n');
    elseif pos_drift < 2.0 && max(pos_std) < 1.0 && max(vel_std) < 0.5
        fprintf('  ✅ Basic hover control: GOOD\n');
    else
        fprintf('  ⚠️  Basic hover control: NEEDS TUNING\n');
    end
    
    % Plot results
    plot_simple_results(time, true_pos, true_vel, control_inputs, 'Basic Hover Control');
end

function test_simple_position_control(params)
    %% Test Simple Position Control
    
    % Test parameters
    duration = 15;  % seconds
    dt = params.Ts.physics;
    steps = round(duration / dt);
    
    % Initial state
    true_state = [0; 0; 10; 0; 0; 0; 0; 0; 0];
    
    % Simple controller gains
    kp_z = 0.5;
    kd_z = 0.2;
    kp_xy = 0.3;
    kd_xy = 0.1;
    
    % Logging
    time = zeros(1, steps);
    true_pos = zeros(3, steps);
    true_vel = zeros(3, steps);
    target_pos = zeros(3, steps);
    control_inputs = zeros(4, steps);
    
    fprintf('  Testing simple position control for %.1f seconds...\n', duration);
    
    for step = 1:steps
        t = step * dt;
        time(step) = t;
        
        % Generate simple target trajectory
        if t < 5
            target_state = [0; 0; 10; 0; 0; 0; 0; 0; 0];  % Hover
        elseif t < 10
            target_state = [5; 0; 10; 0; 0; 0; 0; 0; 0];  % Move to X=5
        else
            target_state = [5; 5; 10; 0; 0; 0; 0; 0; 0];  % Move to Y=5
        end
        
        target_pos(:, step) = target_state(1:3);
        
        % Extract current state
        pos = true_state(1:3);
        vel = true_state(4:6);
        att = true_state(7:9);
        
        % Simple proportional control
        pos_error = target_state(1:3) - pos;
        vel_error = target_state(4:6) - vel;
        
        % Z control (altitude)
        thrust = params.mass * abs(params.g(3)) + kp_z * pos_error(3) + kd_z * vel_error(3);
        thrust = max(min(thrust, 2.0 * params.mass * abs(params.g(3))), 0.5 * params.mass * abs(params.g(3)));
        
        % XY control (position)
        torque_x = kp_xy * pos_error(2) + kd_xy * vel_error(2);  % Roll for Y movement
        torque_y = -kp_xy * pos_error(1) - kd_xy * vel_error(1);  % Pitch for X movement
        torque_z = 0;  % No yaw control for now
        
        % Limit torques
        max_torque = 0.02;
        torque_x = max(min(torque_x, max_torque), -max_torque);
        torque_y = max(min(torque_y, max_torque), -max_torque);
        torque_z = max(min(torque_z, max_torque), -max_torque);
        
        control_inputs(:, step) = [thrust; torque_x; torque_y; torque_z];
        
        % Log current state
        true_pos(:, step) = pos;
        true_vel(:, step) = vel;
        
        % Simulate dynamics
        true_state = simulate_simple_drone_dynamics(true_state, control_inputs(:, step), dt, params);
    end
    
    % Analyze results
    final_pos = true_pos(:, end);
    final_target = target_pos(:, end);
    final_error = norm(final_pos - final_target);
    
    fprintf('  Final position: [%.1f %.1f %.1f]\n', final_pos(1), final_pos(2), final_pos(3));
    fprintf('  Final target: [%.1f %.1f %.1f]\n', final_target(1), final_target(2), final_target(3));
    fprintf('  Final error: %.3f m\n', final_error);
    
    if final_error < 1.0
        fprintf('  ✅ Simple position control: EXCELLENT\n');
    elseif final_error < 2.0
        fprintf('  ✅ Simple position control: GOOD\n');
    else
        fprintf('  ⚠️  Simple position control: NEEDS IMPROVEMENT\n');
    end
    
    % Plot results
    plot_simple_results(time, true_pos, true_vel, control_inputs, 'Simple Position Control');
end

function test_stability_analysis(params)
    %% Test Stability Analysis
    
    % Test parameters
    duration = 20;  % seconds
    dt = params.Ts.physics;
    steps = round(duration / dt);
    
    % Initial state with small disturbance
    true_state = [0.1; 0.1; 10.1; 0.1; 0.1; 0.1; 0.05; 0.05; 0.05];
    target_state = [0; 0; 10; 0; 0; 0; 0; 0; 0];
    
    % Simple controller gains
    kp_z = 0.5;
    kd_z = 0.2;
    kp_xy = 0.3;
    kd_xy = 0.1;
    
    % Logging
    time = zeros(1, steps);
    true_pos = zeros(3, steps);
    true_vel = zeros(3, steps);
    true_att = zeros(3, steps);
    control_inputs = zeros(4, steps);
    
    fprintf('  Testing stability analysis for %.1f seconds...\n', duration);
    
    for step = 1:steps
        t = step * dt;
        time(step) = t;
        
        % Extract current state
        pos = true_state(1:3);
        vel = true_state(4:6);
        att = true_state(7:9);
        
        % Simple proportional control
        pos_error = target_state(1:3) - pos;
        vel_error = target_state(4:6) - vel;
        
        % Z control (altitude)
        thrust = params.mass * abs(params.g(3)) + kp_z * pos_error(3) + kd_z * vel_error(3);
        thrust = max(min(thrust, 2.0 * params.mass * abs(params.g(3))), 0.5 * params.mass * abs(params.g(3)));
        
        % XY control (position)
        torque_x = kp_xy * pos_error(2) + kd_xy * vel_error(2);  % Roll for Y movement
        torque_y = -kp_xy * pos_error(1) - kd_xy * vel_error(1);  % Pitch for X movement
        torque_z = 0;  % No yaw control for now
        
        % Limit torques
        max_torque = 0.02;
        torque_x = max(min(torque_x, max_torque), -max_torque);
        torque_y = max(min(torque_y, max_torque), -max_torque);
        torque_z = max(min(torque_z, max_torque), -max_torque);
        
        control_inputs(:, step) = [thrust; torque_x; torque_y; torque_z];
        
        % Log current state
        true_pos(:, step) = pos;
        true_vel(:, step) = vel;
        true_att(:, step) = att;
        
        % Simulate dynamics
        true_state = simulate_simple_drone_dynamics(true_state, control_inputs(:, step), dt, params);
    end
    
    % Analyze stability
    pos_rmse = sqrt(mean(sum(true_pos.^2, 1)));
    vel_rmse = sqrt(mean(sum(true_vel.^2, 1)));
    att_rmse = sqrt(mean(sum(true_att.^2, 1)));
    
    fprintf('  Position RMSE: %.3f m\n', pos_rmse);
    fprintf('  Velocity RMSE: %.3f m/s\n', vel_rmse);
    fprintf('  Attitude RMSE: %.3f rad\n', att_rmse);
    
    if pos_rmse < 0.5 && vel_rmse < 0.2 && att_rmse < 0.1
        fprintf('  ✅ Stability analysis: EXCELLENT\n');
    elseif pos_rmse < 1.0 && vel_rmse < 0.5 && att_rmse < 0.2
        fprintf('  ✅ Stability analysis: GOOD\n');
    else
        fprintf('  ⚠️  Stability analysis: NEEDS IMPROVEMENT\n');
    end
    
    % Plot results
    plot_simple_results(time, true_pos, true_vel, control_inputs, 'Stability Analysis');
end

function true_state = simulate_simple_drone_dynamics(current_state, control_inputs, dt, params)
    %% Very Simple and Stable Drone Dynamics Simulation
    
    % Extract control inputs
    thrust = control_inputs(1);
    torque = control_inputs(2:4);
    
    % Extract current state
    pos = current_state(1:3);
    vel = current_state(4:6);
    att = current_state(7:9);
    
    % Very simple dynamics
    % Position integration
    new_pos = pos + vel * dt;
    
    % Simple velocity integration
    gravity = [0; 0; params.g(3)];  % Gravity vector (positive down in NED)
    thrust_world = [0; 0; -thrust];  % Thrust in world frame (upward, so negative Z)
    
    % Total acceleration
    accel = gravity + thrust_world / params.mass;
    
    % Strong damping for stability
    damping = -0.8 * vel;  % Very strong velocity damping
    accel = accel + damping;
    
    new_vel = vel + accel * dt;
    
    % Simple attitude integration
    if length(torque) == 3 && length(params.I) == 3
        if isvector(params.I)
            I_diag = params.I;
        else
            I_diag = diag(params.I);
        end
        att_rate = torque ./ I_diag;
    else
        att_rate = torque / params.mass;  % Fallback
    end
    
    % Add attitude damping
    att_damping = -0.5 * att;  % Attitude damping
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

function plot_simple_results(time, true_pos, true_vel, control_inputs, title_str)
    %% Plot Simple Control Results
    
    figure('Position', [100, 100, 1200, 800]);
    
    subplot(2, 3, 1);
    plot(time, true_pos, 'b-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('Position Tracking');
    legend('X', 'Y', 'Z', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 2);
    plot(time, true_vel, 'r-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity');
    legend('Vx', 'Vy', 'Vz', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 3);
    plot(time, control_inputs, 'k-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Control Input');
    title('Control Inputs');
    legend('Thrust', 'Torque X', 'Torque Y', 'Torque Z', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 4);
    plot(time, sqrt(sum(true_pos.^2, 1)), 'm-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Distance from Origin (m)');
    title('Position Stability');
    grid on;
    
    subplot(2, 3, 5);
    plot(time, sqrt(sum(true_vel.^2, 1)), 'c-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Velocity Magnitude (m/s)');
    title('Velocity Stability');
    grid on;
    
    subplot(2, 3, 6);
    plot3(true_pos(1, :), true_pos(2, :), true_pos(3, :), 'b-', 'LineWidth', 2);
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('3D Trajectory');
    grid on;
    
    sgtitle(title_str, 'FontSize', 14);
    saveas(gcf, 'simple_flight_control_test.png');
    fprintf('  Plot saved: simple_flight_control_test.png\n');
end
