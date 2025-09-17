%% test_professional_flight_control.m - Test Professional Flight Control System
% PURPOSE
% Test the professional flight control system using MATLAB toolboxes
% before integrating with RL training.

function test_professional_flight_control()
    fprintf('=== Professional Flight Control System Testing ===\n\n');
    
    % Setup
    addpath('rl_obstacle_avoidance');
    addpath('.');
    params = rl_parameters();
    
    % Test 1: Professional Hover Control
    fprintf('Test 1: Professional Hover Control\n');
    test_professional_hover(params);
    
    % Test 2: Waypoint Following with Navigation Toolbox
    fprintf('\nTest 2: Waypoint Following with Navigation Toolbox\n');
    test_waypoint_following_professional(params);
    
    % Test 3: Advanced Flight Maneuvers
    fprintf('\nTest 3: Advanced Flight Maneuvers\n');
    test_advanced_maneuvers(params);
    
    % Test 4: Control System Performance Analysis
    fprintf('\nTest 4: Control System Performance Analysis\n');
    test_control_performance(params);
    
    fprintf('\n=== Professional Flight Control Testing Complete ===\n');
end

function test_professional_hover(params)
    %% Test Professional Hover Control Using Control System Toolbox
    
    % Create professional flight controller
    flight_controller = autonomous_flight_controller(params);
    
    % Test parameters
    duration = 15;  % seconds
    dt = params.Ts.physics;
    steps = round(duration / dt);
    
    % Initial state (hover at 10m altitude)
    true_state = [0; 0; 10; 0; 0; 0; 0; 0; 0];  % [pos; vel; att]
    target_state = [0; 0; 10; 0; 0; 0; 0; 0; 0];  % Hover target
    
    % Logging
    time = zeros(1, steps);
    true_pos = zeros(3, steps);
    true_vel = zeros(3, steps);
    true_att = zeros(3, steps);
    control_inputs = zeros(4, steps);
    control_info = cell(1, steps);
    
    fprintf('  Testing professional hover control for %.1f seconds...\n', duration);
    
    for step = 1:steps
        t = step * dt;
        time(step) = t;
        
        % Compute control using professional controller
        [control_inputs(:, step), control_info{step}] = flight_controller.compute_control(true_state, target_state, dt);
        
        % Log current state
        true_pos(:, step) = true_state(1:3);
        true_vel(:, step) = true_state(4:6);
        true_att(:, step) = true_state(7:9);
        
        % Simulate dynamics (simplified for testing)
        true_state = simulate_drone_dynamics(true_state, control_inputs(:, step), dt, params);
    end
    
    % Analyze results
    pos_drift = norm(true_pos(:, end) - true_pos(:, 1));
    pos_std = std(true_pos, 0, 2);
    vel_std = std(true_vel, 0, 2);
    att_std = std(true_att, 0, 2);
    
    fprintf('  Position drift: %.3f m\n', pos_drift);
    fprintf('  Position std: [%.3f %.3f %.3f] m\n', pos_std(1), pos_std(2), pos_std(3));
    fprintf('  Velocity std: [%.3f %.3f %.3f] m/s\n', vel_std(1), vel_std(2), vel_std(3));
    fprintf('  Attitude std: [%.3f %.3f %.3f] rad\n', att_std(1), att_std(2), att_std(3));
    
    % Performance assessment
    if pos_drift < 0.5 && max(pos_std) < 0.2 && max(vel_std) < 0.1
        fprintf('  ✅ Professional hover control: EXCELLENT\n');
    elseif pos_drift < 1.0 && max(pos_std) < 0.5 && max(vel_std) < 0.2
        fprintf('  ✅ Professional hover control: GOOD\n');
    else
        fprintf('  ⚠️  Professional hover control: NEEDS TUNING\n');
    end
    
    % Plot results
    plot_hover_results(time, true_pos, true_vel, true_att, control_inputs, 'Professional Hover Control');
end

function test_waypoint_following_professional(params)
    %% Test Waypoint Following Using Navigation Toolbox
    
    flight_controller = autonomous_flight_controller(params);
    
    % Define waypoints
    waypoints = [0, 10, 20, 30, 20, 10, 0; 0, 5, 10, 15, 20, 15, 10; 10, 12, 15, 18, 15, 12, 10];
    flight_controller.set_waypoints(waypoints);
    
    % Test parameters
    duration = 40;  % seconds
    dt = params.Ts.physics;
    steps = round(duration / dt);
    
    % Initial state
    true_state = [waypoints(:, 1); 0; 0; 0; 0; 0; 0];
    
    % Logging
    time = zeros(1, steps);
    true_pos = zeros(3, steps);
    target_pos = zeros(3, steps);
    waypoint_distances = zeros(1, steps);
    control_inputs = zeros(4, steps);
    
    fprintf('  Testing waypoint following with %d waypoints...\n', size(waypoints, 2));
    
    for step = 1:steps
        t = step * dt;
        time(step) = t;
        
        % Get next target with look-ahead and velocity feed-forward
        la_target = flight_controller.lookahead_target(true_state(1:3), 1.5, 3.0);
        % Use get_next_target only to advance waypoint index when within tolerance
        [~, waypoint_reached] = flight_controller.get_next_target(true_state(1:3));
        target_state = la_target;
        target_pos(:, step) = target_state(1:3);
        
        % Compute control
        [control_inputs(:, step), ~] = flight_controller.compute_control(true_state, target_state, dt);
        
        % Log current state
        true_pos(:, step) = true_state(1:3);
        
        % Calculate distance to current waypoint
        if flight_controller.current_waypoint <= size(waypoints, 2)
            current_waypoint_pos = waypoints(:, flight_controller.current_waypoint);
            waypoint_distances(step) = norm(true_state(1:3) - current_waypoint_pos);
        else
            waypoint_distances(step) = 0;
        end
        
        % Simulate dynamics
        true_state = simulate_drone_dynamics(true_state, control_inputs(:, step), dt, params);
        
        if waypoint_reached
            fprintf('    Reached waypoint %d at t=%.1fs\n', flight_controller.current_waypoint-1, t);
        end
    end
    
    % Analyze results
    final_pos = true_pos(:, end);
    final_waypoint = waypoints(:, end);
    final_error = norm(final_pos - final_waypoint);
    
    % Calculate path efficiency
    total_distance = 0;
    for i = 2:size(waypoints, 2)
        total_distance = total_distance + norm(waypoints(:, i) - waypoints(:, i-1));
    end
    actual_distance = sum(sqrt(sum(diff(true_pos, 1, 2).^2, 1)));
    efficiency = total_distance / actual_distance;
    
    fprintf('  Final position: [%.1f %.1f %.1f]\n', final_pos(1), final_pos(2), final_pos(3));
    fprintf('  Final waypoint: [%.1f %.1f %.1f]\n', final_waypoint(1), final_waypoint(2), final_waypoint(3));
    fprintf('  Final error: %.3f m\n', final_error);
    fprintf('  Path efficiency: %.2f\n', efficiency);
    
    if final_error < 2.0 && efficiency > 0.8
        fprintf('  ✅ Professional waypoint following: EXCELLENT\n');
    elseif final_error < 5.0 && efficiency > 0.6
        fprintf('  ✅ Professional waypoint following: GOOD\n');
    else
        fprintf('  ⚠️  Professional waypoint following: NEEDS IMPROVEMENT\n');
    end
    
    % Plot results
    plot_waypoint_results(time, true_pos, waypoints, waypoint_distances, 'Professional Waypoint Following');
end

function test_advanced_maneuvers(params)
    %% Test Advanced Flight Maneuvers
    
    flight_controller = autonomous_flight_controller(params);
    
    % Test parameters
    duration = 25;  % seconds
    dt = params.Ts.physics;
    steps = round(duration / dt);
    
    % Initial state
    true_state = [0; 0; 10; 0; 0; 0; 0; 0; 0];
    
    % Logging
    time = zeros(1, steps);
    true_pos = zeros(3, steps);
    true_vel = zeros(3, steps);
    true_att = zeros(3, steps);
    control_inputs = zeros(4, steps);
    
    fprintf('  Testing advanced maneuvers...\n');
    
    for step = 1:steps
        t = step * dt;
        time(step) = t;
        
        % Generate complex trajectory
        target_pos = [5*sin(0.2*t); 3*cos(0.15*t); 10 + 2*sin(0.1*t)];
        target_vel = [cos(0.2*t); -0.45*sin(0.15*t); 0.2*cos(0.1*t)];
        target_att = [0.1*sin(0.3*t); 0.1*cos(0.25*t); 0.2*t];
        
        target_state = [target_pos; target_vel; target_att];
        
        % Compute control
        [control_inputs(:, step), ~] = flight_controller.compute_control(true_state, target_state, dt);
        
        % Log current state
        true_pos(:, step) = true_state(1:3);
        true_vel(:, step) = true_state(4:6);
        true_att(:, step) = true_state(7:9);
        
        % Simulate dynamics
        true_state = simulate_drone_dynamics(true_state, control_inputs(:, step), dt, params);
    end
    
    % Analyze results
    pos_tracking_error = sqrt(mean(sum((true_pos - [5*sin(0.2*time); 3*cos(0.15*time); 10 + 2*sin(0.1*time)]).^2, 1)));
    vel_tracking_error = sqrt(mean(sum((true_vel - [cos(0.2*time); -0.45*sin(0.15*time); 0.2*cos(0.1*time)]).^2, 1)));
    
    fprintf('  Position tracking error: %.3f m\n', pos_tracking_error);
    fprintf('  Velocity tracking error: %.3f m/s\n', vel_tracking_error);
    
    if pos_tracking_error < 1.0 && vel_tracking_error < 0.5
        fprintf('  ✅ Advanced maneuvers: EXCELLENT\n');
    elseif pos_tracking_error < 2.0 && vel_tracking_error < 1.0
        fprintf('  ✅ Advanced maneuvers: GOOD\n');
    else
        fprintf('  ⚠️  Advanced maneuvers: NEEDS IMPROVEMENT\n');
    end
    
    % Plot results
    plot_maneuver_results(time, true_pos, true_vel, true_att, 'Advanced Flight Maneuvers');
end

function test_control_performance(params)
    %% Test Control System Performance Analysis
    
    flight_controller = autonomous_flight_controller(params);
    
    % Test with step inputs
    duration = 20;  % seconds
    dt = params.Ts.physics;
    steps = round(duration / dt);
    
    % Initial state
    true_state = [0; 0; 10; 0; 0; 0; 0; 0; 0];
    
    % Logging
    time = zeros(1, steps);
    true_pos = zeros(3, steps);
    target_pos = zeros(3, steps);
    control_inputs = zeros(4, steps);
    
    fprintf('  Testing control system performance...\n');
    
    for step = 1:steps
        t = step * dt;
        time(step) = t;
        
        % Step inputs at different times
        if t < 5
            target_state = [0; 0; 10; 0; 0; 0; 0; 0; 0];  % Hover
        elseif t < 10
            target_state = [5; 0; 10; 0; 0; 0; 0; 0; 0];  % Step in X
        elseif t < 15
            target_state = [5; 5; 15; 0; 0; 0; 0; 0; 0];  % Step in Y and Z
        else
            target_state = [0; 0; 10; 0; 0; 0; 0; 0; 0];  % Return to hover
        end
        
        target_pos(:, step) = target_state(1:3);
        
        % Compute control
        [control_inputs(:, step), ~] = flight_controller.compute_control(true_state, target_state, dt);
        
        % Log current state
        true_pos(:, step) = true_state(1:3);
        
        % Simulate dynamics
        true_state = simulate_drone_dynamics(true_state, control_inputs(:, step), dt, params);
    end
    
    % Performance analysis
    analyze_control_performance(time, true_pos, target_pos, control_inputs);
    
    % Plot results
    plot_performance_results(time, true_pos, target_pos, control_inputs, 'Control System Performance');
end

function true_state = simulate_drone_dynamics(current_state, control_inputs, dt, params)
    %% Improved Drone Dynamics Simulation
    
    % Extract control inputs
    thrust = control_inputs(1);
    torque = control_inputs(2:4);
    
    % Extract current state
    pos = current_state(1:3);
    vel = current_state(4:6);
    att = current_state(7:9);
    
    % Position integration (semi-implicit)
    new_pos = pos + vel * dt;

    % Gravity (NED, +Z down)
    gravity = [0; 0; params.g(3)];

    % Rotate body thrust to world frame
    % eul2rotm expects [yaw pitch roll] for 'ZYX'; our state is [roll pitch yaw]
    R_wb = eul2rotm([att(3) att(2) att(1)], 'ZYX');
    thrust_world = R_wb * [0; 0; -thrust];

    % Total acceleration (add physical drag)
    accel = gravity + thrust_world / params.mass;
    rho = 1.225;
    CdA = params.drag_coeff;  % effective drag area coeff
    vmag = max(norm(vel), 1e-6);
    drag_world = -0.5 * rho * CdA * vmag * vel; % -k |v| v
    accel = accel + drag_world / params.mass;

    new_vel = vel + accel * dt;

    % Attitude integration using body rates and quaternion internal state
    persistent omega_b q
    if isempty(omega_b); omega_b = zeros(3,1); end
    if isempty(q); q = eul2quat(att(:).', 'ZYX'); end

    if isvector(params.I)
        I = diag(params.I);
    else
        I = params.I;
    end
    omega_dot = I \ (torque(:) - cross(omega_b, I*omega_b));
    omega_b = omega_b + omega_dot * dt;

    % Quaternion kinematics
    w = omega_b(1); x = omega_b(2); y = omega_b(3); %#ok<NASGU>
    Omega = [ 0,          -omega_b';
              omega_b,    -[0, -omega_b(3), omega_b(2); omega_b(3), 0, -omega_b(1); -omega_b(2), omega_b(1), 0] ];
    q = (q.' + 0.5 * Omega * q.' * dt).';
    q = q / norm(q);

    eulZyx = quat2eul(q, 'ZYX');
    new_att = [eulZyx(3); eulZyx(2); eulZyx(1)];
    
    % Wrap angles
    new_att(3) = wrapToPi(new_att(3));
    
    % Ensure all vectors are column vectors
    new_pos = new_pos(:);
    new_vel = new_vel(:);
    new_att = new_att(:);
    
    true_state = [new_pos; new_vel; new_att];
end

function analyze_control_performance(time, true_pos, target_pos, control_inputs)
    %% Analyze Control System Performance
    
    % Calculate tracking errors
    pos_error = target_pos - true_pos;
    pos_rmse = sqrt(mean(sum(pos_error.^2, 1)));
    
    % Calculate control effort
    control_effort = sum(control_inputs.^2, 1);
    avg_control_effort = mean(control_effort);
    
    % Calculate settling time (for step responses)
    step_times = [5, 10, 15];  % Step input times
    settling_times = zeros(1, length(step_times));
    
    for i = 1:length(step_times)
        step_idx = find(time >= step_times(i), 1);
        if ~isempty(step_idx)
            % Find when error drops below 5% of step size
            step_size = norm(target_pos(:, step_idx) - true_pos(:, step_idx));
            error_threshold = 0.05 * step_size;
            
            error_magnitude = sqrt(sum(pos_error(:, step_idx:end).^2, 1));
            settled_idx = find(error_magnitude < error_threshold, 1);
            
            if ~isempty(settled_idx)
                settling_times(i) = time(step_idx + settled_idx - 1) - step_times(i);
            end
        end
    end
    
    fprintf('  Position RMSE: %.3f m\n', pos_rmse);
    fprintf('  Average control effort: %.3f\n', avg_control_effort);
    fprintf('  Settling times: [%.2f %.2f %.2f] s\n', settling_times(1), settling_times(2), settling_times(3));
    
    % Performance assessment
    if pos_rmse < 0.5 && avg_control_effort < 10 && max(settling_times) < 3.0
        fprintf('  ✅ Control performance: EXCELLENT\n');
    elseif pos_rmse < 1.0 && avg_control_effort < 20 && max(settling_times) < 5.0
        fprintf('  ✅ Control performance: GOOD\n');
    else
        fprintf('  ⚠️  Control performance: NEEDS TUNING\n');
    end
end

function plot_hover_results(time, true_pos, true_vel, true_att, control_inputs, title_str)
    %% Plot Hover Control Results
    
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
    plot(time, rad2deg(true_att), 'g-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Attitude (deg)');
    title('Attitude');
    legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 4);
    plot(time, control_inputs, 'k-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Control Input');
    title('Control Inputs');
    legend('Thrust', 'Torque X', 'Torque Y', 'Torque Z', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 5);
    plot(time, sqrt(sum(true_pos.^2, 1)), 'm-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Distance from Origin (m)');
    title('Position Stability');
    grid on;
    
    subplot(2, 3, 6);
    plot(time, sqrt(sum(true_vel.^2, 1)), 'c-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Velocity Magnitude (m/s)');
    title('Velocity Stability');
    grid on;
    
    sgtitle(title_str, 'FontSize', 14);
    saveas(gcf, 'professional_hover_test.png');
    fprintf('  Plot saved: professional_hover_test.png\n');
end

function plot_waypoint_results(time, true_pos, waypoints, waypoint_distances, title_str)
    %% Plot Waypoint Following Results
    
    figure('Position', [200, 200, 1200, 800]);
    
    subplot(2, 3, 1);
    plot3(waypoints(1, :), waypoints(2, :), waypoints(3, :), 'ko-', 'LineWidth', 2, 'MarkerSize', 8);
    hold on;
    plot3(true_pos(1, :), true_pos(2, :), true_pos(3, :), 'b-', 'LineWidth', 2);
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('3D Trajectory');
    legend('Waypoints', 'Actual Path', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 2);
    plot(time, waypoint_distances, 'g-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Distance to Waypoint (m)');
    title('Waypoint Following Progress');
    grid on;
    
    subplot(2, 3, 3);
    plot(time, true_pos, 'b-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('Position vs Time');
    legend('X', 'Y', 'Z', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 4);
    plot(time(2:end), diff(true_pos, 1, 2), 'r-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity (Numerical Derivative)');
    legend('Vx', 'Vy', 'Vz', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 5);
    plot(time, sqrt(sum(true_pos.^2, 1)), 'm-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Distance from Origin (m)');
    title('Path Length');
    grid on;
    
    subplot(2, 3, 6);
    plot(time, waypoint_distances, 'g-', 'LineWidth', 1);
    hold on;
    plot(time, ones(size(time)) * 2, 'r--', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Distance to Waypoint (m)');
    title('Waypoint Proximity');
    legend('Distance', 'Tolerance', 'Location', 'best');
    grid on;
    
    sgtitle(title_str, 'FontSize', 14);
    saveas(gcf, 'professional_waypoint_test.png');
    fprintf('  Plot saved: professional_waypoint_test.png\n');
end

function plot_maneuver_results(time, true_pos, true_vel, true_att, title_str)
    %% Plot Advanced Maneuver Results
    
    figure('Position', [300, 300, 1200, 800]);
    
    subplot(2, 3, 1);
    plot3(true_pos(1, :), true_pos(2, :), true_pos(3, :), 'b-', 'LineWidth', 2);
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('3D Maneuver Trajectory');
    grid on;
    
    subplot(2, 3, 2);
    plot(time, true_pos, 'b-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('Position Tracking');
    legend('X', 'Y', 'Z', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 3);
    plot(time, true_vel, 'r-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity Tracking');
    legend('Vx', 'Vy', 'Vz', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 4);
    plot(time, rad2deg(true_att), 'g-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Attitude (deg)');
    title('Attitude Tracking');
    legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 5);
    plot(time, sqrt(sum(true_vel.^2, 1)), 'c-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Speed (m/s)');
    title('Speed Profile');
    grid on;
    
    subplot(2, 3, 6);
    plot(time, sqrt(sum(true_att.^2, 1)), 'm-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Attitude Magnitude (rad)');
    title('Attitude Magnitude');
    grid on;
    
    sgtitle(title_str, 'FontSize', 14);
    saveas(gcf, 'professional_maneuver_test.png');
    fprintf('  Plot saved: professional_maneuver_test.png\n');
end

function plot_performance_results(time, true_pos, target_pos, control_inputs, title_str)
    %% Plot Control Performance Results
    
    figure('Position', [400, 400, 1200, 800]);
    
    subplot(2, 3, 1);
    plot(time, true_pos, 'b-', 'LineWidth', 2);
    hold on;
    plot(time, target_pos, 'r--', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('Position Tracking');
    legend('Actual', 'Target', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 2);
    pos_error = target_pos - true_pos;
    plot(time, pos_error, 'r-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position Error (m)');
    title('Tracking Error');
    legend('X Error', 'Y Error', 'Z Error', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 3);
    plot(time, sqrt(sum(pos_error.^2, 1)), 'k-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position Error Magnitude (m)');
    title('Tracking Error Magnitude');
    grid on;
    
    subplot(2, 3, 4);
    plot(time, control_inputs, 'g-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Control Input');
    title('Control Inputs');
    legend('Thrust', 'Torque X', 'Torque Y', 'Torque Z', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 5);
    control_effort = sum(control_inputs.^2, 1);
    plot(time, control_effort, 'm-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Control Effort');
    title('Control Effort');
    grid on;
    
    subplot(2, 3, 6);
    plot(time, sqrt(sum(true_pos.^2, 1)), 'c-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Distance from Origin (m)');
    title('Position Magnitude');
    grid on;
    
    sgtitle(title_str, 'FontSize', 14);
    saveas(gcf, 'professional_performance_test.png');
    fprintf('  Plot saved: professional_performance_test.png\n');
end
