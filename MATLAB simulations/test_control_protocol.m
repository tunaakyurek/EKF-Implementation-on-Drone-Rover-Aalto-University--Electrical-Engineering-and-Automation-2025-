%% test_control_protocol.m - Test Control Protocol Before RL Training
% PURPOSE
% Test the autonomous flight control protocol (RL commands → Control → Dynamics → EKF)
% to ensure it works correctly before plugging into RL training.

function test_control_protocol()
    fprintf('=== Control Protocol Testing ===\n\n');
    
    % Setup
    addpath('rl_obstacle_avoidance');
    addpath('.');
    params = rl_parameters();
    
    % Test 1: Basic Hover Test
    fprintf('Test 1: Basic Hover Test\n');
    test_hover_control(params);
    
    % Test 2: Waypoint Following
    fprintf('\nTest 2: Waypoint Following Test\n');
    test_waypoint_following(params);
    
    % Test 3: EKF State Estimation
    fprintf('\nTest 3: EKF State Estimation Test\n');
    test_ekf_estimation(params);
    
    fprintf('\n=== Control Protocol Testing Complete ===\n');
end

function test_hover_control(params)
    %% Test Basic Hover Control
    
    % Create RL-EKF integration
    rl_ekf = rl_ekf_integration(params);
    
    % Initial state (hover at 10m altitude)
    true_state = [0; 0; 10; 0; 0; 0; 0; 0; 0];  % [pos; vel; att]
    dt = params.Ts.physics;
    duration = 10;  % seconds
    steps = round(duration / dt);
    
    % Logging
    time = zeros(1, steps);
    true_pos = zeros(3, steps);
    est_pos = zeros(3, steps);
    vel_cmds = zeros(4, steps);
    control_inputs = zeros(4, steps);
    
    fprintf('  Testing hover control for %.1f seconds...\n', duration);
    
    for step = 1:steps
        t = step * dt;
        time(step) = t;
        
        % Hover command (zero velocity)
        vel_cmd = [0; 0; 0; 0];
        
        % Apply control and get new state
        [new_true_state, uncertainty_info] = rl_ekf.step(true_state, vel_cmd, dt);
        
        % Log data
        true_pos(:, step) = true_state(1:3);
        est_pos(:, step) = rl_ekf.ekf_state(1:3);
        vel_cmds(:, step) = vel_cmd;
        
        % Get control inputs (for analysis)
        measurements = rl_ekf.get_current_measurements();
        if ~isempty(measurements) && isfield(measurements, 'control_inputs')
            control_inputs(:, step) = measurements.control_inputs;
        end
        
        % Update state
        true_state = new_true_state;
    end
    
    % Analyze results
    pos_drift = norm(true_pos(:, end) - true_pos(:, 1));
    pos_rmse = sqrt(mean(sum((true_pos - est_pos).^2, 1)));
    
    fprintf('  Position drift: %.3f m\n', pos_drift);
    fprintf('  Position RMSE: %.3f m\n', pos_rmse);
    
    if pos_drift < 1.0 && pos_rmse < 0.5
        fprintf('  ✅ Hover control working well\n');
    else
        fprintf('  ❌ Hover control needs improvement\n');
    end
    
    % Plot results
    figure('Position', [100, 100, 1000, 600]);
    
    subplot(2, 2, 1);
    plot(time, true_pos, 'b-', 'LineWidth', 2);
    hold on;
    plot(time, est_pos, 'r--', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('Hover Control: Position');
    legend('True', 'Estimated', 'Location', 'best');
    grid on;
    
    subplot(2, 2, 2);
    plot(time, vel_cmds, 'k-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Velocity Command (m/s)');
    title('Velocity Commands');
    grid on;
    
    subplot(2, 2, 3);
    plot(time, control_inputs, 'g-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Control Input');
    title('Control Inputs (Thrust/Torque)');
    grid on;
    
    subplot(2, 2, 4);
    pos_error = true_pos - est_pos;
    plot(time, sqrt(sum(pos_error.^2, 1)), 'r-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position Error (m)');
    title('EKF Position Error');
    grid on;
    
    sgtitle('Hover Control Test Results', 'FontSize', 14);
    saveas(gcf, 'hover_control_test.png');
    fprintf('  Plot saved: hover_control_test.png\n');
end

function test_waypoint_following(params)
    %% Test Waypoint Following Control
    
    rl_ekf = rl_ekf_integration(params);
    
    % Define waypoints
    waypoints = [0, 10, 20, 30; 0, 5, 10, 0; 10, 15, 20, 10];  % [x; y; z]
    current_waypoint = 1;
    
    true_state = [waypoints(:, 1); 0; 0; 0; 0; 0; 0];  % Start at first waypoint
    dt = params.Ts.physics;
    duration = 30;  % seconds
    steps = round(duration / dt);
    
    % Logging
    time = zeros(1, steps);
    true_pos = zeros(3, steps);
    est_pos = zeros(3, steps);
    waypoint_distances = zeros(1, steps);
    
    fprintf('  Testing waypoint following...\n');
    fprintf('  Waypoints: %d\n', size(waypoints, 2));
    
    for step = 1:steps
        t = step * dt;
        time(step) = t;
        
        % Generate velocity command towards current waypoint
        if current_waypoint <= size(waypoints, 2)
            target_pos = waypoints(:, current_waypoint);
            current_pos = true_state(1:3);
            direction = target_pos - current_pos;
            distance = norm(direction);
            
            if distance > 0.5  % Not too close
                direction = direction / distance;
                vel_cmd = [direction * 2; 0];  % 2 m/s towards waypoint
            else
                vel_cmd = [0; 0; 0; 0];  % Stop when close
            end
            
            waypoint_distances(step) = distance;
            
            % Check if reached waypoint
            if distance < 1.0  % Within 1m
                current_waypoint = current_waypoint + 1;
                fprintf('    Reached waypoint %d at t=%.1fs\n', current_waypoint-1, t);
            end
        else
            vel_cmd = [0; 0; 0; 0];  % All waypoints reached
        end
        
        % Apply control
        [new_true_state, ~] = rl_ekf.step(true_state, vel_cmd, dt);
        
        % Log data
        true_pos(:, step) = true_state(1:3);
        est_pos(:, step) = rl_ekf.ekf_state(1:3);
        
        % Update state
        true_state = new_true_state;
    end
    
    % Analyze results
    final_pos = true_pos(:, end);
    final_waypoint = waypoints(:, end);
    final_error = norm(final_pos - final_waypoint);
    
    fprintf('  Final position: [%.1f %.1f %.1f]\n', final_pos(1), final_pos(2), final_pos(3));
    fprintf('  Final waypoint: [%.1f %.1f %.1f]\n', final_waypoint(1), final_waypoint(2), final_waypoint(3));
    fprintf('  Final error: %.3f m\n', final_error);
    
    if final_error < 2.0
        fprintf('  ✅ Waypoint following working well\n');
    else
        fprintf('  ❌ Waypoint following needs improvement\n');
    end
    
    % Plot results
    figure('Position', [200, 200, 1000, 600]);
    
    subplot(2, 2, 1);
    plot3(waypoints(1, :), waypoints(2, :), waypoints(3, :), 'ko-', 'LineWidth', 2, 'MarkerSize', 8);
    hold on;
    plot3(true_pos(1, :), true_pos(2, :), true_pos(3, :), 'b-', 'LineWidth', 2);
    plot3(est_pos(1, :), est_pos(2, :), est_pos(3, :), 'r--', 'LineWidth', 1);
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('Waypoint Following: 3D Trajectory');
    legend('Waypoints', 'True Path', 'Estimated Path', 'Location', 'best');
    grid on;
    
    subplot(2, 2, 2);
    plot(time, waypoint_distances, 'g-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Distance to Waypoint (m)');
    title('Waypoint Following Progress');
    grid on;
    
    subplot(2, 2, 3);
    plot(time, true_pos, 'b-', 'LineWidth', 1);
    hold on;
    plot(time, est_pos, 'r--', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('Position Tracking');
    legend('True X', 'True Y', 'True Z', 'Est X', 'Est Y', 'Est Z', 'Location', 'best');
    grid on;
    
    subplot(2, 2, 4);
    pos_error = true_pos - est_pos;
    plot(time, sqrt(sum(pos_error.^2, 1)), 'r-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position Error (m)');
    title('EKF Position Error');
    grid on;
    
    sgtitle('Waypoint Following Test Results', 'FontSize', 14);
    saveas(gcf, 'waypoint_following_test.png');
    fprintf('  Plot saved: waypoint_following_test.png\n');
end

function test_ekf_estimation(params)
    %% Test EKF State Estimation Accuracy
    
    rl_ekf = rl_ekf_integration(params);
    
    % Test with known trajectory
    true_state = [0; 0; 10; 0; 0; 0; 0; 0; 0];
    dt = params.Ts.physics;
    duration = 20;  % seconds
    steps = round(duration / dt);
    
    % Logging
    time = zeros(1, steps);
    true_pos = zeros(3, steps);
    est_pos = zeros(3, steps);
    true_vel = zeros(3, steps);
    est_vel = zeros(3, steps);
    true_att = zeros(3, steps);
    est_att = zeros(3, steps);
    ekf_uncertainty = zeros(1, steps);
    
    fprintf('  Testing EKF state estimation...\n');
    
    for step = 1:steps
        t = step * dt;
        time(step) = t;
        
        % Simple trajectory (sine wave)
        vel_cmd = [2*sin(0.5*t); 0; 0; 0.1*sin(0.3*t)];
        
        % Apply control
        [new_true_state, uncertainty_info] = rl_ekf.step(true_state, vel_cmd, dt);
        
        % Log data
        true_pos(:, step) = true_state(1:3);
        est_pos(:, step) = rl_ekf.ekf_state(1:3);
        true_vel(:, step) = true_state(4:6);
        est_vel(:, step) = rl_ekf.ekf_state(4:6);
        true_att(:, step) = true_state(7:9);
        est_att(:, step) = rl_ekf.ekf_state(7:9);
        ekf_uncertainty(step) = trace(uncertainty_info.covariance);
        
        % Update state
        true_state = new_true_state;
    end
    
    % Analyze results
    pos_error = true_pos - est_pos;
    vel_error = true_vel - est_vel;
    att_error = true_att - est_att;
    
    pos_rmse = sqrt(mean(sum(pos_error.^2, 1)));
    vel_rmse = sqrt(mean(sum(vel_error.^2, 1)));
    att_rmse = sqrt(mean(sum(att_error.^2, 1)));
    
    fprintf('  Position RMSE: %.3f m\n', pos_rmse);
    fprintf('  Velocity RMSE: %.3f m/s\n', vel_rmse);
    fprintf('  Attitude RMSE: %.3f rad (%.1f deg)\n', att_rmse, rad2deg(att_rmse));
    fprintf('  Average EKF Uncertainty: %.3f\n', mean(ekf_uncertainty));
    
    if pos_rmse < 1.0 && vel_rmse < 0.5 && att_rmse < 0.1
        fprintf('  ✅ EKF estimation working well\n');
    else
        fprintf('  ❌ EKF estimation needs improvement\n');
    end
    
    % Plot results
    figure('Position', [300, 300, 1200, 800]);
    
    subplot(2, 3, 1);
    plot(time, true_pos, 'b-', 'LineWidth', 2);
    hold on;
    plot(time, est_pos, 'r--', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('Position Estimation');
    legend('True', 'Estimated', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 2);
    plot(time, true_vel, 'b-', 'LineWidth', 2);
    hold on;
    plot(time, est_vel, 'r--', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity Estimation');
    legend('True', 'Estimated', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 3);
    plot(time, rad2deg(true_att), 'b-', 'LineWidth', 2);
    hold on;
    plot(time, rad2deg(est_att), 'r--', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Attitude (deg)');
    title('Attitude Estimation');
    legend('True', 'Estimated', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 4);
    plot(time, sqrt(sum(pos_error.^2, 1)), 'r-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position Error (m)');
    title('Position Estimation Error');
    grid on;
    
    subplot(2, 3, 5);
    plot(time, sqrt(sum(vel_error.^2, 1)), 'r-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Velocity Error (m/s)');
    title('Velocity Estimation Error');
    grid on;
    
    subplot(2, 3, 6);
    plot(time, ekf_uncertainty, 'g-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('EKF Uncertainty');
    title('EKF Uncertainty');
    grid on;
    
    sgtitle('EKF State Estimation Test Results', 'FontSize', 14);
    saveas(gcf, 'ekf_estimation_test.png');
    fprintf('  Plot saved: ekf_estimation_test.png\n');
end
