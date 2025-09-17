%% expert_demonstrator_system.m - Expert Demonstrator with RRT* Path Planning
% PURPOSE
% Generate expert demonstrations using RRT* path planning + Pure Pursuit
% for autonomous obstacle avoidance with realistic EKF/UKF state estimation.

function expert_demonstrator_system()
    fprintf('=== Expert Demonstrator System ===\n\n');
    
    % Setup
    addpath('rl_obstacle_avoidance');
    addpath('.');
    
    % Load parameters
    run('parameters.m');
    params = rl_parameters();
    
    % Test 1: Expert Obstacle Avoidance with RRT*
    fprintf('Test 1: Expert Obstacle Avoidance with RRT*\n');
    test_expert_obstacle_avoidance(params);
    
    % Test 2: Expert Data Collection for RL Training
    fprintf('\nTest 2: Expert Data Collection for RL Training\n');
    collect_expert_demonstrations(params);
    
    % Test 3: EKF/UKF Integration with Expert System
    fprintf('\nTest 3: EKF/UKF Integration with Expert System\n');
    test_ekf_ukf_integration(params);
    
    fprintf('\n=== Expert Demonstrator System Complete ===\n');
end

function test_expert_obstacle_avoidance(params)
    %% Test Expert Obstacle Avoidance with RRT* + Pure Pursuit
    
    % Create expert flight controller
    expert_controller = expert_flight_controller(params);
    
    % Generate random obstacle map
    map_gen = expert_map_generator();
    obstacle_map = map_gen.generate_column_obstacles(0.3);  % 30% density
    
    % Define start and goal positions
    start_pos = [-30; -30; 5];  % Start position
    goal_pos = [30; 30; 15];    % Goal position
    
    % Test parameters
    duration = 60;  % seconds
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
    expert_vel = zeros(3, steps);
    expert_att = zeros(3, steps);
    control_inputs = zeros(4, steps);
    path_data = cell(1, steps);
    
    fprintf('  Testing expert obstacle avoidance for %.1f seconds...\n', duration);
    
    for step = 1:steps
        t = step * dt;
        time(step) = t;
        
        % Expert path planning and control
        [expert_state, control_inputs(:, step), path_info] = expert_controller.compute_expert_control(true_state, t);
        
        % Log expert state
        expert_pos(:, step) = expert_state(1:3);
        expert_vel(:, step) = expert_state(4:6);
        expert_att(:, step) = expert_state(7:9);
        path_data{step} = path_info;
        
        % Log true state
        true_pos(:, step) = true_state(1:3);
        
        % Simulate dynamics
        true_state = simulate_drone_dynamics(true_state, control_inputs(:, step), dt, params);
        
        % Check if goal reached
        if norm(true_state(1:3) - goal_pos) < 2.0
            fprintf('    Goal reached at t=%.1fs\n', t);
            break;
        end
    end
    
    % Analyze results
    final_pos = true_pos(:, step);
    final_error = norm(final_pos - goal_pos);
    path_length = calculate_path_length(expert_pos(:, 1:step));
    optimal_length = norm(goal_pos - start_pos);
    efficiency = optimal_length / path_length;
    
    fprintf('  Final position: [%.1f %.1f %.1f]\n', final_pos(1), final_pos(2), final_pos(3));
    fprintf('  Goal position: [%.1f %.1f %.1f]\n', goal_pos(1), goal_pos(2), goal_pos(3));
    fprintf('  Final error: %.3f m\n', final_error);
    fprintf('  Path efficiency: %.2f\n', efficiency);
    
    if final_error < 3.0 && efficiency > 0.7
        fprintf('  ✅ Expert obstacle avoidance: EXCELLENT\n');
    elseif final_error < 5.0 && efficiency > 0.5
        fprintf('  ✅ Expert obstacle avoidance: GOOD\n');
    else
        fprintf('  ⚠️  Expert obstacle avoidance: NEEDS IMPROVEMENT\n');
    end
    
    % Plot results
    plot_expert_results(time, true_pos, expert_pos, obstacle_map, start_pos, goal_pos, 'Expert Obstacle Avoidance');
end

function collect_expert_demonstrations(params)
    %% Collect Expert Demonstrations for RL Training
    
    % Create expert flight controller
    expert_controller = expert_flight_controller(params);
    
    % Create EKF/UKF for state estimation
    ekf_filter = ekf_sensor_only();
    ukf_filter = ukf9_step();
    
    % Collection parameters
    num_demonstrations = 10;
    duration = 45;  % seconds per demonstration
    dt = params.Ts.physics;
    steps = round(duration / dt);
    
    % Initialize data storage
    demonstrations = struct();
    demonstrations.observations = [];
    demonstrations.actions = [];
    demonstrations.rewards = [];
    demonstrations.next_observations = [];
    demonstrations.dones = [];
    
    fprintf('  Collecting %d expert demonstrations...\n', num_demonstrations);
    
    for demo = 1:num_demonstrations
        fprintf('    Demonstration %d/%d...\n', demo, num_demonstrations);
        
        % Generate random obstacle map
        map_gen = expert_map_generator();
        obstacle_map = map_gen.generate_column_obstacles(0.2 + 0.1*rand());  % 20-30% density
        
        % Random start and goal positions
        start_pos = [-35 + 10*rand(); -35 + 10*rand(); 5 + 5*rand()];
        goal_pos = [25 + 10*rand(); 25 + 10*rand(); 10 + 10*rand()];
        
        % Initialize expert system
        expert_controller.set_obstacle_map(obstacle_map);
        expert_controller.set_start_goal(start_pos, goal_pos);
        
        % Initialize filters
        ekf_state = [start_pos; zeros(6, 1)];
        ekf_covariance = eye(9) * 0.1;
        ukf_state = [start_pos; zeros(6, 1)];
        ukf_covariance = eye(9) * 0.1;
        
        % Initial state
        true_state = [start_pos; zeros(6, 1)];
        
        for step = 1:steps
            t = step * dt;
            
            % Expert path planning and control
            [expert_state, control_input, path_info] = expert_controller.compute_expert_control(true_state, t);
            
            % Simulate sensors (realistic, no ground truth)
            [imu_meas, gps_meas, baro_meas, mag_meas] = simulate_realistic_sensors(true_state, params);
            
            % EKF state estimation
            [ekf_state, ekf_covariance] = ekf_filter.step(ekf_state, ekf_covariance, imu_meas, gps_meas, baro_meas, mag_meas, dt);
            
            % UKF state estimation
            [ukf_state, ukf_covariance] = ukf_filter.step(ukf_state, ukf_covariance, imu_meas, gps_meas, baro_meas, mag_meas, dt);
            
            % Generate observation (EKF estimates + limited sensors)
            observation = generate_rl_observation(ekf_state, obstacle_map, true_state(1:3), goal_pos);
            
            % Calculate reward (expert reward shaping)
            reward = calculate_expert_reward(true_state, expert_state, goal_pos, obstacle_map);
            
            % Check if done
            done = (norm(true_state(1:3) - goal_pos) < 2.0) || (t >= duration);
            
            % Store demonstration data
            demonstrations.observations = [demonstrations.observations, observation];
            demonstrations.actions = [demonstrations.actions, control_input];
            demonstrations.rewards = [demonstrations.rewards, reward];
            demonstrations.dones = [demonstrations.dones, done];
            
            % Simulate dynamics
            true_state = simulate_drone_dynamics(true_state, control_input, dt, params);
            
            if done
                break;
            end
        end
    end
    
    % Save demonstrations
    save('expert_demonstrations.mat', 'demonstrations');
    fprintf('  ✅ Expert demonstrations saved: %d samples\n', length(demonstrations.observations));
end

function test_ekf_ukf_integration(params)
    %% Test EKF/UKF Integration with Expert System
    
    % Create expert flight controller
    expert_controller = expert_flight_controller(params);
    
    % Create EKF/UKF filters
    ekf_filter = ekf_sensor_only();
    ukf_filter = ukf9_step();
    
    % Test parameters
    duration = 30;  % seconds
    dt = params.Ts.physics;
    steps = round(duration / dt);
    
    % Generate obstacle map
    map_gen = expert_map_generator();
    obstacle_map = map_gen.generate_column_obstacles(0.25);
    
    % Start and goal
    start_pos = [-25; -25; 8];
    goal_pos = [25; 25; 12];
    
    % Initialize expert system
    expert_controller.set_obstacle_map(obstacle_map);
    expert_controller.set_start_goal(start_pos, goal_pos);
    
    % Initialize filters
    ekf_state = [start_pos; zeros(6, 1)];
    ekf_covariance = eye(9) * 0.1;
    ukf_state = [start_pos; zeros(6, 1)];
    ukf_covariance = eye(9) * 0.1;
    
    % Initial state
    true_state = [start_pos; zeros(6, 1)];
    
    % Logging
    time = zeros(1, steps);
    true_pos = zeros(3, steps);
    ekf_pos = zeros(3, steps);
    ukf_pos = zeros(3, steps);
    expert_pos = zeros(3, steps);
    
    fprintf('  Testing EKF/UKF integration for %.1f seconds...\n', duration);
    
    for step = 1:steps
        t = step * dt;
        time(step) = t;
        
        % Expert control
        [expert_state, control_input, ~] = expert_controller.compute_expert_control(true_state, t);
        
        % Simulate realistic sensors
        [imu_meas, gps_meas, baro_meas, mag_meas] = simulate_realistic_sensors(true_state, params);
        
        % EKF estimation
        [ekf_state, ekf_covariance] = ekf_filter.step(ekf_state, ekf_covariance, imu_meas, gps_meas, baro_meas, mag_meas, dt);
        
        % UKF estimation
        [ukf_state, ukf_covariance] = ukf_filter.step(ukf_state, ukf_covariance, imu_meas, gps_meas, baro_meas, mag_meas, dt);
        
        % Log states
        true_pos(:, step) = true_state(1:3);
        ekf_pos(:, step) = ekf_state(1:3);
        ukf_pos(:, step) = ukf_state(1:3);
        expert_pos(:, step) = expert_state(1:3);
        
        % Simulate dynamics
        true_state = simulate_drone_dynamics(true_state, control_input, dt, params);
        
        if norm(true_state(1:3) - goal_pos) < 2.0
            fprintf('    Goal reached at t=%.1fs\n', t);
            break;
        end
    end
    
    % Analyze estimation accuracy
    ekf_error = sqrt(mean(sum((true_pos - ekf_pos).^2, 1)));
    ukf_error = sqrt(mean(sum((true_pos - ukf_pos).^2, 1)));
    
    fprintf('  EKF position error: %.3f m\n', ekf_error);
    fprintf('  UKF position error: %.3f m\n', ukf_error);
    
    if ekf_error < 1.0 && ukf_error < 1.0
        fprintf('  ✅ EKF/UKF integration: EXCELLENT\n');
    elseif ekf_error < 2.0 && ukf_error < 2.0
        fprintf('  ✅ EKF/UKF integration: GOOD\n');
    else
        fprintf('  ⚠️  EKF/UKF integration: NEEDS IMPROVEMENT\n');
    end
    
    % Plot results
    plot_ekf_ukf_results(time, true_pos, ekf_pos, ukf_pos, expert_pos, 'EKF/UKF Integration');
end

function path_length = calculate_path_length(positions)
    %% Calculate total path length
    path_length = 0;
    for i = 2:size(positions, 2)
        path_length = path_length + norm(positions(:, i) - positions(:, i-1));
    end
end

function [imu_meas, gps_meas, baro_meas, mag_meas] = simulate_realistic_sensors(true_state, params)
    %% Simulate realistic sensors (no ground truth)
    
    % Extract true state
    pos = true_state(1:3);
    vel = true_state(4:6);
    att = true_state(7:9);
    
    % IMU measurements
    imu_meas = struct();
    imu_meas.accel = [0; 0; -params.g(3)] + randn(3, 1) * 0.1;
    imu_meas.gyro = randn(3, 1) * 0.01;
    
    % GPS measurements
    gps_meas = struct();
    gps_meas.pos = pos + randn(3, 1) * params.GPS.sigma_xy;
    gps_meas.vel = vel + randn(3, 1) * 0.1;
    
    % Barometer measurements
    baro_meas = -pos(3) + randn() * params.Baro.sigma_z;
    
    % Magnetometer measurements
    mag_meas = params.mag_NED + randn(3, 1) * params.Mag.sigma_rad;
end

function observation = generate_rl_observation(ekf_state, obstacle_map, true_pos, goal_pos)
    %% Generate RL observation from EKF estimates and limited sensors
    
    % EKF state estimate
    ekf_pos = ekf_state(1:3);
    ekf_vel = ekf_state(4:6);
    ekf_att = ekf_state(7:9);
    
    % Relative goal position
    rel_goal = goal_pos - ekf_pos;
    
    % Limited sensor data (lidar-like ranges)
    num_beams = 16;
    max_range = 15.0;
    ranges = zeros(num_beams, 1);
    
    for i = 1:num_beams
        angle = (i-1) * 2*pi / num_beams;
        direction = [cos(angle); sin(angle); 0];
        
        % Simple ray casting for range measurement
        ranges(i) = raycast_range(ekf_pos, direction, obstacle_map, max_range);
    end
    
    % Combine into observation vector
    observation = [ekf_pos; ekf_vel; ekf_att; rel_goal; ranges];
end

function range = raycast_range(pos, direction, obstacle_map, max_range)
    %% Simple ray casting for range measurement
    
    range = max_range;
    step_size = 0.5;
    steps = round(max_range / step_size);
    
    for i = 1:steps
        test_pos = pos + direction * i * step_size;
        
        % Check collision with obstacles
        if check_collision(test_pos, obstacle_map)
            range = i * step_size;
            break;
        end
    end
end

function collision = check_collision(pos, obstacle_map)
    %% Check collision with obstacles
    
    collision = false;
    
    % Check cylindrical obstacles
    for i = 1:length(obstacle_map.cylinders)
        cyl = obstacle_map.cylinders{i};
        dist_xy = norm(pos(1:2) - cyl.center(1:2));
        
        if dist_xy < cyl.radius && pos(3) >= cyl.center(3) && pos(3) <= cyl.center(3) + cyl.height
            collision = true;
            return;
        end
    end
end

function reward = calculate_expert_reward(true_state, expert_state, goal_pos, obstacle_map)
    %% Calculate expert reward for RL training
    
    pos = true_state(1:3);
    vel = true_state(4:6);
    
    % Goal reaching reward
    goal_distance = norm(pos - goal_pos);
    goal_reward = exp(-goal_distance / 10.0) * 10.0;
    
    % Progress reward
    progress_reward = -goal_distance * 0.1;
    
    % Collision penalty
    collision_penalty = 0;
    if check_collision(pos, obstacle_map)
        collision_penalty = -100.0;
    end
    
    % Smoothness reward
    smoothness_reward = -norm(vel) * 0.01;
    
    % Total reward
    reward = goal_reward + progress_reward + collision_penalty + smoothness_reward;
end

function plot_expert_results(time, true_pos, expert_pos, obstacle_map, start_pos, goal_pos, title_str)
    %% Plot Expert Results
    
    figure('Position', [100, 100, 1200, 800]);
    
    subplot(2, 3, 1);
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
    legend('Expert Path', 'Start', 'Goal', 'Location', 'best');
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
    
    sgtitle(title_str, 'FontSize', 14);
    saveas(gcf, 'expert_obstacle_avoidance_test.png');
    fprintf('  Plot saved: expert_obstacle_avoidance_test.png\n');
end

function plot_ekf_ukf_results(time, true_pos, ekf_pos, ukf_pos, expert_pos, title_str)
    %% Plot EKF/UKF Results
    
    figure('Position', [200, 200, 1200, 800]);
    
    subplot(2, 3, 1);
    plot3(true_pos(1, :), true_pos(2, :), true_pos(3, :), 'b-', 'LineWidth', 2);
    hold on;
    plot3(ekf_pos(1, :), ekf_pos(2, :), ekf_pos(3, :), 'r--', 'LineWidth', 1);
    plot3(ukf_pos(1, :), ukf_pos(2, :), ukf_pos(3, :), 'g--', 'LineWidth', 1);
    plot3(expert_pos(1, :), expert_pos(2, :), expert_pos(3, :), 'k:', 'LineWidth', 1);
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('3D Trajectory Comparison');
    legend('True', 'EKF', 'UKF', 'Expert', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 2);
    plot(time, true_pos, 'b-', 'LineWidth', 1);
    hold on;
    plot(time, ekf_pos, 'r--', 'LineWidth', 1);
    plot(time, ukf_pos, 'g--', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('Position Estimation');
    legend('True', 'EKF', 'UKF', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 3);
    ekf_error = sqrt(sum((true_pos - ekf_pos).^2, 1));
    ukf_error = sqrt(sum((true_pos - ukf_pos).^2, 1));
    plot(time, ekf_error, 'r-', 'LineWidth', 1);
    hold on;
    plot(time, ukf_error, 'g-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position Error (m)');
    title('Estimation Error');
    legend('EKF', 'UKF', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 4);
    plot(time, sqrt(sum(true_pos.^2, 1)), 'b-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Distance from Origin (m)');
    title('True Path Length');
    grid on;
    
    subplot(2, 3, 5);
    plot(time, sqrt(sum(ekf_pos.^2, 1)), 'r-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('EKF Distance from Origin (m)');
    title('EKF Path Length');
    grid on;
    
    subplot(2, 3, 6);
    plot(time, sqrt(sum(ukf_pos.^2, 1)), 'g-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('UKF Distance from Origin (m)');
    title('UKF Path Length');
    grid on;
    
    sgtitle(title_str, 'FontSize', 14);
    saveas(gcf, 'ekf_ukf_integration_test.png');
    fprintf('  Plot saved: ekf_ukf_integration_test.png\n');
end

function true_state = simulate_drone_dynamics(current_state, control_inputs, dt, params)
    %% Drone Dynamics Simulation (Fixed)
    
    % Extract control inputs
    thrust = control_inputs(1);
    torque = control_inputs(2:4);
    
    % Extract current state
    pos = current_state(1:3);
    vel = current_state(4:6);
    att = current_state(7:9);
    
    % Position integration
    new_pos = pos + vel * dt;
    
    % Velocity integration with proper gravity compensation
    gravity = [0; 0; params.g(3)];  % Gravity vector (positive down in NED)
    
    % Convert thrust to world frame using rotation matrix
    R = rotation_matrix(att(1), att(2), att(3));
    thrust_body = [0; 0; -thrust];  % Thrust in body frame (upward)
    thrust_world = R * thrust_body;  % Convert to world frame
    
    % Total acceleration
    accel = gravity + thrust_world / params.mass;
    
    % Add damping
    damping = -0.1 * vel;
    accel = accel + damping;
    
    new_vel = vel + accel * dt;
    
    % Attitude integration
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
