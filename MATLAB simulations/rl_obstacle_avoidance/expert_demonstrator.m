%% expert_demonstrator.m - Expert Demonstration Collection System
% PURPOSE
% Collects expert demonstrations from EKF-guided drone navigation for use in
% imitation learning and demonstration-augmented RL. The "expert" is the
% EKF-equipped drone with full sensors navigating optimally.
%
% This implements the PDF requirement for using EKF-driven trajectories
% as training data for limited-sensor drones.

classdef expert_demonstrator < handle
    properties
        params
        environment
        limited_observer
        
        % Demonstration storage
        demonstrations
        demo_buffer_size
        demo_count
        
        % Expert policy (EKF-based)
        expert_controller
        path_planner
        
        % Collection statistics
        success_rate
        collection_stats
    end
    
    methods
        function obj = expert_demonstrator(params, environment)
            obj.params = params;
            obj.environment = environment;
            
            % Initialize limited sensor observer
            obj.limited_observer = limited_sensor_observer(params);
            
            % Initialize demonstration buffer
            obj.demo_buffer_size = params.rl.expert.demo_buffer_size;
            obj.demonstrations = cell(obj.demo_buffer_size, 1);
            obj.demo_count = 0;
            
            % Initialize expert controller
            obj.initialize_expert_controller();
            
            % Statistics
            obj.success_rate = 0;
            obj.collection_stats = struct();
            
            fprintf('Expert demonstrator initialized\n');
            fprintf('  Buffer size: %d demonstrations\n', obj.demo_buffer_size);
            fprintf('  Sensor mode: %s\n', params.rl.sensor_mode);
        end
        
        function collect_demonstrations(obj, num_episodes)
            %% Collect Expert Demonstrations
            
            fprintf('Collecting %d expert demonstrations...\n', num_episodes);
            
            successful_demos = 0;
            
            for episode = 1:num_episodes
                % Reset environment
                obj.environment.reset();
                
                % Collect single demonstration
                demo_data = obj.collect_single_demonstration();
                
                if ~isempty(demo_data) && demo_data.success
                    % Store demonstration
                    obj.store_demonstration(demo_data);
                    successful_demos = successful_demos + 1;
                    
                    if mod(successful_demos, 50) == 0
                        fprintf('  Collected %d successful demonstrations\n', successful_demos);
                    end
                end
                
                if obj.demo_count >= obj.demo_buffer_size
                    fprintf('Demo buffer full, stopping collection\n');
                    break;
                end
            end
            
            obj.success_rate = successful_demos / episode;
            fprintf('Collection complete: %d/%d successful (%.1f%%)\n', ...
                    successful_demos, episode, obj.success_rate * 100);
        end
        
        function demo_data = collect_single_demonstration(obj)
            %% Collect Single Expert Demonstration
            
            % Initialize demonstration data
            demo_data = struct();
            demo_data.states = [];
            demo_data.limited_observations = [];
            demo_data.actions = [];
            demo_data.rewards = [];
            demo_data.success = false;
            demo_data.episode_length = 0;
            
            max_steps = obj.environment.max_episode_steps;
            
            for step = 1:max_steps
                % Get current state
                current_state = obj.environment.drone_state;
                
                % Generate limited sensor observation (what the learning agent sees)
                limited_obs = obj.limited_observer.generate_limited_observation(...
                    obj.environment.occupancy_grid, current_state, ...
                    obj.environment.map_bounds, obj.environment.map_resolution);
                
                % Expert action (EKF-guided optimal policy)
                expert_action = obj.get_expert_action(current_state);
                
                % Execute action in environment
                [next_obs, reward, done, info] = obj.environment.step(expert_action);
                
                % Store experience
                demo_data.states = [demo_data.states, current_state];
                demo_data.limited_observations = [demo_data.limited_observations, limited_obs];
                demo_data.actions = [demo_data.actions, expert_action];
                demo_data.rewards = [demo_data.rewards, reward];
                
                demo_data.episode_length = demo_data.episode_length + 1;
                
                % Check termination
                if done
                    demo_data.success = info.goal_reached;
                    break;
                end
            end
            
            % Only return if successful
            if ~demo_data.success
                demo_data = [];
            end
        end
        
        function action = get_expert_action(obj, current_state)
            %% Generate Expert Action Using EKF-Based Policy
            
            % Get current position and goal
            current_pos = current_state(1:3);
            goal_pos = obj.environment.target_waypoints(:, end);
            
            % Get current waypoint
            current_waypoint = obj.environment.target_waypoints(:, ...
                min(obj.environment.current_waypoint_idx, size(obj.environment.target_waypoints, 2)));
            
            % Expert policy: sophisticated path following with obstacle avoidance
            action = obj.expert_path_following_policy(current_state, current_waypoint, goal_pos);
            
            % Add some exploration noise for diversity (small amount)
            if obj.params.rl.advanced.domain_randomization
                exploration_noise = 0.05 * randn(size(action));  % 5% noise
                action = action + exploration_noise;
                action = max(min(action, 1), -1);  % Clamp to [-1, 1]
            end
        end
        
        function action = expert_path_following_policy(obj, current_state, target_waypoint, final_goal)
            %% Expert Path Following Policy
            
            % Extract state components
            pos = current_state(1:3);
            vel = current_state(4:6);
            att = current_state(7:9);
            
            % 1) Path following component
            to_waypoint = target_waypoint - pos;
            distance_to_waypoint = norm(to_waypoint);
            
            if distance_to_waypoint > 0.1
                desired_direction = to_waypoint / distance_to_waypoint;
            else
                % Close to waypoint, head to final goal
                to_goal = final_goal - pos;
                if norm(to_goal) > 0.1
                    desired_direction = to_goal / norm(to_goal);
                else
                    desired_direction = [0; 0; 0];
                end
            end
            
            % 2) Obstacle avoidance component
            avoidance_force = obj.calculate_obstacle_avoidance_force(pos);
            
            % 3) Combine path following and avoidance
            desired_velocity = 3.0 * desired_direction + 2.0 * avoidance_force;  % Max 3 m/s forward, 2 m/s avoidance
            
            % 4) Velocity control
            vel_error = desired_velocity - vel;
            
            % Convert to normalized action
            max_vel = obj.params.rl.max_velocity;
            max_yaw_rate = obj.params.rl.max_yaw_rate;
            
            action = [
                vel_error(1) / max_vel;
                vel_error(2) / max_vel;
                vel_error(3) / max_vel;
                0.0  % No yaw rate for now
            ];
            
            % 5) Yaw alignment (point towards desired direction)
            if norm(desired_direction(1:2)) > 0.1
                desired_yaw = atan2(desired_direction(2), desired_direction(1));
                yaw_error = atan2(sin(desired_yaw - att(3)), cos(desired_yaw - att(3)));
                yaw_rate = 2.0 * yaw_error;  % P controller
                action(4) = yaw_rate / max_yaw_rate;
            end
            
            % Clamp to valid range
            action = max(min(action, 1), -1);
        end
        
        function avoidance_force = calculate_obstacle_avoidance_force(obj, position)
            %% Calculate Obstacle Avoidance Force Using Artificial Potential Field
            
            % Get occupancy grid information
            occupancy_grid = obj.environment.occupancy_grid;
            map_bounds = obj.environment.map_bounds;
            map_resolution = obj.environment.map_resolution;
            
            % Convert position to grid coordinates
            x_idx = round((position(1) - map_bounds(1)) / map_resolution) + 1;
            y_idx = round((position(2) - map_bounds(3)) / map_resolution) + 1;
            z_idx = round((position(3) - map_bounds(5)) / map_resolution) + 1;
            
            [x_size, y_size, z_size] = size(occupancy_grid);
            
            % Initialize avoidance force
            avoidance_force = [0; 0; 0];
            
            % Check nearby obstacles
            search_radius = round(5.0 / map_resolution);  % 5m search radius
            
            for dx = -search_radius:search_radius
                for dy = -search_radius:search_radius
                    for dz = -search_radius:search_radius
                        xi = x_idx + dx;
                        yi = y_idx + dy;
                        zi = z_idx + dz;
                        
                        % Check bounds
                        if xi >= 1 && xi <= x_size && yi >= 1 && yi <= y_size && zi >= 1 && zi <= z_size
                            if occupancy_grid(xi, yi, zi) == 1
                                % Calculate repulsive force
                                obstacle_pos = [
                                    map_bounds(1) + (xi - 1) * map_resolution;
                                    map_bounds(3) + (yi - 1) * map_resolution;
                                    map_bounds(5) + (zi - 1) * map_resolution
                                ];
                                
                                to_drone = position - obstacle_pos;
                                distance = norm(to_drone);
                                
                                if distance > 0.1 && distance < 5.0  % Within influence range
                                    force_magnitude = 1.0 / (distance^2);  % Inverse square law
                                    force_direction = to_drone / distance;
                                    avoidance_force = avoidance_force + force_magnitude * force_direction;
                                end
                            end
                        end
                    end
                end
            end
            
            % Limit maximum avoidance force
            max_force = 2.0;
            if norm(avoidance_force) > max_force
                avoidance_force = max_force * avoidance_force / norm(avoidance_force);
            end
        end
        
        function store_demonstration(obj, demo_data)
            %% Store Demonstration in Buffer
            
            if obj.demo_count < obj.demo_buffer_size
                obj.demo_count = obj.demo_count + 1;
                obj.demonstrations{obj.demo_count} = demo_data;
            else
                % Replace oldest demonstration (circular buffer)
                idx = mod(obj.demo_count, obj.demo_buffer_size) + 1;
                obj.demonstrations{idx} = demo_data;
                obj.demo_count = obj.demo_count + 1;
            end
        end
        
        function [states, limited_obs, actions, rewards] = get_demonstration_batch(obj, batch_size)
            %% Get Random Batch of Demonstrations for Training
            
            if obj.demo_count == 0
                error('No demonstrations collected');
            end
            
            % Sample random demonstrations
            available_demos = min(obj.demo_count, obj.demo_buffer_size);
            demo_indices = randperm(available_demos, min(batch_size, available_demos));
            
            % Extract data
            states = [];
            limited_obs = [];
            actions = [];
            rewards = [];
            
            for i = 1:length(demo_indices)
                demo = obj.demonstrations{demo_indices(i)};
                
                % Sample random transitions from this demonstration
                demo_length = size(demo.states, 2);
                if demo_length > 1
                    num_samples = min(10, demo_length);  % Up to 10 samples per demo
                    sample_indices = randperm(demo_length, num_samples);
                    
                    states = [states, demo.states(:, sample_indices)];
                    limited_obs = [limited_obs, demo.limited_observations(:, sample_indices)];
                    actions = [actions, demo.actions(:, sample_indices)];
                    rewards = [rewards, demo.rewards(sample_indices)];
                end
            end
        end
        
        function save_demonstrations(obj, filename)
            %% Save Demonstrations to File
            
            demo_data = struct();
            demo_data.demonstrations = obj.demonstrations(1:min(obj.demo_count, obj.demo_buffer_size));
            demo_data.collection_params = obj.params.rl;
            demo_data.success_rate = obj.success_rate;
            demo_data.demo_count = obj.demo_count;
            
            save(filename, 'demo_data');
            fprintf('Saved %d demonstrations to: %s\n', obj.demo_count, filename);
        end
        
        function load_demonstrations(obj, filename)
            %% Load Demonstrations from File
            
            loaded = load(filename);
            demo_data = loaded.demo_data;
            
            obj.demonstrations = demo_data.demonstrations;
            obj.demo_count = demo_data.demo_count;
            obj.success_rate = demo_data.success_rate;
            
            fprintf('Loaded %d demonstrations from: %s\n', obj.demo_count, filename);
        end
        
        function initialize_expert_controller(obj)
            %% Initialize Expert Controller Components
            
            % Simple initialization for now
            obj.expert_controller = struct();
            obj.expert_controller.gains = struct();
            obj.expert_controller.gains.Kp_pos = 2.0;
            obj.expert_controller.gains.Kp_vel = 1.0;
            obj.expert_controller.gains.Kp_yaw = 3.0;
            
            obj.path_planner = struct();
            obj.path_planner.lookahead_distance = 5.0;
            obj.path_planner.obstacle_avoidance_gain = 2.0;
        end
    end
end
