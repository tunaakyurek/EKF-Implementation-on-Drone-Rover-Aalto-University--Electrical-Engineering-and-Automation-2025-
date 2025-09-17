%% rl_environment.m - Reinforcement Learning Environment for Drone Obstacle Avoidance
% PURPOSE
% Creates a comprehensive RL training environment for autonomous drone navigation
% with obstacle avoidance using only map data and trajectory information.
% No direct obstacle sensors - the agent must learn spatial awareness from maps.
%
% FEATURES
% - 3D obstacle-filled environments with varying terrain
% - Map-based perception (occupancy grids, height maps)
% - Trajectory-based reward system for navigation efficiency
% - Integration with EKF state estimation for realistic sensing
% - Blind drone simulation (no direct distance/lidar sensors)
%
% USAGE
%   env = rl_environment(params);
%   [obs, reward, done, info] = env.step(action);
%   obs = env.reset();

classdef rl_environment < handle
    properties
        % Environment parameters
        params
        
        % Map and terrain
        occupancy_grid      % 3D occupancy grid [x, y, z]
        height_map         % 2D height map for terrain
        map_resolution     % meters per grid cell
        map_bounds         % [x_min, x_max, y_min, y_max, z_min, z_max]
        omap3d             % occupancyMap3D for planning
        
        % Drone state and dynamics
        drone_state        % [pos(3); vel(3); att(3)] - current state
        target_waypoints   % [3 x N] target trajectory points
        current_waypoint_idx
        traj_T             % trajectory time vector
        traj_Q             % 3xM trajectory positions
        traj_Qd            % 3xM trajectory velocities
        haveWaypointFollower
        vfh_controller
        
        % EKF integration
        ekf_state
        ekf_covariance
        sensor_history
        rl_ekf_integration  % Integration layer
        
        % Episode management
        episode_step
        max_episode_steps
        collision_occurred
        goal_reached
        
        % Observation and action spaces
        observation_dim
        action_dim
        action_bounds
        
        % Reward system
        previous_distance_to_goal
        path_efficiency_history
        collision_penalty
        goal_reward
        
        % Visualization
        fig_handle
        trajectory_history
    end
    
    methods
        function obj = rl_environment(params)
            %% Initialize RL Environment
            obj.params = params;
            obj.setup_environment();
            obj.reset();
        end
        
        function setup_environment(obj)
            %% Setup Environment Parameters and Spaces
            
            % Map parameters (adjustable terrain complexity)
            obj.map_resolution = obj.params.rl.map_resolution; % 0.5m per cell
            obj.map_bounds = obj.params.rl.map_bounds; % [x_min, x_max, y_min, y_max, z_min, z_max]
            
            % Create observation space (map-based + state + trajectory info)
            % Observation includes:
            % - Local occupancy grid around drone (16x16x8 voxels)
            % - Current EKF state estimate (9 elements)
            % - Target waypoint relative position (3 elements)
            % - EKF uncertainty (9 diagonal covariance elements)
            % - Previous N trajectory points (3*N elements)
            local_map_size = 16 * 16 * 8;   % Reduced local 3D occupancy grid (16x16x8)
            state_size = 9;                 % EKF state
            target_size = 3;                % Relative target position
            uncertainty_size = 9;           % EKF diagonal covariance
            trajectory_history_size = 3 * 10; % Last 10 trajectory points
            
            obj.observation_dim = local_map_size + state_size + target_size + ...
                                uncertainty_size + trajectory_history_size;
            
            % Action space: velocity commands in body frame
            % [v_x, v_y, v_z, yaw_rate] - continuous actions
            obj.action_dim = 4;
            obj.action_bounds = [
                -obj.params.rl.max_velocity, obj.params.rl.max_velocity;  % v_x
                -obj.params.rl.max_velocity, obj.params.rl.max_velocity;  % v_y
                -obj.params.rl.max_velocity, obj.params.rl.max_velocity;  % v_z
                -obj.params.rl.max_yaw_rate, obj.params.rl.max_yaw_rate   % yaw_rate
            ];
            
            % Episode parameters
            obj.max_episode_steps = obj.params.rl.max_episode_steps;
            
            % Reward system parameters
            obj.collision_penalty = -100;
            obj.goal_reward = 100;
            
            fprintf('RL Environment initialized:\n');
            fprintf('  Observation dim: %d\n', obj.observation_dim);
            fprintf('  Action dim: %d\n', obj.action_dim);
            fprintf('  Map resolution: %.2f m/cell\n', obj.map_resolution);
            fprintf('  Map bounds: [%.1f %.1f %.1f %.1f %.1f %.1f]\n', obj.map_bounds);
            
            % Initialize guidance availability and VFH
            obj.haveWaypointFollower = (exist('uavWaypointFollower','class')==8);
            obj.vfh_controller = controllerVFH('DistanceLimits',[0.2 6.0], 'RobotRadius',0.3, 'SafetyDistance',0.4, 'NumAngularSectors',72);
        end
        
        function obs = reset(obj)
            %% Reset Environment for New Episode
            
            % Generate new map and obstacles
            obj.generate_map();
            obj.build_occupancy_map3d();
            
            % Set random start position (collision-free)
            obj.drone_state = obj.sample_collision_free_state();
            
            % Generate target waypoints for navigation task
            obj.generate_target_trajectory();
            obj.plan_and_smooth_trajectory();
            obj.current_waypoint_idx = 1;
            
            % Initialize EKF
            obj.initialize_ekf();
            
            % Reset episode tracking
            obj.episode_step = 0;
            obj.collision_occurred = false;
            obj.goal_reached = false;
            obj.trajectory_history = obj.drone_state(1:3);
            obj.path_efficiency_history = [];
            
            % Calculate initial distance to goal
            obj.previous_distance_to_goal = norm(obj.drone_state(1:3) - obj.target_waypoints(:, end));
            
            % Generate observation
            obs = obj.generate_observation();
            
            fprintf('Episode reset. Start: [%.1f %.1f %.1f], Goal: [%.1f %.1f %.1f]\n', ...
                obj.drone_state(1:3), obj.target_waypoints(:, end));
        end
        
        function [obs, reward, done, info] = step(obj, action)
            %% Execute Action and Return New State
            
            obj.episode_step = obj.episode_step + 1;
            
            % Clamp actions to bounds
            action = max(min(action, obj.action_bounds(:,2)), obj.action_bounds(:,1));

            % Guidance: compute reference velocity/yaw from smoothed trajectory and VFH
            dt = obj.params.Ts.physics;
            t_now = (obj.episode_step-1) * dt;
            v_ref = [0;0;0]; yaw_rate_ref = 0;
            if ~isempty(obj.traj_T)
                stepT = max(obj.traj_T(2)-obj.traj_T(1), dt);
                idx = min(numel(obj.traj_T), max(1, round(t_now/stepT)+1));
                p_ref = obj.traj_Q(:,idx); v_ref_traj = obj.traj_Qd(:,idx);
                dir_vec = (p_ref - obj.ekf_state(1:3)); dir_vec(3) = 0;
                desiredHeading = atan2(dir_vec(2), dir_vec(1));
                scanAngles = linspace(-pi, pi, 72); ranges = 10*ones(size(scanAngles));
                steerDir = obj.vfh_controller(ranges, scanAngles, desiredHeading);
                if isnan(steerDir), steerDir = desiredHeading; end
                speedCmd = min(obj.params.rl.max_velocity, max(0.5, norm(v_ref_traj(1:2))));
                v_ref_xy = speedCmd * [cos(steerDir); sin(steerDir)];
                v_ref = [v_ref_xy; v_ref_traj(3)];
                yaw_err = atan2(sin(steerDir - obj.ekf_state(9)), cos(steerDir - obj.ekf_state(9)));
                yaw_rate_ref = max(min(yaw_err / max(dt,1e-3), obj.params.rl.max_yaw_rate), -obj.params.rl.max_yaw_rate);
            end
            % Combine guidance with agent action (agent outputs deltas)
            action_combined = [v_ref(1:3); yaw_rate_ref] + action;
            action_combined = max(min(action_combined, obj.action_bounds(:,2)), obj.action_bounds(:,1));

            % Apply action to drone dynamics (with realistic physics)
            obj.apply_action(action_combined);
            
            % Simulate sensors and update EKF
            obj.simulate_sensors_and_ekf();
            
            % Check for collisions
            obj.collision_occurred = obj.check_collision();
            
            % Check if goal reached
            obj.goal_reached = obj.check_goal_reached();
            
            % Calculate reward
            reward = obj.calculate_reward(action);
            
            % Check termination conditions
            done = obj.collision_occurred || obj.goal_reached || ...
                   obj.episode_step >= obj.max_episode_steps || ...
                   obj.check_out_of_bounds();
            
            % Generate new observation
            obs = obj.generate_observation();
            
            % Create info structure
            info = struct();
            info.collision = obj.collision_occurred;
            info.goal_reached = obj.goal_reached;
            info.episode_step = obj.episode_step;
            info.distance_to_goal = norm(obj.drone_state(1:3) - obj.target_waypoints(:, end));
            info.ekf_uncertainty = trace(obj.ekf_covariance);
            info.estimated_state = obj.ekf_state;
            info.covariance = obj.ekf_covariance;
            
            % Update trajectory history
            obj.trajectory_history = [obj.trajectory_history, obj.drone_state(1:3)];
        end
        
        function generate_map(obj)
            %% Generate 3D Map with Obstacles and Terrain
            
            bounds = obj.map_bounds;
            res = obj.map_resolution;
            
            % Calculate grid dimensions
            x_cells = ceil((bounds(2) - bounds(1)) / res);
            y_cells = ceil((bounds(4) - bounds(3)) / res);
            z_cells = ceil((bounds(6) - bounds(5)) / res);
            
            % Initialize occupancy grid (0 = free, 1 = occupied)
            obj.occupancy_grid = zeros(x_cells, y_cells, z_cells);
            
            % Generate terrain height map
            [X, Y] = meshgrid(linspace(bounds(1), bounds(2), x_cells), ...
                            linspace(bounds(3), bounds(4), y_cells));
            
            % Create varied terrain with multiple difficulty levels
            terrain_complexity = obj.params.rl.terrain_complexity; % 0-1 scale
            
            % Base terrain with fractal noise
            obj.height_map = obj.generate_fractal_terrain(X, Y, terrain_complexity);
            
            % Add specific obstacle types based on scenario
            obstacle_scenario = obj.params.rl.obstacle_scenario;
            switch obstacle_scenario
                case 'forest'
                    obj.add_forest_obstacles();
                case 'urban'
                    obj.add_urban_obstacles();
                case 'canyon'
                    obj.add_canyon_obstacles();
                case 'mixed'
                    obj.add_mixed_obstacles();
                otherwise
                    obj.add_random_obstacles();
            end
            
            % Ensure boundaries are occupied (walls)
            obj.add_boundary_walls();
        end

        function build_occupancy_map3d(obj)
            %% Build occupancyMap3D from grid for planning
            try
                res = obj.map_resolution;
                obj.omap3d = occupancyMap3D(res);
                % Fill free space volume first
                fx = obj.map_bounds(1):res:obj.map_bounds(2);
                fy = obj.map_bounds(3):res:obj.map_bounds(4);
                fz = obj.map_bounds(5):res:obj.map_bounds(6);
                [Xf,Yf,Zf] = ndgrid(fx,fy,fz);
                setOccupancy(obj.omap3d, [Xf(:) Yf(:) Zf(:)], 0);
                % Occupied cells from occupancy_grid
                [xi, yi, zi] = ind2sub(size(obj.occupancy_grid), find(obj.occupancy_grid==1));
                if ~isempty(xi)
                    pts = [obj.map_bounds(1) + (xi-1)*res, ...
                           obj.map_bounds(3) + (yi-1)*res, ...
                           obj.map_bounds(5) + (zi-1)*res];
                    setOccupancy(obj.omap3d, pts, 1);
                end
                inflate(obj.omap3d, 0.4);
            catch
                obj.omap3d = [];
            end
        end

        function plan_and_smooth_trajectory(obj)
            %% Plan SE(3) path with RRT* and smooth with cubicpolytraj
            try
                if isempty(obj.omap3d)
                    obj.traj_T = []; obj.traj_Q = []; obj.traj_Qd = [];
                    return;
                end
                ss = stateSpaceSE3;
                b = obj.map_bounds;
                ss.StateBounds = [b(1:2)'; b(3:4)'; b(5:6)'; -1 1; -1 1; -1 1; -1 1];
                sv = validatorOccupancyMap3D(ss);
                sv.Map = obj.omap3d; sv.ValidationDistance = 0.5;
                planner = plannerRRTStar(ss, sv, 'MaxConnectionDistance', 3.0, 'GoalBias', 0.15, 'MaxIterations', 4000);
                start = [obj.drone_state(1:3).' 1 0 0 0];
                goal = [obj.target_waypoints(:,end).' 1 0 0 0];
                [pth, ~] = plan(planner, start, goal);
                raw = pth.States(:,1:3);
                pts = obj.shortcut_path(raw);
                s = [0; cumsum(vecnorm(diff(pts),2,2))]; total = s(end);
                v_nom = min(2.0, obj.params.rl.max_velocity);
                tf = max(1.0, total / max(0.5, v_nom));
                M = max(25, ceil(total/max(obj.map_resolution,0.5)));
                tvec = linspace(0, tf, M);
                Q = cubicpolytraj(pts.', s/s(end)*tf, tvec);
                dt = max(1e-3, tvec(2)-tvec(1));
                Qd = gradient(Q, dt);
                obj.traj_T = tvec; obj.traj_Q = Q; obj.traj_Qd = Qd;
            catch
                obj.traj_T = []; obj.traj_Q = []; obj.traj_Qd = [];
            end
        end

        function path = shortcut_path(obj, pts)
            %% Simple shortcutting over occupancy map
            if size(pts,1) < 3 || isempty(obj.omap3d), path = pts; return; end
            out = pts(1,:); i = 1; n = size(pts,1);
            while i < n
                j = n;
                while j > i+1
                    if obj.line_free(pts(i,:), pts(j,:))
                        out(end+1,:) = pts(j,:); i = j; break; %#ok<AGROW>
                    end
                    j = j - 1;
                end
                if j == i+1
                    out(end+1,:) = pts(j,:); i = j; %#ok<AGROW>
                end
            end
            path = out;
        end

        function ok = line_free(obj, p, q)
            if isempty(obj.omap3d), ok = true; return; end
            n = max(2, ceil(norm(q-p)/obj.map_resolution));
            s = linspace(0,1,n).'; seg = p.*(1-s) + q.*s;
            occ = getOccupancy(obj.omap3d, seg);
            ok = all(occ < 0.5);
        end
        
        function height_map = generate_fractal_terrain(obj, X, Y, complexity)
            %% Generate Fractal Terrain Using Perlin-like Noise
            
            height_map = zeros(size(X));
            
            % Multiple octaves of noise for realistic terrain
            octaves = 4;
            amplitude = 1.0;
            frequency = 0.1 * complexity;
            
            for octave = 1:octaves
                % Simple noise generation (can be replaced with Perlin noise)
                noise = sin(frequency * X) .* cos(frequency * Y) + ...
                       0.5 * sin(2 * frequency * X) .* sin(2 * frequency * Y);
                
                height_map = height_map + amplitude * noise;
                
                amplitude = amplitude * 0.5;
                frequency = frequency * 2;
            end
            
            % Scale to desired height range
            max_terrain_height = obj.params.rl.max_terrain_height;
            height_map = max_terrain_height * (height_map - min(height_map(:))) / ...
                        (max(height_map(:)) - min(height_map(:)));
        end
        
        function add_forest_obstacles(obj)
            %% Add Tree-like Vertical Obstacles
            
            density = obj.params.rl.obstacle_density;
            bounds = obj.map_bounds;
            res = obj.map_resolution;
            
            [x_cells, y_cells, z_cells] = size(obj.occupancy_grid);
            
            % Random tree positions
            num_trees = round(density * x_cells * y_cells / 100);
            
            for i = 1:num_trees
                % Random tree position
                tree_x = randi([1, x_cells]);
                tree_y = randi([1, y_cells]);
                
                % Tree height (varied)
                tree_height_cells = randi([5, z_cells-2]);
                
                % Tree radius (1-3 cells)
                tree_radius = randi([1, 3]);
                
                % Create cylindrical tree
                for dx = -tree_radius:tree_radius
                    for dy = -tree_radius:tree_radius
                        if sqrt(dx^2 + dy^2) <= tree_radius
                            x_idx = tree_x + dx;
                            y_idx = tree_y + dy;
                            
                            if x_idx >= 1 && x_idx <= x_cells && ...
                               y_idx >= 1 && y_idx <= y_cells
                                obj.occupancy_grid(x_idx, y_idx, 1:tree_height_cells) = 1;
                            end
                        end
                    end
                end
            end
        end
        
        function add_urban_obstacles(obj)
            %% Add Building-like Rectangular Obstacles
            
            density = obj.params.rl.obstacle_density;
            [x_cells, y_cells, z_cells] = size(obj.occupancy_grid);
            
            num_buildings = round(density * x_cells * y_cells / 200);
            
            for i = 1:num_buildings
                % Building dimensions
                width = randi([3, 8]);
                length = randi([3, 8]);
                height = randi([3, z_cells-2]);
                
                % Building position
                x_start = randi([1, max(1, x_cells - width)]);
                y_start = randi([1, max(1, y_cells - length)]);
                
                % Create building
                obj.occupancy_grid(x_start:x_start+width-1, ...
                                 y_start:y_start+length-1, ...
                                 1:height) = 1;
            end
        end
        
        function state = sample_collision_free_state(obj)
            %% Sample Random Collision-Free Starting State
            
            max_attempts = 1000;
            bounds = obj.map_bounds;
            
            for attempt = 1:max_attempts
                % Random position
                pos = [
                    bounds(1) + rand() * (bounds(2) - bounds(1));
                    bounds(3) + rand() * (bounds(4) - bounds(3));
                    bounds(5) + rand() * (bounds(6) - bounds(5))
                ];
                
                % Check collision
                if ~obj.is_position_occupied(pos)
                    % Random initial velocity (small)
                    vel = 0.5 * randn(3, 1);
                    
                    % Random initial attitude (small angles)
                    att = [0; 0; 2*pi*rand()]; % Only random yaw
                    
                    state = [pos; vel; att];
                    return;
                end
            end
            
            error('Could not find collision-free starting position');
        end
        
        function generate_target_trajectory(obj)
            %% Generate Target Waypoint Trajectory
            
            % Simple goal: navigate to opposite corner avoiding obstacles
            start_pos = obj.drone_state(1:3);
            bounds = obj.map_bounds;
            
            % Target is in opposite corner with some randomization
            target_pos = [
                bounds(2) - 0.1 * (bounds(2) - bounds(1));
                bounds(4) - 0.1 * (bounds(4) - bounds(3));
                bounds(5) + 0.7 * (bounds(6) - bounds(5))
            ] + 0.1 * randn(3, 1);
            
            % Ensure target is collision-free
            if obj.is_position_occupied(target_pos)
                % Move target slightly until collision-free
                for attempt = 1:100
                    offset = 2 * obj.map_resolution * randn(3, 1);
                    test_target = target_pos + offset;
                    if ~obj.is_position_occupied(test_target)
                        target_pos = test_target;
                        break;
                    end
                end
            end
            
            % Create intermediate waypoints for path guidance
            num_waypoints = 5;
            waypoints = zeros(3, num_waypoints);
            
            for i = 1:num_waypoints
                alpha = i / num_waypoints;
                waypoints(:, i) = (1 - alpha) * start_pos + alpha * target_pos;
                
                % Add some randomization to intermediate waypoints
                if i > 1 && i < num_waypoints
                    waypoints(:, i) = waypoints(:, i) + 0.5 * randn(3, 1);
                end
            end
            
            obj.target_waypoints = waypoints;
        end
        
        function obs = generate_observation(obj)
            %% Generate Observation Vector for RL Agent
            
            % 1) Local occupancy grid around drone
            local_grid = obj.extract_local_occupancy_grid();
            
            % 2) Current EKF state estimate (normalized)
            norm_state = obj.normalize_state(obj.ekf_state);
            
            % 3) Relative position to current target waypoint
            current_target = obj.target_waypoints(:, min(obj.current_waypoint_idx, size(obj.target_waypoints, 2)));
            relative_target = current_target - obj.ekf_state(1:3);
            relative_target = relative_target / 10.0; % Normalize by typical distance
            
            % 4) EKF uncertainty (diagonal covariance elements)
            uncertainty = diag(obj.ekf_covariance);
            norm_uncertainty = log10(uncertainty + 1e-6); % Log scale, normalized
            norm_uncertainty = norm_uncertainty(:); % Ensure column vector
            
            % 5) Recent trajectory history (relative positions)
            if size(obj.trajectory_history, 2) >= 10
                recent_traj = obj.trajectory_history(:, end-9:end);
            else
                % Pad with current position if not enough history
                needed = 10 - size(obj.trajectory_history, 2);
                padding = repmat(obj.trajectory_history(:, end), 1, needed);
                recent_traj = [padding, obj.trajectory_history];
            end
            
            % Convert to relative displacements
            relative_traj = diff(recent_traj, 1, 2);
            if size(relative_traj, 2) < 10
                relative_traj = [relative_traj, zeros(3, 10 - size(relative_traj, 2))];
            end
            relative_traj = relative_traj(:) / obj.map_resolution; % Normalize
            
            % Concatenate all observation components
            obs = [
                local_grid(:);
                norm_state;
                relative_target;
                norm_uncertainty;
                relative_traj
            ];
            
            % Ensure correct dimension
            if length(obs) ~= obj.observation_dim
                warning('Observation dimension mismatch: expected %d, got %d', ...
                       obj.observation_dim, length(obs));
                obs = [obs; zeros(obj.observation_dim - length(obs), 1)];
                obs = obs(1:obj.observation_dim);
            end
        end
        
        function reward = calculate_reward(obj, action)
            %% Calculate Enhanced Reward with Proximity Penalties (PDF-Aligned)
            
            reward = 0;
            
            % 1) Goal reaching reward
            if obj.goal_reached
                reward = reward + obj.params.rl.rewards.goal_reached;
                return;
            end
            
            % 2) Collision penalty
            if obj.collision_occurred
                reward = reward + obj.params.rl.rewards.collision;
                return;
            end
            
            % 3) Proximity-based penalty using distance transform
            proximity_penalty = obj.calculate_proximity_penalty();
            reward = reward + proximity_penalty;
            
            % 4) Near-collision penalty (close but not touching)
            min_distance = obj.calculate_minimum_obstacle_distance();
            if min_distance < 1.5 && min_distance > 0
                reward = reward + obj.params.rl.rewards.near_collision;
            end
            
            % 5) Progress toward goal
            current_distance = norm(obj.drone_state(1:3) - obj.target_waypoints(:, end));
            progress_reward = (obj.previous_distance_to_goal - current_distance) * obj.params.rl.rewards.progress_scale;
            reward = reward + progress_reward;
            obj.previous_distance_to_goal = current_distance;
            
            % 6) Energy efficiency (penalize large actions)
            energy_penalty = -obj.params.rl.rewards.energy_penalty * norm(action);
            reward = reward + energy_penalty;
            
            % 7) Smooth flight (penalize rapid changes)
            if obj.episode_step > 1
                smoothness_penalty = -obj.params.rl.rewards.smoothness_penalty * norm(diff(obj.trajectory_history(:, end-1:end), 1, 2));
                reward = reward + smoothness_penalty;
            end
            
            % 8) Waypoint following bonus
            if obj.current_waypoint_idx <= size(obj.target_waypoints, 2)
                waypoint_distance = norm(obj.drone_state(1:3) - obj.target_waypoints(:, obj.current_waypoint_idx));
                if waypoint_distance < 2.0 % Within 2m of waypoint
                    reward = reward + obj.params.rl.rewards.waypoint_bonus;
                    obj.current_waypoint_idx = obj.current_waypoint_idx + 1;
                end
            end
            
            % 9) Staying in bounds bonus
            if ~obj.check_out_of_bounds()
                reward = reward + 0.1;
            end
            
            % 10) Survival bonus (small reward for each step)
            reward = reward + obj.params.rl.rewards.survival_bonus;
            
            % 11) EKF confidence reward (reward good state estimation)
            if obj.params.rl.use_ekf_uncertainty
                ekf_confidence = 1.0 / (1.0 + trace(obj.ekf_covariance));
                reward = reward + obj.params.rl.rewards.ekf_confidence_bonus * ekf_confidence;
            end
            
            % 12) Path efficiency bonus
            if obj.episode_step > 10
                path_length = size(obj.trajectory_history, 2) * obj.map_resolution;
                direct_distance = norm(obj.trajectory_history(:, 1) - obj.drone_state(1:3));
                if direct_distance > 0.1
                    efficiency = direct_distance / path_length;
                    reward = reward + obj.params.rl.rewards.efficiency_bonus * efficiency;
                end
            end
        end
        
        function proximity_penalty = calculate_proximity_penalty(obj)
            %% Calculate Proximity-Based Penalty Using Distance Transform
            
            pos = obj.drone_state(1:3);
            
            % Convert position to grid coordinates
            x_idx = round((pos(1) - obj.map_bounds(1)) / obj.map_resolution) + 1;
            y_idx = round((pos(2) - obj.map_bounds(3)) / obj.map_resolution) + 1;
            z_idx = round((pos(3) - obj.map_bounds(5)) / obj.map_resolution) + 1;
            
            [x_size, y_size, z_size] = size(obj.occupancy_grid);
            
            % Check bounds
            if x_idx < 1 || x_idx > x_size || y_idx < 1 || y_idx > y_size || z_idx < 1 || z_idx > z_size
                proximity_penalty = -20;  % Out of bounds penalty
                return;
            end
            
            % Calculate distance to nearest obstacle using local search
            min_distance = obj.calculate_minimum_obstacle_distance();
            
            % Apply proximity penalty based on distance
            safe_distance = obj.params.rl.rewards.safe_distance;
            penalty_scale = obj.params.rl.rewards.proximity_penalty_scale;
            
            if min_distance < safe_distance
                % Inverse relationship: closer = more penalty
                proximity_penalty = -penalty_scale * (safe_distance - min_distance) / safe_distance;
            else
                proximity_penalty = 0;  % No penalty if far enough
            end
        end
        
        function min_distance = calculate_minimum_obstacle_distance(obj)
            %% Calculate Minimum Distance to Any Obstacle
            
            pos = obj.drone_state(1:3);
            min_distance = inf;
            
            % Search in local area around drone
            search_radius_cells = round(obj.params.rl.rewards.safe_distance / obj.map_resolution) + 2;
            
            % Convert position to grid coordinates
            x_center = round((pos(1) - obj.map_bounds(1)) / obj.map_resolution) + 1;
            y_center = round((pos(2) - obj.map_bounds(3)) / obj.map_resolution) + 1;
            z_center = round((pos(3) - obj.map_bounds(5)) / obj.map_resolution) + 1;
            
            [x_size, y_size, z_size] = size(obj.occupancy_grid);
            
            % Search around current position
            for dx = -search_radius_cells:search_radius_cells
                for dy = -search_radius_cells:search_radius_cells
                    for dz = -search_radius_cells:search_radius_cells
                        xi = x_center + dx;
                        yi = y_center + dy;
                        zi = z_center + dz;
                        
                        % Check bounds
                        if xi >= 1 && xi <= x_size && yi >= 1 && yi <= y_size && zi >= 1 && zi <= z_size
                            if obj.occupancy_grid(xi, yi, zi) == 1
                                % Calculate actual distance
                                obstacle_pos = [
                                    obj.map_bounds(1) + (xi - 1) * obj.map_resolution;
                                    obj.map_bounds(3) + (yi - 1) * obj.map_resolution;
                                    obj.map_bounds(5) + (zi - 1) * obj.map_resolution
                                ];
                                
                                distance = norm(pos - obstacle_pos);
                                min_distance = min(min_distance, distance);
                            end
                        end
                    end
                end
            end
            
            % If no obstacles found in search area, set to safe distance
            if min_distance == inf
                min_distance = obj.params.rl.rewards.safe_distance + 1;
            end
        end
        
        function apply_action(obj, action)
            %% Apply RL Action to Update Drone State
            
            % Use the RL-EKF integration to handle action application
            if isempty(obj.rl_ekf_integration)
                obj.rl_ekf_integration = rl_ekf_integration(obj.params);
            end
            
            % Apply action and get new state
            dt = obj.params.Ts.physics;
            [new_state, uncertainty_info] = obj.rl_ekf_integration.step(obj.drone_state, action, dt);
            
            % Update drone state with new position
            obj.drone_state = new_state;
            obj.ekf_state = new_state;
            obj.ekf_covariance = uncertainty_info.covariance;
        end
        
        function simulate_sensors_and_ekf(obj)
            %% Simulate Sensors and Update EKF
            
            % This is now handled by the RL-EKF integration
            % Just update the sensor history for logging
            obj.sensor_history = obj.rl_ekf_integration.get_rl_observation_data();
        end
        
        function collision = check_collision(obj)
            %% Check if Current Position Results in Collision
            
            pos = obj.drone_state(1:3);
            collision = obj.is_position_occupied(pos);
        end
        
        function occupied = is_position_occupied(obj, position)
            %% Check if Given Position is Occupied in Grid
            
            % Convert world coordinates to grid indices
            x_idx = round((position(1) - obj.map_bounds(1)) / obj.map_resolution) + 1;
            y_idx = round((position(2) - obj.map_bounds(3)) / obj.map_resolution) + 1;
            z_idx = round((position(3) - obj.map_bounds(5)) / obj.map_resolution) + 1;
            
            % Check bounds
            if x_idx < 1 || x_idx > size(obj.occupancy_grid, 1) || ...
               y_idx < 1 || y_idx > size(obj.occupancy_grid, 2) || ...
               z_idx < 1 || z_idx > size(obj.occupancy_grid, 3)
                occupied = true;  % Out of bounds
                return;
            end
            
            % Check occupancy with safety margin (2x2x2 cube around position)
            margin = 1;  % cells
            occupied = false;
            
            for dx = -margin:margin
                for dy = -margin:margin
                    for dz = -margin:margin
                        xi = x_idx + dx;
                        yi = y_idx + dy;
                        zi = z_idx + dz;
                        
                        if xi >= 1 && xi <= size(obj.occupancy_grid, 1) && ...
                           yi >= 1 && yi <= size(obj.occupancy_grid, 2) && ...
                           zi >= 1 && zi <= size(obj.occupancy_grid, 3)
                            if obj.occupancy_grid(xi, yi, zi) == 1
                                occupied = true;
                                return;
                            end
                        end
                    end
                end
            end
        end
        
        function goal_reached = check_goal_reached(obj)
            %% Check if Goal Has Been Reached
            
            current_pos = obj.drone_state(1:3);
            goal_pos = obj.target_waypoints(:, end);
            distance = norm(current_pos - goal_pos);
            
            goal_reached = distance < obj.params.rl.success_distance;
        end
        
        function out_of_bounds = check_out_of_bounds(obj)
            %% Check if Drone is Out of Map Bounds
            
            pos = obj.drone_state(1:3);
            bounds = obj.map_bounds;
            
            out_of_bounds = pos(1) < bounds(1) || pos(1) > bounds(2) || ...
                           pos(2) < bounds(3) || pos(2) > bounds(4) || ...
                           pos(3) < bounds(5) || pos(3) > bounds(6);
        end
        
        function local_grid = extract_local_occupancy_grid(obj)
            %% Extract Local Occupancy Grid Around Drone
            
            local_size = [16, 16, 8];  % Reduced local grid dimensions for manageable observation size
            pos = obj.drone_state(1:3);
            
            % Convert position to grid coordinates
            x_center = round((pos(1) - obj.map_bounds(1)) / obj.map_resolution) + 1;
            y_center = round((pos(2) - obj.map_bounds(3)) / obj.map_resolution) + 1;
            z_center = round((pos(3) - obj.map_bounds(5)) / obj.map_resolution) + 1;
            
            % Calculate extraction bounds
            x_start = max(1, x_center - local_size(1)/2);
            x_end = min(size(obj.occupancy_grid, 1), x_center + local_size(1)/2 - 1);
            y_start = max(1, y_center - local_size(2)/2);
            y_end = min(size(obj.occupancy_grid, 2), y_center + local_size(2)/2 - 1);
            z_start = max(1, z_center - local_size(3)/2);
            z_end = min(size(obj.occupancy_grid, 3), z_center + local_size(3)/2 - 1);
            
            % Extract local grid
            local_grid = zeros(local_size);
            
            % Copy available data
            x_range = x_start:x_end;
            y_range = y_start:y_end;
            z_range = z_start:z_end;
            
            local_x_start = max(1, local_size(1)/2 - (x_center - x_start) + 1);
            local_y_start = max(1, local_size(2)/2 - (y_center - y_start) + 1);
            local_z_start = max(1, local_size(3)/2 - (z_center - z_start) + 1);
            
            local_x_end = local_x_start + length(x_range) - 1;
            local_y_end = local_y_start + length(y_range) - 1;
            local_z_end = local_z_start + length(z_range) - 1;
            
            if ~isempty(x_range) && ~isempty(y_range) && ~isempty(z_range)
                local_grid(local_x_start:local_x_end, ...
                          local_y_start:local_y_end, ...
                          local_z_start:local_z_end) = ...
                    obj.occupancy_grid(x_range, y_range, z_range);
            end
        end
        
        function norm_state = normalize_state(obj, state)
            %% Normalize State Vector for Neural Network Input
            
            % Position normalization (by map size)
            map_size = [
                obj.map_bounds(2) - obj.map_bounds(1);
                obj.map_bounds(4) - obj.map_bounds(3);
                obj.map_bounds(6) - obj.map_bounds(5)
            ];
            norm_pos = state(1:3) ./ map_size;
            
            % Velocity normalization (by max velocity)
            norm_vel = state(4:6) / obj.params.rl.max_velocity;
            
            % Attitude normalization (angles already in [-pi, pi])
            norm_att = state(7:9) / pi;
            
            norm_state = [norm_pos; norm_vel; norm_att];
        end
        
        function add_building_complexity(obj, occupancy_grid, x_start, y_start, x_end, y_end, terrain_z, z_end)
            %% Add Architectural Complexity to Buildings
            
            % Add random extensions and overhangs
            if rand() < 0.5
                % Add extension
                extension_size = randi([2, 4]);
                direction = randi(4);  % 1=north, 2=east, 3=south, 4=west
                
                switch direction
                    case 1  % North
                        ext_y_start = max(1, y_start - extension_size);
                        ext_y_end = y_start - 1;
                        if ext_y_end >= ext_y_start
                            occupancy_grid(x_start:x_end, ext_y_start:ext_y_end, terrain_z+1:z_end) = 1;
                        end
                    case 2  % East
                        ext_x_start = x_end + 1;
                        ext_x_end = min(size(occupancy_grid, 1), x_end + extension_size);
                        if ext_x_start <= ext_x_end
                            occupancy_grid(ext_x_start:ext_x_end, y_start:y_end, terrain_z+1:z_end) = 1;
                        end
                    case 3  % South
                        ext_y_start = y_end + 1;
                        ext_y_end = min(size(occupancy_grid, 2), y_end + extension_size);
                        if ext_y_start <= ext_y_end
                            occupancy_grid(x_start:x_end, ext_y_start:ext_y_end, terrain_z+1:z_end) = 1;
                        end
                    case 4  % West
                        ext_x_start = max(1, x_start - extension_size);
                        ext_x_end = x_start - 1;
                        if ext_x_end >= ext_x_start
                            occupancy_grid(ext_x_start:ext_x_end, y_start:y_end, terrain_z+1:z_end) = 1;
                        end
                end
            end
        end
        
        function add_canyon_obstacles(obj)
            %% Add Canyon-like Obstacles (placeholder implementation)
            
            % This would create narrow passages and vertical walls
            % Implementation depends on specific canyon geometry desired
            fprintf('Canyon obstacles generation not yet implemented\n');
        end
        
        function add_mixed_obstacles(obj)
            %% Add Mixed Obstacle Types
            
            % Combine forest and urban obstacles
            obj.add_forest_obstacles();
            obj.add_urban_obstacles();
        end
        
        function add_random_obstacles(obj)
            %% Add Random Scattered Obstacles
            
            density = obj.params.rl.obstacle_density;
            [x_cells, y_cells, z_cells] = size(obj.occupancy_grid);
            
            num_obstacles = round(density * x_cells * y_cells / 500);
            
            for obs = 1:num_obstacles
                x = randi(x_cells);
                y = randi(y_cells);
                
                obstacle_size = 1 + randi(2);  % 1-3 cells
                obstacle_height = 3 + randi(round(z_cells/4));
                
                % Find terrain height at this location
                terrain_z = find(obj.occupancy_grid(x, y, :), 1, 'last');
                if isempty(terrain_z), terrain_z = 1; end
                
                % Create obstacle
                for dx = 0:obstacle_size-1
                    for dy = 0:obstacle_size-1
                        xi = x + dx;
                        yi = y + dy;
                        if xi <= x_cells && yi <= y_cells
                            z_end = min(terrain_z + obstacle_height, z_cells);
                            obj.occupancy_grid(xi, yi, terrain_z+1:z_end) = 1;
                        end
                    end
                end
            end
        end
        
        function add_boundary_walls(obj)
            %% Add Rectangular Boundary Walls
            
            [x_cells, y_cells, z_cells] = size(obj.occupancy_grid);
            wall_height = round(z_cells * 0.7);  % 70% of max height
            wall_thickness = 2;  % cells
            
            % Create rectangular boundary walls
            for t = 1:wall_thickness
                % X boundaries (North and South walls)
                if t <= x_cells && (x_cells - t + 1) >= 1
                    obj.occupancy_grid(t, :, 1:wall_height) = 1;
                    obj.occupancy_grid(x_cells - t + 1, :, 1:wall_height) = 1;
                end
                
                % Y boundaries (East and West walls)
                if t <= y_cells && (y_cells - t + 1) >= 1
                    obj.occupancy_grid(:, t, 1:wall_height) = 1;
                    obj.occupancy_grid(:, y_cells - t + 1, 1:wall_height) = 1;
                end
            end
            
            % Add navigation openings (gates) in walls for training variety
            if rand() < 0.3  % 30% chance of having openings
                % Random opening in one wall
                wall_side = randi(4);  % 1=North, 2=South, 3=East, 4=West
                opening_size = 8;  % cells (4m opening)
                
                switch wall_side
                    case 1  % North wall opening
                        opening_center = randi([opening_size, y_cells - opening_size]);
                        opening_range = max(1, opening_center - opening_size/2):min(y_cells, opening_center + opening_size/2);
                        obj.occupancy_grid(1:wall_thickness, opening_range, :) = 0;
                        
                    case 2  % South wall opening
                        opening_center = randi([opening_size, y_cells - opening_size]);
                        opening_range = max(1, opening_center - opening_size/2):min(y_cells, opening_center + opening_size/2);
                        obj.occupancy_grid(x_cells-wall_thickness+1:x_cells, opening_range, :) = 0;
                        
                    case 3  % East wall opening
                        opening_center = randi([opening_size, x_cells - opening_size]);
                        opening_range = max(1, opening_center - opening_size/2):min(x_cells, opening_center + opening_size/2);
                        obj.occupancy_grid(opening_range, 1:wall_thickness, :) = 0;
                        
                    case 4  % West wall opening
                        opening_center = randi([opening_size, x_cells - opening_size]);
                        opening_range = max(1, opening_center - opening_size/2):min(x_cells, opening_center + opening_size/2);
                        obj.occupancy_grid(opening_range, y_cells-wall_thickness+1:y_cells, :) = 0;
                end
            end
        end
        
        function initialize_ekf(obj)
            %% Initialize EKF State and Covariance
            
            % Initialize EKF integration if not already done
            if isempty(obj.rl_ekf_integration)
                obj.rl_ekf_integration = rl_ekf_integration(obj.params);
            end
            
            % Reset EKF to current drone state
            obj.rl_ekf_integration.reset(obj.drone_state);
            obj.ekf_state = obj.drone_state;
            obj.ekf_covariance = obj.rl_ekf_integration.ekf_covariance;
        end
        
        function create_emergency_clearance(obj, occupancy_grid)
            %% Create Emergency Clearance in Overcrowded Map
            
            % Clear a cross-shaped area in the center
            [x_cells, y_cells, z_cells] = size(occupancy_grid);
            center_x = round(x_cells/2);
            center_y = round(y_cells/2);
            
            clearance_size = 5;  % cells
            
            % Vertical clearance
            x_range = max(1, center_x - clearance_size):min(x_cells, center_x + clearance_size);
            y_range = max(1, center_y - 1):min(y_cells, center_y + 1);
            z_range = 1:round(z_cells/2);
            
            occupancy_grid(x_range, y_range, z_range) = 0;
            
            % Horizontal clearance
            x_range = max(1, center_x - 1):min(x_cells, center_x + 1);
            y_range = max(1, center_y - clearance_size):min(y_cells, center_y + clearance_size);
            
            occupancy_grid(x_range, y_range, z_range) = 0;
        end
        
    end
end
