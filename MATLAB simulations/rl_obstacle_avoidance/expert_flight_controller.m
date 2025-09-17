classdef expert_flight_controller < handle
    %% expert_flight_controller - Expert Flight Controller with RRT* Path Planning
    % PURPOSE
    % Expert flight controller that uses RRT* path planning + Pure Pursuit
    % for optimal obstacle avoidance through cylindrical obstacles.
    
    properties
        % Path Planning
        rrt_planner
        pure_pursuit_controller
        
        % Obstacle Map
        obstacle_map
        start_pos
        goal_pos
        
        % Path Planning State
        current_path
        path_index
        path_planned
        
        % Control Parameters
        lookahead_distance
        max_velocity
        max_acceleration
        
        % Navigation Toolbox
        path_planner
        waypoint_follower
    end
    
    methods
        function obj = expert_flight_controller(params)
            %% Initialize Expert Flight Controller
            
            % Initialize path planning parameters
            obj.lookahead_distance = 3.0;  % meters
            obj.max_velocity = 2.0;        % m/s
            obj.max_acceleration = 1.5;    % m/s²
            
            % Initialize path planning state
            obj.current_path = [];
            obj.path_index = 1;
            obj.path_planned = false;
            
            % Initialize Navigation Toolbox components
            obj.setup_navigation_toolbox();
            
            fprintf('✓ Expert flight controller initialized\n');
            fprintf('  - RRT* path planning\n');
            fprintf('  - Pure Pursuit following\n');
            fprintf('  - Navigation Toolbox integration\n');
        end
        
        function setup_navigation_toolbox(obj)
            %% Setup Navigation Toolbox Components (Simplified)
            
            % Initialize path planner as empty (will use custom RRT*)
            obj.path_planner = [];
            
            % Create Pure Pursuit controller if available
            try
                obj.pure_pursuit_controller = controllerPurePursuit;
                obj.pure_pursuit_controller.LookaheadDistance = obj.lookahead_distance;
                obj.pure_pursuit_controller.DesiredLinearVelocity = obj.max_velocity;
            catch
                obj.pure_pursuit_controller = [];
            end
            
            % Create waypoint follower if available
            try
                obj.waypoint_follower = uavWaypointFollower('UAVType', 'multirotor', ...
                                                           'TransitionRadius', 3.0, ...
                                                           'MinLookaheadDistance', 2.0);
            catch
                obj.waypoint_follower = [];
            end
        end
        
        function set_obstacle_map(obj, obstacle_map)
            %% Set Obstacle Map for Path Planning
            
            obj.obstacle_map = obstacle_map;
            
            % Create state space for RRT*
            obj.create_state_space();
            
            % Reset path planning
            obj.path_planned = false;
            obj.path_index = 1;
        end
        
        function set_start_goal(obj, start_pos, goal_pos)
            %% Set Start and Goal Positions
            
            obj.start_pos = start_pos;
            obj.goal_pos = goal_pos;
            
            % Reset path planning
            obj.path_planned = false;
            obj.path_index = 1;
        end
        
        function create_state_space(obj)
            %% Create State Space for RRT* Planning (Simplified)
            
            % Skip state space creation for now - use custom RRT*
            % The custom RRT* doesn't need the Navigation Toolbox state space
        end
        
        function validator = create_collision_checker(obj)
            %% Create Collision Checker for Cylindrical Obstacles
            
            % Create state validator
            validator = validatorOccupancyMap3D;
            
            % Create occupancy map
            map_resolution = 0.5;  % meters per cell
            map_size = [160, 160, 50];  % [x, y, z] cells
            
            % Initialize occupancy map
            occupancy_map = occupancyMap3D(map_size, map_resolution);
            
            % Add cylindrical obstacles to occupancy map
            for i = 1:length(obj.obstacle_map.cylinders)
                cyl = obj.obstacle_map.cylinders{i};
                
                % Create cylindrical obstacle
                [X, Y, Z] = meshgrid(...
                    (cyl.center(1) - cyl.radius):map_resolution:(cyl.center(1) + cyl.radius), ...
                    (cyl.center(2) - cyl.radius):map_resolution:(cyl.center(2) + cyl.radius), ...
                    cyl.center(3):map_resolution:(cyl.center(3) + cyl.height));
                
                % Check which points are inside cylinder
                dist_xy = sqrt((X - cyl.center(1)).^2 + (Y - cyl.center(2)).^2);
                inside_cylinder = (dist_xy <= cyl.radius) & (Z >= cyl.center(3)) & (Z <= cyl.center(3) + cyl.height);
                
                % Set occupied cells
                for j = 1:numel(X)
                    if inside_cylinder(j)
                        pos = [X(j), Y(j), Z(j)];
                        setOccupancy(occupancy_map, pos, 1);
                    end
                end
            end
            
            % Set validator occupancy map
            validator.Map = occupancy_map;
        end
        
        function [expert_state, control_input, path_info] = compute_expert_control(obj, current_state, current_time)
            %% Compute Expert Control Using RRT* + Pure Pursuit
            
            % Extract current position
            current_pos = current_state(1:3);
            
            % Plan path if not already planned
            if ~obj.path_planned
                obj.plan_path();
            end
            
            % If no path available, use simple goal-seeking behavior
            if isempty(obj.current_path)
                [expert_state, control_input, path_info] = obj.simple_goal_seeking(current_state);
                return;
            end
            
            % Follow planned path using Pure Pursuit
            [expert_state, control_input, path_info] = obj.follow_path(current_state, current_time);
        end
        
        function plan_path(obj)
            %% Plan Path Using Custom RRT*
            
            try
                % Use custom RRT* implementation
                path = obj.custom_rrt_star(obj.start_pos, obj.goal_pos);
                
                if ~isempty(path)
                    obj.current_path = path;
                    obj.path_planned = true;
                    obj.path_index = 1;
                    fprintf('    Path planned successfully: %d waypoints\n', size(path, 1));
                else
                    fprintf('    Path planning failed, using simple goal seeking\n');
                    obj.current_path = [];
                    obj.path_planned = false;
                end
            catch ME
                fprintf('    Path planning error: %s\n', ME.message);
                obj.current_path = [];
                obj.path_planned = false;
            end
        end
        
        function path = custom_rrt_star(obj, start, goal)
            %% Custom RRT* Implementation
            
            % RRT* parameters
            max_iterations = 1000;
            step_size = 2.0;
            goal_tolerance = 3.0;
            
            % Initialize tree
            tree = struct();
            tree.nodes = start;
            tree.parents = 0;
            tree.costs = 0;
            
            % Main RRT* loop
            for i = 1:max_iterations
                % Sample random point
                if rand() < 0.1  % 10% chance to sample goal
                    sample = goal;
                else
                    sample = obj.sample_random_point();
                end
                
                % Find nearest node
                [nearest_idx, nearest_node] = obj.find_nearest_node(tree, sample);
                
                % Steer towards sample
                new_node = obj.steer(nearest_node, sample, step_size);
                
                % Check collision
                if obj.check_collision_free(nearest_node, new_node)
                    % Find nearby nodes for rewiring
                    nearby_indices = obj.find_nearby_nodes(tree, new_node, step_size);
                    
                    % Find best parent
                    best_parent = nearest_idx;
                    best_cost = tree.costs(nearest_idx) + norm(new_node - nearest_node);
                    
                    for j = 1:length(nearby_indices)
                        idx = nearby_indices(j);
                        if idx ~= nearest_idx
                            cost = tree.costs(idx) + norm(new_node - tree.nodes(:, idx));
                            if cost < best_cost && obj.check_collision_free(tree.nodes(:, idx), new_node)
                                best_parent = idx;
                                best_cost = cost;
                            end
                        end
                    end
                    
                    % Add node to tree
                    tree.nodes = [tree.nodes, new_node];
                    tree.parents = [tree.parents, best_parent];
                    tree.costs = [tree.costs, best_cost];
                    
                    % Rewire nearby nodes
                    for j = 1:length(nearby_indices)
                        idx = nearby_indices(j);
                        if idx ~= best_parent
                            new_cost = best_cost + norm(new_node - tree.nodes(:, idx));
                            if new_cost < tree.costs(idx) && obj.check_collision_free(new_node, tree.nodes(:, idx))
                                tree.parents(idx) = size(tree.nodes, 2);
                                tree.costs(idx) = new_cost;
                            end
                        end
                    end
                    
                    % Check if goal reached
                    if norm(new_node - goal) < goal_tolerance
                        path = obj.extract_path(tree, size(tree.nodes, 2), goal);
                        return;
                    end
                end
            end
            
            % If no path found, return empty
            path = [];
        end
        
        function sample = sample_random_point(obj)
            %% Sample Random Point in State Space
            
            x = obj.obstacle_map.bounds(1) + (obj.obstacle_map.bounds(2) - obj.obstacle_map.bounds(1)) * rand();
            y = obj.obstacle_map.bounds(3) + (obj.obstacle_map.bounds(4) - obj.obstacle_map.bounds(3)) * rand();
            z = obj.obstacle_map.bounds(5) + (obj.obstacle_map.bounds(6) - obj.obstacle_map.bounds(5)) * rand();
            
            sample = [x; y; z];
        end
        
        function [idx, node] = find_nearest_node(obj, tree, sample)
            %% Find Nearest Node in Tree
            
            distances = vecnorm(tree.nodes - sample, 2, 1);
            [~, idx] = min(distances);
            node = tree.nodes(:, idx);
        end
        
        function new_node = steer(obj, from, to, step_size)
            %% Steer from 'from' towards 'to' with step_size
            
            direction = to - from;
            distance = norm(direction);
            
            if distance <= step_size
                new_node = to;
            else
                new_node = from + (direction / distance) * step_size;
            end
        end
        
        function nearby_indices = find_nearby_nodes(obj, tree, node, radius)
            %% Find Nearby Nodes within radius
            
            distances = vecnorm(tree.nodes - node, 2, 1);
            nearby_indices = find(distances <= radius);
        end
        
        function collision_free = check_collision_free(obj, from, to)
            %% Check if path from 'from' to 'to' is collision-free
            
            collision_free = true;
            
            % Check multiple points along the path
            num_checks = 10;
            for i = 0:num_checks
                t = i / num_checks;
                point = from + t * (to - from);
                
                if obj.check_collision_with_obstacles(point)
                    collision_free = false;
                    return;
                end
            end
        end
        
        function collision = check_collision_with_obstacles(obj, point)
            %% Check collision with obstacles
            
            collision = false;
            
            % Check cylindrical obstacles
            for i = 1:length(obj.obstacle_map.cylinders)
                cyl = obj.obstacle_map.cylinders{i};
                dist_xy = norm(point(1:2) - cyl.center(1:2));
                
                if dist_xy < cyl.radius && point(3) >= cyl.center(3) && point(3) <= cyl.center(3) + cyl.height
                    collision = true;
                    return;
                end
            end
        end
        
        function path = extract_path(obj, tree, goal_idx, goal)
            %% Extract path from tree
            
            path = [];
            current_idx = goal_idx;
            
            while current_idx > 0
                path = [tree.nodes(:, current_idx), path];
                current_idx = tree.parents(current_idx);
            end
            
            % Add goal if not already there
            if norm(path(:, end) - goal) > 0.1
                path = [path, goal];
            end
        end
        
        function [expert_state, control_input, path_info] = follow_path(obj, current_state, current_time)
            %% Follow Planned Path Using Pure Pursuit
            
            current_pos = current_state(1:3);
            current_vel = current_state(4:6);
            current_att = current_state(7:9);
            
            % Find closest point on path
            if obj.path_index < size(obj.current_path, 2)
                % Move to next waypoint if close enough
                next_waypoint = obj.current_path(:, obj.path_index);
                distance_to_waypoint = norm(current_pos - next_waypoint);
                
                if distance_to_waypoint < 2.0
                    obj.path_index = obj.path_index + 1;
                end
            end
            
            % Get current target waypoint
            if obj.path_index <= size(obj.current_path, 2)
                target_pos = obj.current_path(:, obj.path_index);
            else
                target_pos = obj.goal_pos;
            end
            
            % Compute desired velocity using Pure Pursuit
            desired_vel = obj.compute_pure_pursuit_velocity(current_pos, target_pos, current_vel);
            
            % Compute desired acceleration
            desired_accel = obj.compute_desired_acceleration(current_vel, desired_vel);
            
            % Compute attitude commands
            [desired_att, thrust_cmd] = obj.compute_attitude_commands(desired_accel, current_att);
            
            % Compute torque commands
            torque_cmd = obj.compute_torque_commands(current_att, desired_att, current_vel);
            
            % Generate expert state
            expert_state = [target_pos; desired_vel; desired_att];
            
            % Generate control input
            control_input = [thrust_cmd; torque_cmd];
            
            % Generate path info
            path_info = struct();
            path_info.target_pos = target_pos;
            path_info.desired_vel = desired_vel;
            path_info.desired_accel = desired_accel;
            path_info.path_index = obj.path_index;
            path_info.path_length = size(obj.current_path, 1);
        end
        
        function desired_vel = compute_pure_pursuit_velocity(obj, current_pos, target_pos, current_vel)
            %% Compute Pure Pursuit Velocity (Improved)
            
            % Compute direction to target
            direction = target_pos - current_pos;
            distance = norm(direction);
            
            if distance > 1e-6
                direction = direction / distance;
            else
                direction = [0; 0; 0];
            end
            
            % Compute desired velocity magnitude with better control
            if distance > 5.0
                % Far from target - use maximum velocity
                vel_magnitude = obj.max_velocity;
            elseif distance > 2.0
                % Medium distance - proportional velocity
                vel_magnitude = obj.max_velocity * (distance / 5.0);
            else
                % Close to target - slow down
                vel_magnitude = obj.max_velocity * 0.3;
            end
            
            % Compute desired velocity
            desired_vel = direction * vel_magnitude;
            
            % Add velocity damping for smoothness
            damping_factor = 0.8;
            desired_vel = damping_factor * desired_vel + (1 - damping_factor) * current_vel;
            
            % Limit velocity
            desired_vel = obj.limit_velocity(desired_vel);
        end
        
        function desired_accel = compute_desired_acceleration(obj, current_vel, desired_vel)
            %% Compute Desired Acceleration (Improved)
            
            % Proportional-derivative control
            vel_error = desired_vel - current_vel;
            
            % Proportional gain
            kp = 3.0;
            desired_accel = vel_error * kp;
            
            % Add feedforward term for better tracking
            feedforward_gain = 0.5;
            desired_accel = desired_accel + desired_vel * feedforward_gain;
            
            % Limit acceleration
            desired_accel = obj.limit_acceleration(desired_accel);
        end
        
        function [desired_att, thrust_cmd] = compute_attitude_commands(obj, desired_accel, current_att)
            %% Compute Attitude Commands from Acceleration (Improved)
            
            % Compute desired thrust direction
            if norm(desired_accel) > 1e-6
                z_body_des = desired_accel / norm(desired_accel);
            else
                z_body_des = [0; 0; 1];
            end
            
            % Compute desired roll and pitch with better control
            max_tilt = deg2rad(20);  % 20 degrees max tilt
            
            % Roll control (banking for lateral movement)
            roll_des = asin(max(min(-z_body_des(2), sin(max_tilt)), -sin(max_tilt)));
            
            % Pitch control (forward/backward movement)
            if abs(cos(roll_des)) > 1e-6
                pitch_des = asin(max(min(z_body_des(1) / cos(roll_des), sin(max_tilt)), -sin(max_tilt)));
            else
                pitch_des = 0;
            end
            
            % Yaw control - maintain current yaw for now
            yaw_des = current_att(3);
            
            desired_att = [roll_des; pitch_des; yaw_des];
            
            % Compute thrust magnitude with better scaling
            thrust_magnitude = norm(desired_accel) * 0.8;  % Increased mass scaling
            thrust_magnitude = max(min(thrust_magnitude, 20.0), 8.0);  % 8-20 N range
            thrust_cmd = thrust_magnitude;
        end
        
        function torque_cmd = compute_torque_commands(obj, current_att, desired_att, current_vel)
            %% Compute Torque Commands for Attitude Control (Improved)
            
            % Compute attitude error
            att_error = desired_att - current_att;
            att_error(3) = atan2(sin(att_error(3)), cos(att_error(3)));  % Wrap yaw error
            
            % Proportional control
            kp = [4.0, 4.0, 2.0];  % [roll, pitch, yaw] gains
            
            % Compute torque commands
            torque_cmd = kp' .* att_error;
            
            % Limit torque
            max_torque = 0.2;  % Increased max torque
            torque_cmd = max(min(torque_cmd, max_torque), -max_torque);
        end
        
        function [expert_state, control_input, path_info] = simple_goal_seeking(obj, current_state)
            %% Simple Goal Seeking Behavior (Fallback)
            
            current_pos = current_state(1:3);
            current_vel = current_state(4:6);
            current_att = current_state(7:9);
            
            % Compute direction to goal
            direction = obj.goal_pos - current_pos;
            distance = norm(direction);
            
            if distance > 1e-6
                direction = direction / distance;
            else
                direction = [0; 0; 0];
            end
            
            % Compute desired velocity
            desired_vel = direction * min(obj.max_velocity, distance * 0.3);
            desired_vel = obj.limit_velocity(desired_vel);
            
            % Compute desired acceleration
            desired_accel = (desired_vel - current_vel) * 1.0;
            desired_accel = obj.limit_acceleration(desired_accel);
            
            % Compute attitude commands
            [desired_att, thrust_cmd] = obj.compute_attitude_commands(desired_accel, current_att);
            
            % Compute torque commands
            torque_cmd = obj.compute_torque_commands(current_att, desired_att, current_vel);
            
            % Generate expert state
            expert_state = [obj.goal_pos; desired_vel; desired_att];
            
            % Generate control input
            control_input = [thrust_cmd; torque_cmd];
            
            % Generate path info
            path_info = struct();
            path_info.target_pos = obj.goal_pos;
            path_info.desired_vel = desired_vel;
            path_info.desired_accel = desired_accel;
            path_info.path_index = 1;
            path_info.path_length = 1;
        end
        
        function vel = limit_velocity(obj, vel)
            %% Limit Velocity Commands
            vel = max(min(vel, obj.max_velocity), -obj.max_velocity);
        end
        
        function accel = limit_acceleration(obj, accel)
            %% Limit Acceleration Commands
            accel = max(min(accel, obj.max_acceleration), -obj.max_acceleration);
        end
        
        function reset(obj)
            %% Reset Expert Controller
            obj.current_path = [];
            obj.path_index = 1;
            obj.path_planned = false;
        end
    end
end
