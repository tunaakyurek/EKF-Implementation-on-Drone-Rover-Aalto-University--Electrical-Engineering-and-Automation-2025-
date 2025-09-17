%% limited_sensor_observer.m - Limited Sensor Observation Generator
% PURPOSE
% Generates limited-sensor observations for training "blind" drones that lack
% full sensor suites. Converts ground-truth maps to simulated sensor readings
% like lidar ranges, depth features, or compressed visual features.
%
% This implements the PDF requirement for "limited sensing capabilities"
% where the drone learns from minimal sensor data rather than full maps.

classdef limited_sensor_observer < handle
    properties
        params
        sensor_mode         % 'ranges', 'depth', 'compressed'
        
        % Range sensor parameters
        num_beams
        max_range
        fov_radians
        beam_angles
        
        % Depth sensor parameters
        depth_resolution
        depth_fov
        
        % Sensor noise and failure simulation
        noise_params
        dropout_params
    end
    
    methods
        function obj = limited_sensor_observer(params)
            obj.params = params;
            obj.sensor_mode = params.rl.sensor_mode;
            
            % Range sensor setup
            obj.num_beams = params.rl.limited_sensors.num_range_beams;
            obj.max_range = params.rl.limited_sensors.max_range;
            obj.fov_radians = deg2rad(params.rl.limited_sensors.fov_degrees);
            
            % Calculate beam angles (evenly distributed)
            if obj.num_beams > 1
                angle_step = obj.fov_radians / (obj.num_beams - 1);
                obj.beam_angles = linspace(-obj.fov_radians/2, obj.fov_radians/2, obj.num_beams);
            else
                obj.beam_angles = 0;
            end
            
            % Depth sensor setup
            obj.depth_resolution = params.rl.limited_sensors.depth_resolution;
            obj.depth_fov = deg2rad(90); % 90 degree depth camera FOV
            
            % Sensor noise parameters
            obj.noise_params = struct();
            obj.noise_params.range_noise_std = 0.05;     % 5cm range noise
            obj.noise_params.range_bias_max = 0.02;      % 2cm bias
            obj.noise_params.quantization_bits = 12;     % 12-bit quantization
            
            % Dropout parameters
            obj.dropout_params = struct();
            obj.dropout_params.beam_dropout_prob = 0.05; % 5% beam dropout
            obj.dropout_params.latency_steps = 2;        % 2-step sensor latency
            
            fprintf('Limited sensor observer initialized: %s mode\n', obj.sensor_mode);
            fprintf('  Range beams: %d, FOV: %.0f°, Max range: %.1fm\n', ...
                    obj.num_beams, rad2deg(obj.fov_radians), obj.max_range);
        end
        
        function observation = generate_limited_observation(obj, occupancy_grid, drone_state, map_bounds, map_resolution)
            %% Generate Limited Sensor Observation
            
            switch obj.sensor_mode
                case 'ranges'
                    observation = obj.generate_range_observation(occupancy_grid, drone_state, map_bounds, map_resolution);
                case 'depth'
                    observation = obj.generate_depth_observation(occupancy_grid, drone_state, map_bounds, map_resolution);
                case 'compressed'
                    observation = obj.generate_compressed_observation(occupancy_grid, drone_state, map_bounds, map_resolution);
                case 'limited'
                    % Combination of ranges + minimal state
                    ranges = obj.generate_range_observation(occupancy_grid, drone_state, map_bounds, map_resolution);
                    state_features = obj.extract_minimal_state_features(drone_state);
                    observation = [ranges; state_features];
                otherwise
                    error('Unknown sensor mode: %s', obj.sensor_mode);
            end
        end
        
        function ranges = generate_range_observation(obj, occupancy_grid, drone_state, map_bounds, map_resolution)
            %% Generate Lidar-like Range Measurements
            
            % Drone position and orientation
            pos = drone_state(1:3);
            yaw = drone_state(9);
            
            % Initialize ranges
            ranges = obj.max_range * ones(obj.num_beams, 1);
            
            % Cast rays for each beam
            for beam_idx = 1:obj.num_beams
                % Beam direction in world frame
                beam_angle = yaw + obj.beam_angles(beam_idx);
                beam_dir = [cos(beam_angle); sin(beam_angle); 0];
                
                % Ray casting
                range = obj.cast_ray(occupancy_grid, pos, beam_dir, map_bounds, map_resolution);
                
                % Apply sensor limitations
                range = min(range, obj.max_range);
                
                % Add noise and quantization
                if obj.params.rl.advanced.domain_randomization
                    range = obj.add_sensor_noise(range, beam_idx);
                end
                
                ranges(beam_idx) = range;
            end
            
            % Normalize ranges to [0, 1]
            ranges = ranges / obj.max_range;
        end
        
        function depth_features = generate_depth_observation(obj, occupancy_grid, drone_state, map_bounds, map_resolution)
            %% Generate Depth Camera-like Features
            
            % Extract depth image
            depth_image = obj.extract_depth_image(occupancy_grid, drone_state, map_bounds, map_resolution);
            
            % Compress to feature vector
            depth_features = obj.compress_depth_to_features(depth_image);
        end
        
        function compressed_obs = generate_compressed_observation(obj, occupancy_grid, drone_state, map_bounds, map_resolution)
            %% Generate Compressed Visual-like Features
            
            % Extract local patch
            local_patch = obj.extract_local_patch(occupancy_grid, drone_state, map_bounds, map_resolution);
            
            % Apply visual processing (edge detection, corners, etc.)
            compressed_obs = obj.compress_visual_features(local_patch);
        end
        
        function range = cast_ray(obj, occupancy_grid, start_pos, direction, map_bounds, map_resolution)
            %% Cast Ray Through Occupancy Grid
            
            % Ray parameters
            step_size = map_resolution / 2;  % Half-resolution steps
            max_steps = ceil(obj.max_range / step_size);
            
            % Current position
            current_pos = start_pos;
            
            for step = 1:max_steps
                % Move along ray
                current_pos = current_pos + step_size * direction;
                
                % Check if hit obstacle
                if obj.is_position_occupied(occupancy_grid, current_pos, map_bounds, map_resolution)
                    range = norm(current_pos - start_pos);
                    return;
                end
                
                % Check bounds
                if obj.is_out_of_bounds(current_pos, map_bounds)
                    range = norm(current_pos - start_pos);
                    return;
                end
            end
            
            % No hit within max range
            range = obj.max_range;
        end
        
        function occupied = is_position_occupied(obj, occupancy_grid, position, map_bounds, map_resolution)
            %% Check if Position is Occupied
            
            % Convert to grid coordinates
            x_idx = round((position(1) - map_bounds(1)) / map_resolution) + 1;
            y_idx = round((position(2) - map_bounds(3)) / map_resolution) + 1;
            z_idx = round((position(3) - map_bounds(5)) / map_resolution) + 1;
            
            % Check bounds
            [x_size, y_size, z_size] = size(occupancy_grid);
            if x_idx < 1 || x_idx > x_size || ...
               y_idx < 1 || y_idx > y_size || ...
               z_idx < 1 || z_idx > z_size
                occupied = true;
                return;
            end
            
            % Check occupancy
            occupied = occupancy_grid(x_idx, y_idx, z_idx) == 1;
        end
        
        function out_of_bounds = is_out_of_bounds(obj, position, map_bounds)
            %% Check if Position is Out of Bounds
            
            out_of_bounds = position(1) < map_bounds(1) || position(1) > map_bounds(2) || ...
                           position(2) < map_bounds(3) || position(2) > map_bounds(4) || ...
                           position(3) < map_bounds(5) || position(3) > map_bounds(6);
        end
        
        function noisy_range = add_sensor_noise(obj, true_range, beam_idx)
            %% Add Realistic Sensor Noise and Failures
            
            % Beam dropout
            if rand() < obj.dropout_params.beam_dropout_prob
                noisy_range = obj.max_range;  % Return max range for dropped beam
                return;
            end
            
            % Gaussian noise
            noise = obj.noise_params.range_noise_std * randn();
            
            % Random bias
            bias = obj.noise_params.range_bias_max * (2*rand() - 1);
            
            % Apply noise and bias
            noisy_range = true_range + noise + bias;
            
            % Quantization
            if obj.noise_params.quantization_bits < 16
                max_value = 2^obj.noise_params.quantization_bits - 1;
                quantized = round((noisy_range / obj.max_range) * max_value);
                noisy_range = (quantized / max_value) * obj.max_range;
            end
            
            % Clamp to valid range
            noisy_range = max(0, min(noisy_range, obj.max_range));
        end
        
        function depth_image = extract_depth_image(obj, occupancy_grid, drone_state, map_bounds, map_resolution)
            %% Extract Depth Image from Occupancy Grid
            
            % Simplified depth extraction (front-facing camera)
            pos = drone_state(1:3);
            yaw = drone_state(9);
            
            % Camera parameters
            [height, width] = obj.depth_resolution;
            
            % Initialize depth image
            depth_image = obj.max_range * ones(height, width);
            
            % For each pixel, cast a ray
            for row = 1:height
                for col = 1:width
                    % Pixel direction
                    u = (col - width/2) / (width/2);   % [-1, 1]
                    v = (row - height/2) / (height/2); % [-1, 1]
                    
                    % Ray direction (front-facing camera)
                    camera_dir = [1; u * tan(obj.depth_fov/2); -v * tan(obj.depth_fov/2)];
                    camera_dir = camera_dir / norm(camera_dir);
                    
                    % Rotate to world frame
                    R_yaw = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
                    world_dir = R_yaw * camera_dir;
                    
                    % Cast ray
                    range = obj.cast_ray(occupancy_grid, pos, world_dir, map_bounds, map_resolution);
                    depth_image(row, col) = range;
                end
            end
        end
        
        function features = compress_depth_to_features(obj, depth_image)
            %% Compress Depth Image to Feature Vector
            
            % Simple compression strategies
            
            % 1) Downsampled patches
            patch_size = 4;
            [height, width] = size(depth_image);
            patches_h = floor(height / patch_size);
            patches_w = floor(width / patch_size);
            
            patch_features = zeros(patches_h * patches_w, 1);
            idx = 1;
            
            for i = 1:patches_h
                for j = 1:patches_w
                    row_start = (i-1)*patch_size + 1;
                    row_end = i*patch_size;
                    col_start = (j-1)*patch_size + 1;
                    col_end = j*patch_size;
                    
                    patch = depth_image(row_start:row_end, col_start:col_end);
                    patch_features(idx) = min(patch(:));  % Closest obstacle in patch
                    idx = idx + 1;
                end
            end
            
            % 2) Edge features (simple gradient)
            [grad_x, grad_y] = gradient(depth_image);
            edge_strength = sqrt(grad_x.^2 + grad_y.^2);
            edge_features = [mean(edge_strength(:)); max(edge_strength(:))];
            
            % 3) Statistical features
            depth_stats = [
                min(depth_image(:));
                mean(depth_image(:));
                std(depth_image(:))
            ];
            
            % Combine features
            features = [patch_features; edge_features; depth_stats];
            
            % Normalize
            features = features / obj.max_range;
        end
        
        function local_patch = extract_local_patch(obj, occupancy_grid, drone_state, map_bounds, map_resolution)
            %% Extract Local Occupancy Patch Around Drone
            
            pos = drone_state(1:3);
            patch_size = [16, 16, 8];  % Smaller than full observation
            
            % Convert position to grid coordinates
            x_center = round((pos(1) - map_bounds(1)) / map_resolution) + 1;
            y_center = round((pos(2) - map_bounds(3)) / map_resolution) + 1;
            z_center = round((pos(3) - map_bounds(5)) / map_resolution) + 1;
            
            % Extract patch
            [x_size, y_size, z_size] = size(occupancy_grid);
            
            x_start = max(1, x_center - patch_size(1)/2);
            x_end = min(x_size, x_center + patch_size(1)/2 - 1);
            y_start = max(1, y_center - patch_size(2)/2);
            y_end = min(y_size, y_center + patch_size(2)/2 - 1);
            z_start = max(1, z_center - patch_size(3)/2);
            z_end = min(z_size, z_center + patch_size(3)/2 - 1);
            
            local_patch = occupancy_grid(x_start:x_end, y_start:y_end, z_start:z_end);
        end
        
        function features = compress_visual_features(obj, local_patch)
            %% Compress Local Patch to Visual Features
            
            % 1) Occupancy density in different regions
            [x_size, y_size, z_size] = size(local_patch);
            
            % Divide into regions
            regions = {
                local_patch(1:x_size/2, 1:y_size/2, :);           % Front-left
                local_patch(1:x_size/2, y_size/2+1:end, :);       % Front-right
                local_patch(x_size/2+1:end, 1:y_size/2, :);       % Back-left
                local_patch(x_size/2+1:end, y_size/2+1:end, :);   % Back-right
            };
            
            region_densities = zeros(4, 1);
            for i = 1:4
                region_densities(i) = sum(regions{i}(:)) / numel(regions{i});
            end
            
            % 2) Height profile (max occupancy in each XY column)
            height_profile = squeeze(max(local_patch, [], 3));
            height_features = [
                max(height_profile(:));
                mean(height_profile(:));
                sum(height_profile(:) > 0) / numel(height_profile)
            ];
            
            % 3) Connectivity features (simple)
            connectivity = sum(local_patch(:)) / numel(local_patch);
            
            % Combine features
            features = [region_densities; height_features; connectivity];
        end
        
        function state_features = extract_minimal_state_features(obj, drone_state)
            %% Extract Minimal State Features for Limited-Sensor Drone
            
            % Only include what a simple drone might know
            velocity = drone_state(4:6);
            roll_pitch = drone_state(7:8);  % Exclude yaw (no compass)
            
            % Normalize
            vel_norm = velocity / 5.0;  % Assume 5 m/s max
            att_norm = roll_pitch / (pi/4);  % Assume ±45° max
            
            state_features = [vel_norm; att_norm];
        end
    end
end
