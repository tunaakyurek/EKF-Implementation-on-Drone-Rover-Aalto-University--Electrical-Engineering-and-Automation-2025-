%% map_generator.m - Advanced 3D Map Generation for RL Training
% PURPOSE
% Generates diverse 3D environments with obstacles and terrain for drone
% navigation training. Supports multiple scenarios and difficulty levels.
%
% FEATURES
% - Fractal terrain generation
% - Multiple obstacle types (forest, urban, canyon, mixed)
% - Procedural generation with reproducible seeds
% - Collision-free path validation
% - Map serialization and loading
%
% USAGE
%   map_gen = map_generator(params);
%   [occupancy_grid, height_map] = map_gen.generate_map(scenario, seed);

classdef map_generator < handle
    properties
        params
        map_resolution
        map_bounds
        x_cells
        y_cells
        z_cells
        
        % Terrain generation
        fractal_octaves
        fractal_persistence
        fractal_lacunarity
        
        % Obstacle parameters
        obstacle_densities
        obstacle_scenarios
        
        % Path planning for validation
        path_planner
    end
    
    methods
        function obj = map_generator(params)
            obj.params = params;
            obj.map_resolution = params.rl.map_resolution;
            obj.map_bounds = params.rl.map_bounds;
            
            % Calculate grid dimensions
            obj.x_cells = ceil((obj.map_bounds(2) - obj.map_bounds(1)) / obj.map_resolution);
            obj.y_cells = ceil((obj.map_bounds(4) - obj.map_bounds(3)) / obj.map_resolution);
            obj.z_cells = ceil((obj.map_bounds(6) - obj.map_bounds(5)) / obj.map_resolution);
            
            % Fractal terrain parameters
            obj.fractal_octaves = 6;
            obj.fractal_persistence = 0.6;
            obj.fractal_lacunarity = 2.0;
            
            % Setup obstacle scenarios
            obj.setup_obstacle_scenarios();
            
            fprintf('Map generator initialized: %dx%dx%d grid\n', ...
                    obj.x_cells, obj.y_cells, obj.z_cells);
        end
        
        function [occupancy_grid, height_map, metadata] = generate_map(obj, scenario, seed, difficulty)
            %% Generate Complete 3D Map
            
            if nargin < 3, seed = randi(10000); end
            if nargin < 4, difficulty = 0.5; end
            
            % Set random seed for reproducible generation
            rng(seed);
            
            % Initialize grids
            occupancy_grid = zeros(obj.x_cells, obj.y_cells, obj.z_cells);
            
            % Generate base terrain
            height_map = obj.generate_fractal_terrain(difficulty);
            
            % Add terrain to occupancy grid
            occupancy_grid = obj.add_terrain_to_grid(occupancy_grid, height_map);
            
            % Add scenario-specific obstacles (Focus on navigation-friendly layouts)
            switch lower(scenario)
                case 'columns'
                    occupancy_grid = obj.add_forest_obstacles(occupancy_grid, difficulty);  % Cylindrical columns
                case 'pillars'
                    occupancy_grid = obj.add_rectangular_pillars(occupancy_grid, difficulty);
                case 'mixed'
                    occupancy_grid = obj.add_mixed_obstacles(occupancy_grid, difficulty);
                case 'forest'  % Legacy support
                    occupancy_grid = obj.add_forest_obstacles(occupancy_grid, difficulty);
                case 'urban'   % Legacy support
                    occupancy_grid = obj.add_urban_obstacles(occupancy_grid, difficulty);
                otherwise
                    % Default to cylindrical columns for best navigation training
                    occupancy_grid = obj.add_forest_obstacles(occupancy_grid, difficulty);
            end
            
            % Add boundary walls
            occupancy_grid = obj.add_boundary_walls(occupancy_grid);
            
            % Validate map (ensure navigable paths exist)
            [occupancy_grid, path_exists] = obj.validate_and_fix_map(occupancy_grid);
            
            % Create metadata
            metadata = struct();
            metadata.scenario = scenario;
            metadata.seed = seed;
            metadata.difficulty = difficulty;
            metadata.path_exists = path_exists;
            metadata.obstacle_density = sum(occupancy_grid(:)) / numel(occupancy_grid);
            metadata.generation_time = toc;
            
            fprintf('Generated %s map (seed=%d, difficulty=%.2f)\n', ...
                    scenario, seed, difficulty);
            fprintf('Obstacle density: %.1f%%, Path exists: %d\n', ...
                    metadata.obstacle_density * 100, path_exists);
        end
        
        function height_map = generate_fractal_terrain(obj, complexity)
            %% Generate Realistic Fractal Terrain
            
            % Create coordinate grids
            x_coords = linspace(obj.map_bounds(1), obj.map_bounds(2), obj.x_cells);
            y_coords = linspace(obj.map_bounds(3), obj.map_bounds(4), obj.y_cells);
            [X, Y] = meshgrid(x_coords, y_coords);
            X = X'; Y = Y';  % Transpose to match grid indexing
            
            % Initialize height map
            height_map = zeros(size(X));
            
            % Generate multiple octaves of noise
            amplitude = 1.0;
            frequency = 0.01 * (1 + complexity);  % Higher complexity = more variation
            
            for octave = 1:obj.fractal_octaves
                % Generate noise layer using sine/cosine functions
                % (In practice, you'd use Perlin or Simplex noise)
                noise_layer = obj.generate_noise_layer(X, Y, frequency);
                
                % Add to height map
                height_map = height_map + amplitude * noise_layer;
                
                % Update for next octave
                amplitude = amplitude * obj.fractal_persistence;
                frequency = frequency * obj.fractal_lacunarity;
            end
            
            % Normalize and scale height map
            height_map = height_map - min(height_map(:));
            height_map = height_map / max(height_map(:));
            
            % Apply terrain shaping based on scenario
            height_map = obj.apply_terrain_shaping(height_map, complexity);
            
            % Scale to desired height range
            max_height = obj.params.rl.max_terrain_height;
            height_map = height_map * max_height;
        end
        
        function noise = generate_noise_layer(obj, X, Y, frequency)
            %% Generate Single Noise Layer
            
            % Multi-frequency noise for more realistic terrain
            noise = sin(frequency * X) .* cos(frequency * Y) + ...
                   0.5 * sin(2.5 * frequency * X) .* sin(1.7 * frequency * Y) + ...
                   0.25 * sin(4.3 * frequency * X + pi/4) .* cos(3.1 * frequency * Y + pi/3);
            
            % Add some randomness
            noise = noise + 0.1 * randn(size(X));
            
            % Smooth slightly to avoid aliasing
            if exist('imgaussfilt', 'file')
                noise = imgaussfilt(noise, 0.5);
            end
        end
        
        function height_map = apply_terrain_shaping(obj, height_map, complexity)
            %% Apply Scenario-Specific Terrain Shaping
            
            [rows, cols] = size(height_map);
            
            % Create distance from center
            center_x = cols / 2;
            center_y = rows / 2;
            [X, Y] = meshgrid(1:cols, 1:rows);
            dist_from_center = sqrt((X - center_x).^2 + (Y - center_y).^2);
            max_dist = sqrt(center_x^2 + center_y^2);
            dist_normalized = dist_from_center / max_dist;
            
            % Apply shaping based on complexity
            if complexity > 0.7
                % High complexity: create valleys and ridges
                ridge_factor = sin(4 * pi * dist_normalized) .* cos(6 * pi * dist_normalized);
                height_map = height_map + 0.3 * ridge_factor;
            elseif complexity > 0.4
                % Medium complexity: gentle rolling hills
                hill_factor = 1 - 0.5 * dist_normalized.^2;
                height_map = height_map .* hill_factor;
            else
                % Low complexity: mostly flat with small variations
                height_map = 0.3 * height_map .* (1 - 0.8 * dist_normalized);
            end
            
            % Ensure non-negative heights
            height_map = max(height_map, 0);
        end
        
        function occupancy_grid = add_terrain_to_grid(obj, occupancy_grid, height_map)
            %% Convert Height Map to 3D Occupancy Grid
            
            for i = 1:obj.x_cells
                for j = 1:obj.y_cells
                    if i <= size(height_map, 1) && j <= size(height_map, 2)
                        terrain_height = height_map(i, j);
                        terrain_cells = round(terrain_height / obj.map_resolution);
                        terrain_cells = min(terrain_cells, obj.z_cells);
                        
                        if terrain_cells > 0
                            occupancy_grid(i, j, 1:terrain_cells) = 1;
                        end
                    end
                end
            end
        end
        
        function occupancy_grid = add_forest_obstacles(obj, occupancy_grid, difficulty)
            %% Add Tree-like Obstacles
            
            base_density = obj.params.rl.obstacle_density;
            density = base_density * (0.5 + 0.5 * difficulty);
            
            num_trees = round(density * obj.x_cells * obj.y_cells / 200);
            
            for tree = 1:num_trees
                % Random tree position (avoid edges)
                margin = 3;
                x = randi([margin, obj.x_cells - margin]);
                y = randi([margin, obj.y_cells - margin]);
                
                % Tree parameters
                tree_radius = 1 + randi(2);  % 1-3 cells
                tree_height = 8 + randi(round(obj.z_cells/3));  % Variable height
                
                % Get terrain height at this location
                terrain_z = find(occupancy_grid(x, y, :), 1, 'last');
                if isempty(terrain_z), terrain_z = 1; end
                
                % Create cylindrical tree
                for dx = -tree_radius:tree_radius
                    for dy = -tree_radius:tree_radius
                        if sqrt(dx^2 + dy^2) <= tree_radius
                            xi = x + dx;
                            yi = y + dy;
                            
                            if xi >= 1 && xi <= obj.x_cells && yi >= 1 && yi <= obj.y_cells
                                z_start = terrain_z + 1;
                                z_end = min(terrain_z + tree_height, obj.z_cells);
                                if z_start <= z_end
                                    occupancy_grid(xi, yi, z_start:z_end) = 1;
                                end
                            end
                        end
                    end
                end
            end
        end
        
        function occupancy_grid = add_urban_obstacles(obj, occupancy_grid, difficulty)
            %% Add Building-like Obstacles
            
            base_density = obj.params.rl.obstacle_density;
            density = base_density * (0.3 + 0.7 * difficulty);
            
            num_buildings = round(density * obj.x_cells * obj.y_cells / 400);
            
            for building = 1:num_buildings
                % Building size based on difficulty
                min_size = 4;
                max_size = 6 + round(4 * difficulty);
                
                width = randi([min_size, max_size]);
                length = randi([min_size, max_size]);
                height = 5 + randi(round(obj.z_cells/2));
                
                % Random position (ensure building fits)
                x_start = randi([1, max(1, obj.x_cells - width)]);
                y_start = randi([1, max(1, obj.y_cells - length)]);
                
                % Get maximum terrain height in building footprint
                terrain_z = 1;
                for xi = x_start:min(x_start + width - 1, obj.x_cells)
                    for yi = y_start:min(y_start + length - 1, obj.y_cells)
                        terrain_cell = find(occupancy_grid(xi, yi, :), 1, 'last');
                        if ~isempty(terrain_cell)
                            terrain_z = max(terrain_z, terrain_cell);
                        end
                    end
                end
                
                % Create building
                x_end = min(x_start + width - 1, obj.x_cells);
                y_end = min(y_start + length - 1, obj.y_cells);
                z_end = min(terrain_z + height, obj.z_cells);
                
                occupancy_grid(x_start:x_end, y_start:y_end, terrain_z+1:z_end) = 1;
                
                % Add some architectural details (difficulty dependent)
                if difficulty > 0.6 && rand() < 0.3
                    % Add building extensions or complex shapes
                    obj.add_building_complexity(occupancy_grid, x_start, y_start, x_end, y_end, terrain_z, z_end);
                end
            end
        end
        
        function occupancy_grid = add_canyon_obstacles(obj, occupancy_grid, difficulty)
            %% Add Canyon-like Vertical Wall Obstacles
            
            % Create canyon walls and narrow passages
            num_canyons = 2 + round(3 * difficulty);
            
            for canyon = 1:num_canyons
                % Random canyon direction
                if rand() < 0.5
                    % Vertical canyon
                    x_pos = randi([round(obj.x_cells/4), round(3*obj.x_cells/4)]);
                    width = 2 + randi(4);
                    
                    for y = 1:obj.y_cells
                        wall_height = round(obj.z_cells * (0.3 + 0.5 * difficulty));
                        for w = 0:width-1
                            xi = x_pos + w;
                            if xi <= obj.x_cells
                                terrain_z = find(occupancy_grid(xi, y, :), 1, 'last');
                                if isempty(terrain_z), terrain_z = 1; end
                                z_end = min(terrain_z + wall_height, obj.z_cells);
                                occupancy_grid(xi, y, terrain_z+1:z_end) = 1;
                            end
                        end
                    end
                else
                    % Horizontal canyon
                    y_pos = randi([round(obj.y_cells/4), round(3*obj.y_cells/4)]);
                    width = 2 + randi(4);
                    
                    for x = 1:obj.x_cells
                        wall_height = round(obj.z_cells * (0.3 + 0.5 * difficulty));
                        for w = 0:width-1
                            yi = y_pos + w;
                            if yi <= obj.y_cells
                                terrain_z = find(occupancy_grid(x, yi, :), 1, 'last');
                                if isempty(terrain_z), terrain_z = 1; end
                                z_end = min(terrain_z + wall_height, obj.z_cells);
                                occupancy_grid(x, yi, terrain_z+1:z_end) = 1;
                            end
                        end
                    end
                end
            end
        end
        
        function occupancy_grid = add_mixed_obstacles(obj, occupancy_grid, difficulty)
            %% Add Mixed Cylindrical and Rectangular Obstacles
            
            % Primary: Cylindrical columns (main navigation challenge)
            occupancy_grid = obj.add_forest_obstacles(occupancy_grid, difficulty * 0.8);
            
            % Secondary: Some rectangular obstacles for variety
            if difficulty > 0.4
                occupancy_grid = obj.add_rectangular_pillars(occupancy_grid, difficulty * 0.3);
            end
            
            % Tertiary: Minimal random clutter
            if difficulty > 0.7
                occupancy_grid = obj.add_navigation_challenges(occupancy_grid, difficulty * 0.2);
            end
        end
        
        function occupancy_grid = add_rectangular_pillars(obj, occupancy_grid, difficulty)
            %% Add Rectangular Pillar Obstacles
            
            [x_cells, y_cells, z_cells] = size(occupancy_grid);
            num_pillars = round(difficulty * x_cells * y_cells / 800);  % Fewer rectangular obstacles
            
            for pillar = 1:num_pillars
                % Pillar dimensions
                width = 2 + randi(2);   % 2-4 cells wide
                length = 2 + randi(2);  % 2-4 cells long
                height = round(z_cells * (0.5 + 0.4 * difficulty));
                
                % Random position with margins
                margin = 6;
                x_start = randi([margin, max(margin, x_cells - width - margin)]);
                y_start = randi([margin, max(margin, y_cells - length - margin)]);
                
                % Get terrain height
                terrain_z = 1;
                for xi = x_start:min(x_start + width - 1, x_cells)
                    for yi = y_start:min(y_start + length - 1, y_cells)
                        terrain_cell = find(occupancy_grid(xi, yi, :), 1, 'last');
                        if ~isempty(terrain_cell)
                            terrain_z = max(terrain_z, terrain_cell);
                        end
                    end
                end
                
                % Create pillar
                x_end = min(x_start + width - 1, x_cells);
                y_end = min(y_start + length - 1, y_cells);
                z_end = min(terrain_z + height, z_cells);
                
                occupancy_grid(x_start:x_end, y_start:y_end, terrain_z+1:z_end) = 1;
            end
        end
        
        function occupancy_grid = add_navigation_challenges(obj, occupancy_grid, difficulty)
            %% Add Small Navigation Challenges (Overhangs, Arches)
            
            [x_cells, y_cells, z_cells] = size(occupancy_grid);
            num_challenges = round(difficulty * 5);  % Just a few challenges
            
            for challenge = 1:num_challenges
                challenge_type = randi(2);  % 1=overhang, 2=arch
                
                if challenge_type == 1
                    % Create overhang
                    x = randi([10, x_cells - 10]);
                    y = randi([10, y_cells - 10]);
                    overhang_size = 3;
                    overhang_height = round(z_cells * 0.6);
                    
                    for dx = -overhang_size:overhang_size
                        for dy = -overhang_size:overhang_size
                            xi = x + dx;
                            yi = y + dy;
                            if xi >= 1 && xi <= x_cells && yi >= 1 && yi <= y_cells
                                occupancy_grid(xi, yi, overhang_height:min(overhang_height+2, z_cells)) = 1;
                            end
                        end
                    end
                end
            end
        end
        
        function occupancy_grid = add_random_obstacles(obj, occupancy_grid, difficulty)
            %% Add Random Scattered Obstacles
            
            density = obj.params.rl.obstacle_density * difficulty / 100;
            num_obstacles = round(density * obj.x_cells * obj.y_cells);
            
            for obs = 1:num_obstacles
                x = randi(obj.x_cells);
                y = randi(obj.y_cells);
                
                obstacle_size = 1 + randi(3);  % 1-4 cells
                obstacle_height = 2 + randi(round(obj.z_cells/4));
                
                terrain_z = find(occupancy_grid(x, y, :), 1, 'last');
                if isempty(terrain_z), terrain_z = 1; end
                
                for dx = 0:obstacle_size-1
                    for dy = 0:obstacle_size-1
                        xi = x + dx;
                        yi = y + dy;
                        if xi <= obj.x_cells && yi <= obj.y_cells
                            z_end = min(terrain_z + obstacle_height, obj.z_cells);
                            occupancy_grid(xi, yi, terrain_z+1:z_end) = 1;
                        end
                    end
                end
            end
        end
        
        function occupancy_grid = add_boundary_walls(obj, occupancy_grid)
            %% Add Rectangular Boundary Walls with Navigation Openings
            
            [x_cells, y_cells, z_cells] = size(occupancy_grid);
            wall_height = round(z_cells * 0.7);  % Slightly lower for better navigation
            wall_thickness = 2;  % cells
            
            % Create rectangular boundary with proper thickness
            for t = 1:wall_thickness
                % X boundaries (North and South walls)
                if t <= x_cells && (x_cells - t + 1) >= 1
                    occupancy_grid(t, :, 1:wall_height) = 1;
                    occupancy_grid(x_cells - t + 1, :, 1:wall_height) = 1;
                end
                
                % Y boundaries (East and West walls)
                if t <= y_cells && (y_cells - t + 1) >= 1
                    occupancy_grid(:, t, 1:wall_height) = 1;
                    occupancy_grid(:, y_cells - t + 1, 1:wall_height) = 1;
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
                        occupancy_grid(1:wall_thickness, opening_range, :) = 0;
                        
                    case 2  % South wall opening
                        opening_center = randi([opening_size, y_cells - opening_size]);
                        opening_range = max(1, opening_center - opening_size/2):min(y_cells, opening_center + opening_size/2);
                        occupancy_grid(x_cells-wall_thickness+1:x_cells, opening_range, :) = 0;
                        
                    case 3  % East wall opening
                        opening_center = randi([opening_size, x_cells - opening_size]);
                        opening_range = max(1, opening_center - opening_size/2):min(x_cells, opening_center + opening_size/2);
                        occupancy_grid(opening_range, 1:wall_thickness, :) = 0;
                        
                    case 4  % West wall opening
                        opening_center = randi([opening_size, x_cells - opening_size]);
                        opening_range = max(1, opening_center - opening_size/2):min(x_cells, opening_center + opening_size/2);
                        occupancy_grid(opening_range, y_cells-wall_thickness+1:y_cells, :) = 0;
                end
            end
        end
        
        function [occupancy_grid, path_exists] = validate_and_fix_map(obj, occupancy_grid)
            %% Validate Map Has Navigable Paths
            
            % Simple path existence check using A* or flood fill
            % Find collision-free start and end points
            start_pos = obj.find_collision_free_position(occupancy_grid);
            end_pos = obj.find_collision_free_position(occupancy_grid);
            
            if isempty(start_pos) || isempty(end_pos)
                path_exists = false;
                % Try to create some clearance
                occupancy_grid = obj.create_emergency_clearance(occupancy_grid);
                return;
            end
            
            % Simple path planning to check connectivity
            path_exists = obj.check_path_exists(occupancy_grid, start_pos, end_pos);
            
            if ~path_exists
                % Create a guaranteed path
                occupancy_grid = obj.create_guaranteed_path(occupancy_grid, start_pos, end_pos);
                path_exists = true;
            end
        end
        
        function pos = find_collision_free_position(obj, occupancy_grid)
            %% Find Random Collision-Free Position
            
            max_attempts = 1000;
            margin = 2;  % cells
            
            for attempt = 1:max_attempts
                x = randi([margin, obj.x_cells - margin]);
                y = randi([margin, obj.y_cells - margin]);
                z = randi([margin, obj.z_cells - margin]);
                
                % Check if position and small neighborhood are free
                free = true;
                for dx = -1:1
                    for dy = -1:1
                        for dz = -1:1
                            xi = x + dx; yi = y + dy; zi = z + dz;
                            if xi >= 1 && xi <= obj.x_cells && ...
                               yi >= 1 && yi <= obj.y_cells && ...
                               zi >= 1 && zi <= obj.z_cells
                                if occupancy_grid(xi, yi, zi) == 1
                                    free = false;
                                    break;
                                end
                            end
                        end
                        if ~free, break; end
                    end
                    if ~free, break; end
                end
                
                if free
                    pos = [x, y, z];
                    return;
                end
            end
            
            pos = [];  % No collision-free position found
        end
        
        function exists = check_path_exists(obj, occupancy_grid, start_pos, end_pos)
            %% Simple Path Existence Check
            
            % Simplified 3D flood fill or line-of-sight check
            % For now, implement basic straight-line obstacle check
            
            % Create line between start and end
            steps = 100;
            path_x = linspace(start_pos(1), end_pos(1), steps);
            path_y = linspace(start_pos(2), end_pos(2), steps);
            path_z = linspace(start_pos(3), end_pos(3), steps);
            
            for i = 1:steps
                x = round(path_x(i));
                y = round(path_y(i));
                z = round(path_z(i));
                
                if x >= 1 && x <= obj.x_cells && ...
                   y >= 1 && y <= obj.y_cells && ...
                   z >= 1 && z <= obj.z_cells
                    if occupancy_grid(x, y, z) == 1
                        exists = false;
                        return;
                    end
                end
            end
            
            exists = true;
        end
        
        function occupancy_grid = create_guaranteed_path(obj, occupancy_grid, start_pos, end_pos)
            %% Create Clear Path Between Points
            
            % Clear a straight-line path with some width
            path_width = 3;  % cells
            
            steps = max(abs(end_pos - start_pos)) * 2;
            path_x = linspace(start_pos(1), end_pos(1), steps);
            path_y = linspace(start_pos(2), end_pos(2), steps);
            path_z = linspace(start_pos(3), end_pos(3), steps);
            
            for i = 1:steps
                center_x = round(path_x(i));
                center_y = round(path_y(i));
                center_z = round(path_z(i));
                
                % Clear area around path point
                for dx = -path_width:path_width
                    for dy = -path_width:path_width
                        for dz = -1:1  % Limited vertical clearance
                            x = center_x + dx;
                            y = center_y + dy;
                            z = center_z + dz;
                            
                            if x >= 1 && x <= obj.x_cells && ...
                               y >= 1 && y <= obj.y_cells && ...
                               z >= 1 && z <= obj.z_cells
                                occupancy_grid(x, y, z) = 0;
                            end
                        end
                    end
                end
            end
        end
        
        function setup_obstacle_scenarios(obj)
            %% Setup Different Obstacle Scenario Configurations
            
            obj.obstacle_scenarios = containers.Map();
            
            % Forest scenario
            obj.obstacle_scenarios('forest') = struct(...
                'density_scale', 1.2, ...
                'height_variation', 0.8, ...
                'cluster_tendency', 0.7);
            
            % Urban scenario
            obj.obstacle_scenarios('urban') = struct(...
                'density_scale', 0.8, ...
                'height_variation', 1.5, ...
                'cluster_tendency', 0.9);
            
            % Canyon scenario
            obj.obstacle_scenarios('canyon') = struct(...
                'density_scale', 0.6, ...
                'height_variation', 2.0, ...
                'cluster_tendency', 0.3);
            
            % Mixed scenario
            obj.obstacle_scenarios('mixed') = struct(...
                'density_scale', 1.0, ...
                'height_variation', 1.2, ...
                'cluster_tendency', 0.6);
        end
        
        function save_map(obj, occupancy_grid, height_map, metadata, filename)
            %% Save Generated Map to File
            
            map_data = struct();
            map_data.occupancy_grid = occupancy_grid;
            map_data.height_map = height_map;
            map_data.metadata = metadata;
            map_data.generator_params = obj.params.rl;
            map_data.map_bounds = obj.map_bounds;
            map_data.map_resolution = obj.map_resolution;
            
            save(filename, 'map_data');
            fprintf('Map saved to: %s\n', filename);
        end
        
        function [occupancy_grid, height_map, metadata] = load_map(obj, filename)
            %% Load Previously Generated Map
            
            loaded = load(filename);
            map_data = loaded.map_data;
            
            occupancy_grid = map_data.occupancy_grid;
            height_map = map_data.height_map;
            metadata = map_data.metadata;
            
            fprintf('Map loaded from: %s\n', filename);
            fprintf('Scenario: %s, Difficulty: %.2f\n', ...
                    metadata.scenario, metadata.difficulty);
        end
        
        function obstacles = generate_column_obstacles(obj, complexity)
            %% Generate Column Obstacles for Expert Demonstrations
            
            % Calculate number of obstacles based on complexity
            num_obstacles = round(complexity * 30);  % 0-30 obstacles
            
            obstacles = struct();
            obstacles.cylinders = {};
            obstacles.bounds = obj.map_bounds;
            
            % Generate random cylindrical obstacles
            for i = 1:num_obstacles
                % Random position
                x = obj.map_bounds(1) + (obj.map_bounds(2) - obj.map_bounds(1)) * rand();
                y = obj.map_bounds(3) + (obj.map_bounds(4) - obj.map_bounds(3)) * rand();
                z = obj.map_bounds(5) + (obj.map_bounds(6) - obj.map_bounds(5)) * rand();
                
                % Random size
                radius = 1.5 + 2.5 * rand();  % 1.5-4 meter radius
                height = 8.0 + 12.0 * rand();  % 8-20 meter height
                
                % Create cylinder
                cylinder = struct();
                cylinder.center = [x; y; z];
                cylinder.radius = radius;
                cylinder.height = height;
                
                obstacles.cylinders{end+1} = cylinder;
            end
            
            % Add boundary walls
            obstacles = obj.add_boundary_walls(obstacles);
        end
        
        function obstacles = add_boundary_walls(obj, obstacles)
            %% Add Boundary Walls
            
            % Create boundary walls as very thin cylinders
            wall_height = obj.map_bounds(6) - obj.map_bounds(5);
            wall_thickness = 0.5;
            
            % Left wall
            wall = struct();
            wall.center = [obj.map_bounds(1) - wall_thickness/2; (obj.map_bounds(3) + obj.map_bounds(4))/2; obj.map_bounds(5) + wall_height/2];
            wall.radius = wall_thickness;
            wall.height = wall_height;
            obstacles.cylinders{end+1} = wall;
            
            % Right wall
            wall = struct();
            wall.center = [obj.map_bounds(2) + wall_thickness/2; (obj.map_bounds(3) + obj.map_bounds(4))/2; obj.map_bounds(5) + wall_height/2];
            wall.radius = wall_thickness;
            wall.height = wall_height;
            obstacles.cylinders{end+1} = wall;
            
            % Front wall
            wall = struct();
            wall.center = [(obj.map_bounds(1) + obj.map_bounds(2))/2; obj.map_bounds(3) - wall_thickness/2; obj.map_bounds(5) + wall_height/2];
            wall.radius = wall_thickness;
            wall.height = wall_height;
            obstacles.cylinders{end+1} = wall;
            
            % Back wall
            wall = struct();
            wall.center = [(obj.map_bounds(1) + obj.map_bounds(2))/2; obj.map_bounds(4) + wall_thickness/2; obj.map_bounds(5) + wall_height/2];
            wall.radius = wall_thickness;
            wall.height = wall_height;
            obstacles.cylinders{end+1} = wall;
        end
    end
end
