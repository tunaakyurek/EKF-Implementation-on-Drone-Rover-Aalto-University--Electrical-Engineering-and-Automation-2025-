classdef expert_map_generator < handle
    %% expert_map_generator - Simple Map Generator for Expert Demonstrations
    % PURPOSE
    % Generate cylindrical obstacle maps for expert flight demonstrations.
    
    properties
        map_bounds
    end
    
    methods
        function obj = expert_map_generator()
            %% Initialize Expert Map Generator
            
            % Default map bounds
            obj.map_bounds = [-40, 40, -40, 40, 0, 25];  % [x_min, x_max, y_min, y_max, z_min, z_max]
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
