%% deployment_interface.m - Real-World Deployment Interface
% PURPOSE
% Provides a clean interface for deploying trained RL policies on real drones
% with limited sensors. Handles sensor data processing, policy execution,
% and safety monitoring for real-world deployment.
%
% This implements the PDF requirement for hardware deployment with
% limited sensing capabilities.

classdef deployment_interface < handle
    properties
        % Deployment configuration
        params
        
        % Trained models
        policy_model        % Main RL policy or BC policy
        policy_type         % 'rl_agent', 'behavior_cloning', 'hybrid'
        backup_controller   % Safety backup controller
        
        % Sensor interfaces
        limited_observer    % Limited sensor processor
        sensor_buffer       % Recent sensor readings buffer
        sensor_latency_compensation
        
        % State estimation
        ekf_estimator      % Optional EKF for state estimation
        state_buffer       % Recent state estimates buffer
        uncertainty_monitor % EKF uncertainty monitoring
        
        % Safety and monitoring
        safety_monitor     % Safety bounds checking
        emergency_stop     % Emergency stop flag
        last_safe_command  % Last known safe command
        
        % Performance monitoring
        execution_stats    % Real-time performance stats
        command_history    % Recent command history
        
        % Hardware interface placeholders
        hardware_interface % Hardware-specific interface
    end
    
    methods
        function obj = deployment_interface(params, policy_model, policy_type)
            obj.params = params;
            obj.policy_model = policy_model;
            obj.policy_type = policy_type;
            
            % Initialize components
            obj.initialize_sensor_interface();
            obj.initialize_state_estimation();
            obj.initialize_safety_systems();
            obj.initialize_monitoring();
            
            % Initialize hardware interface (placeholder)
            obj.initialize_hardware_interface();
            
            fprintf('Deployment interface initialized\n');
            fprintf('  Policy type: %s\n', obj.policy_type);
            fprintf('  Sensor mode: %s\n', params.rl.sensor_mode);
            fprintf('  Safety monitoring: ENABLED\n');
        end
        
        function initialize_sensor_interface(obj)
            %% Initialize Limited Sensor Interface
            
            % Create limited sensor observer for processing sensor data
            obj.limited_observer = limited_sensor_observer(obj.params);
            
            % Initialize sensor buffer for latency compensation
            buffer_size = 50;  % Keep 50 recent readings
            obj.sensor_buffer = struct();
            obj.sensor_buffer.timestamps = zeros(1, buffer_size);
            obj.sensor_buffer.raw_data = cell(1, buffer_size);
            obj.sensor_buffer.processed_data = zeros(obj.limited_observer.num_beams, buffer_size);
            obj.sensor_buffer.index = 1;
            obj.sensor_buffer.count = 0;
            
            % Latency compensation
            obj.sensor_latency_compensation = struct();
            obj.sensor_latency_compensation.enabled = true;
            obj.sensor_latency_compensation.max_latency = 0.1; % 100ms max
            obj.sensor_latency_compensation.prediction_steps = 5;
        end
        
        function initialize_state_estimation(obj)
            %% Initialize State Estimation System
            
            % Optional EKF for state estimation (can be disabled for pure limited-sensor mode)
            if obj.params.rl.limited_sensors.include_ekf_uncertainty
                obj.ekf_estimator = rl_ekf_integration(obj.params);
                obj.uncertainty_monitor = struct();
                obj.uncertainty_monitor.max_position_std = 2.0;  % 2m max position uncertainty
                obj.uncertainty_monitor.max_attitude_std = deg2rad(10);  % 10° max attitude uncertainty
            else
                obj.ekf_estimator = [];
                obj.uncertainty_monitor = [];
            end
            
            % State buffer
            buffer_size = 100;
            obj.state_buffer = struct();
            obj.state_buffer.timestamps = zeros(1, buffer_size);
            obj.state_buffer.states = zeros(9, buffer_size);  % [pos; vel; att]
            obj.state_buffer.uncertainties = zeros(9, buffer_size);
            obj.state_buffer.index = 1;
            obj.state_buffer.count = 0;
        end
        
        function initialize_safety_systems(obj)
            %% Initialize Safety Monitoring Systems
            
            obj.safety_monitor = struct();
            
            % Safety bounds
            obj.safety_monitor.max_altitude = obj.params.rl.safety.max_altitude;
            obj.safety_monitor.min_altitude = obj.params.rl.safety.min_altitude;
            obj.safety_monitor.max_velocity = 8.0;  % 8 m/s max velocity
            obj.safety_monitor.max_acceleration = 5.0;  % 5 m/s² max acceleration
            obj.safety_monitor.max_angular_rate = deg2rad(120);  % 120°/s max rotation
            obj.safety_monitor.emergency_stop_distance = obj.params.rl.safety.emergency_stop_distance;
            
            % Safety state
            obj.emergency_stop = false;
            obj.last_safe_command = [0; 0; 0; 0];  % Hover command
            
            % Backup controller (simple hover/land controller)
            obj.backup_controller = struct();
            obj.backup_controller.hover_gains = [1.0; 1.0; 2.0; 1.0];  % Position and yaw gains
            obj.backup_controller.land_rate = -0.5;  % 0.5 m/s descent rate
        end
        
        function initialize_monitoring(obj)
            %% Initialize Performance Monitoring
            
            obj.execution_stats = struct();
            obj.execution_stats.loop_frequency = 0;
            obj.execution_stats.policy_execution_time = 0;
            obj.execution_stats.sensor_processing_time = 0;
            obj.execution_stats.safety_check_time = 0;
            obj.execution_stats.total_commands_sent = 0;
            obj.execution_stats.emergency_stops = 0;
            
            % Command history for analysis
            history_size = 1000;
            obj.command_history = struct();
            obj.command_history.timestamps = zeros(1, history_size);
            obj.command_history.commands = zeros(4, history_size);
            obj.command_history.policy_outputs = zeros(4, history_size);
            obj.command_history.safety_overrides = false(1, history_size);
            obj.command_history.index = 1;
            obj.command_history.count = 0;
        end
        
        function initialize_hardware_interface(obj)
            %% Initialize Hardware Interface (Placeholder)
            
            obj.hardware_interface = struct();
            obj.hardware_interface.connected = false;
            obj.hardware_interface.last_heartbeat = 0;
            obj.hardware_interface.communication_timeout = 1.0;  % 1 second timeout
            
            % This would be replaced with actual hardware drivers
            fprintf('Hardware interface initialized (simulation mode)\n');
        end
        
        function command = execute_policy(obj, sensor_data, current_time)
            %% Main Policy Execution Loop
            
            tic;
            
            % 1) Process sensor data
            sensor_start = tic;
            processed_observation = obj.process_sensor_data(sensor_data, current_time);
            obj.execution_stats.sensor_processing_time = toc(sensor_start);
            
            % 2) Execute policy
            policy_start = tic;
            policy_command = obj.run_policy(processed_observation);
            obj.execution_stats.policy_execution_time = toc(policy_start);
            
            % 3) Safety checks and monitoring
            safety_start = tic;
            safe_command = obj.apply_safety_checks(policy_command, current_time);
            obj.execution_stats.safety_check_time = toc(safety_start);
            
            % 4) Update monitoring
            obj.update_monitoring(policy_command, safe_command, current_time);
            
            % 5) Store command history
            obj.store_command_history(policy_command, safe_command, current_time);
            
            % Return final command
            command = safe_command;
            
            % Update execution statistics
            total_time = toc;
            obj.execution_stats.loop_frequency = 1.0 / total_time;
            obj.execution_stats.total_commands_sent = obj.execution_stats.total_commands_sent + 1;
        end
        
        function observation = process_sensor_data(obj, sensor_data, current_time)
            %% Process Raw Sensor Data into Policy Observation
            
            % Store in sensor buffer
            obj.store_sensor_data(sensor_data, current_time);
            
            % Apply latency compensation if enabled
            if obj.sensor_latency_compensation.enabled
                % Simple latency compensation (placeholder implementation)
                compensated_data = sensor_data;  % For now, just pass through
            else
                compensated_data = sensor_data;
            end
            
            % Convert to limited sensor observation based on sensor mode
            switch obj.params.rl.sensor_mode
                case 'ranges'
                    observation = obj.process_range_sensor_data(compensated_data);
                case 'depth'
                    observation = obj.process_depth_sensor_data(compensated_data);
                case 'limited'
                    observation = obj.process_limited_sensor_data(compensated_data);
                otherwise
                    error('Unknown sensor mode: %s', obj.params.rl.sensor_mode);
            end
            
            % Add state information if available
            if ~isempty(obj.ekf_estimator)
                state_features = obj.get_current_state_features();
                observation = [observation; state_features];
            end
        end
        
        function policy_command = run_policy(obj, observation)
            %% Execute Policy on Processed Observation
            
            switch obj.policy_type
                case 'rl_agent'
                    policy_command = obj.policy_model.select_action(observation, false);  % No exploration
                    
                case 'behavior_cloning'
                    policy_command = obj.policy_model.predict_action(observation);
                    
                case 'hybrid'
                    % Use BC for safe regions, RL for complex navigation
                    uncertainty = obj.get_current_uncertainty();
                    if uncertainty < 0.5  % Low uncertainty, use RL
                        policy_command = obj.policy_model.rl_policy.select_action(observation, false);
                    else  % High uncertainty, use safer BC policy
                        policy_command = obj.policy_model.bc_policy.predict_action(observation);
                    end
                    
                otherwise
                    error('Unknown policy type: %s', obj.policy_type);
            end
            
            % Ensure command is in correct format
            policy_command = policy_command(:);  % Column vector
            if length(policy_command) ~= 4
                error('Policy output must be 4-dimensional [vx, vy, vz, yaw_rate]');
            end
        end
        
        function safe_command = apply_safety_checks(obj, policy_command, current_time)
            %% Apply Safety Checks and Generate Safe Command
            
            % Start with policy command
            safe_command = policy_command;
            safety_override = false;
            
            % 1) Check for emergency stop conditions
            if obj.check_emergency_conditions(current_time)
                safe_command = obj.get_emergency_stop_command();
                safety_override = true;
                obj.emergency_stop = true;
                obj.execution_stats.emergency_stops = obj.execution_stats.emergency_stops + 1;
            end
            
            % 2) Velocity limits
            max_vel = obj.safety_monitor.max_velocity;
            if norm(safe_command(1:3)) > max_vel
                safe_command(1:3) = safe_command(1:3) * max_vel / norm(safe_command(1:3));
                safety_override = true;
            end
            
            % 3) Angular rate limits
            max_yaw_rate = obj.safety_monitor.max_angular_rate;
            if abs(safe_command(4)) > max_yaw_rate
                safe_command(4) = sign(safe_command(4)) * max_yaw_rate;
                safety_override = true;
            end
            
            % 4) Altitude bounds checking (if state available)
            if ~isempty(obj.ekf_estimator)
                current_state = obj.get_current_state();
                altitude = -current_state(3);  % NED to altitude
                
                if altitude > obj.safety_monitor.max_altitude
                    safe_command(3) = max(safe_command(3), -1.0);  % Force descent
                    safety_override = true;
                elseif altitude < obj.safety_monitor.min_altitude
                    safe_command(3) = min(safe_command(3), 1.0);   % Force ascent
                    safety_override = true;
                end
            end
            
            % 5) Store last safe command
            if ~safety_override
                obj.last_safe_command = safe_command;
            end
        end
        
        function emergency_condition = check_emergency_conditions(obj, current_time)
            %% Check for Emergency Stop Conditions
            
            emergency_condition = false;
            
            % 1) Hardware communication timeout
            if obj.hardware_interface.connected && ...
               (current_time - obj.hardware_interface.last_heartbeat) > obj.hardware_interface.communication_timeout
                fprintf('EMERGENCY: Hardware communication timeout\n');
                emergency_condition = true;
            end
            
            % 2) EKF divergence (if using EKF)
            if ~isempty(obj.ekf_estimator)
                uncertainty = obj.get_current_uncertainty();
                if uncertainty > 10.0  % Very high uncertainty
                    fprintf('EMERGENCY: State estimation diverged\n');
                    emergency_condition = true;
                end
            end
            
            % 3) Sensor failure detection
            if obj.detect_sensor_failure()
                fprintf('EMERGENCY: Critical sensor failure detected\n');
                emergency_condition = true;
            end
            
            % 4) Manual emergency stop (would be set by external monitor)
            if obj.emergency_stop
                emergency_condition = true;
            end
        end
        
        function sensor_failure = detect_sensor_failure(obj)
            %% Detect Critical Sensor Failures
            
            sensor_failure = false;
            
            % Check if no valid sensor readings in recent buffer
            if obj.sensor_buffer.count > 0
                recent_data = obj.sensor_buffer.processed_data(:, max(1, obj.sensor_buffer.count-5):obj.sensor_buffer.count);
                
                % Check for all-zero or all-max readings (sensor failure indicators)
                if all(recent_data(:) == 0) || all(recent_data(:) == obj.limited_observer.max_range)
                    sensor_failure = true;
                end
            end
        end
        
        function emergency_command = get_emergency_stop_command(obj)
            %% Get Emergency Stop Command
            
            % Command to hover in place or gentle landing
            if ~isempty(obj.ekf_estimator)
                current_state = obj.get_current_state();
                altitude = -current_state(3);
                
                if altitude > 2.0  % If high enough, land slowly
                    emergency_command = [0; 0; obj.backup_controller.land_rate; 0];
                else  % If low, hover
                    emergency_command = [0; 0; 0; 0];
                end
            else
                % No state info, just hover
                emergency_command = [0; 0; 0; 0];
            end
        end
        
        function store_sensor_data(obj, sensor_data, timestamp)
            %% Store Sensor Data in Buffer
            
            idx = obj.sensor_buffer.index;
            obj.sensor_buffer.timestamps(idx) = timestamp;
            obj.sensor_buffer.raw_data{idx} = sensor_data;
            
            % Process data if possible
            if isfield(sensor_data, 'ranges')
                obj.sensor_buffer.processed_data(:, idx) = sensor_data.ranges;
            end
            
            % Update buffer indices
            obj.sensor_buffer.index = mod(idx, length(obj.sensor_buffer.timestamps)) + 1;
            obj.sensor_buffer.count = min(obj.sensor_buffer.count + 1, length(obj.sensor_buffer.timestamps));
        end
        
        function observation = process_range_sensor_data(obj, sensor_data)
            %% Process Range Sensor Data (Lidar-like)
            
            if isfield(sensor_data, 'ranges')
                ranges = sensor_data.ranges;
                
                % Normalize ranges
                normalized_ranges = ranges / obj.limited_observer.max_range;
                
                % Apply noise and quantization if in simulation
                if obj.params.rl.advanced.domain_randomization
                    % Add some noise for realism
                    noise_level = 0.02;  % 2% noise
                    normalized_ranges = normalized_ranges + noise_level * randn(size(normalized_ranges));
                    normalized_ranges = max(0, min(normalized_ranges, 1));
                end
                
                observation = normalized_ranges;
            else
                % No range data available, return zeros
                observation = zeros(obj.limited_observer.num_beams, 1);
            end
        end
        
        function observation = process_depth_sensor_data(obj, sensor_data)
            %% Process Depth Camera Data
            
            if isfield(sensor_data, 'depth_image')
                depth_image = sensor_data.depth_image;
                
                % Compress depth image to features
                observation = obj.limited_observer.compress_depth_to_features(depth_image);
            else
                % No depth data, return default observation
                observation = zeros(100, 1);  % Default depth feature size
            end
        end
        
        function observation = process_limited_sensor_data(obj, sensor_data)
            %% Process General Limited Sensor Data
            
            % Combine range and minimal state information
            range_obs = obj.process_range_sensor_data(sensor_data);
            
            % Add minimal state features if available
            if isfield(sensor_data, 'imu')
                % Extract velocity and attitude from IMU integration
                imu_features = obj.extract_imu_features(sensor_data.imu);
                observation = [range_obs; imu_features];
            else
                observation = range_obs;
            end
        end
        
        function features = extract_imu_features(obj, imu_data)
            %% Extract Minimal Features from IMU Data
            
            % Simple integration for velocity estimation (simplified)
            accel = imu_data.accel;
            gyro = imu_data.gyro;
            
            % Normalize accelerometer readings (remove gravity estimate)
            accel_norm = accel / 9.81;
            
            % Normalize gyro readings
            gyro_norm = gyro / deg2rad(200);  % Assume max 200°/s
            
            features = [accel_norm; gyro_norm(1:2)];  % Exclude yaw rate
        end
        
        function current_state = get_current_state(obj)
            %% Get Current State Estimate
            
            if ~isempty(obj.ekf_estimator) && obj.state_buffer.count > 0
                idx = obj.state_buffer.index - 1;
                if idx == 0, idx = length(obj.state_buffer.timestamps); end
                current_state = obj.state_buffer.states(:, idx);
            else
                current_state = zeros(9, 1);  % Default state
            end
        end
        
        function uncertainty = get_current_uncertainty(obj)
            %% Get Current State Uncertainty
            
            if ~isempty(obj.ekf_estimator) && obj.state_buffer.count > 0
                idx = obj.state_buffer.index - 1;
                if idx == 0, idx = length(obj.state_buffer.timestamps); end
                uncertainty_vec = obj.state_buffer.uncertainties(:, idx);
                uncertainty = norm(uncertainty_vec);
            else
                uncertainty = 0.0;
            end
        end
        
        function state_features = get_current_state_features(obj)
            %% Get Current State Features for Policy
            
            current_state = obj.get_current_state();
            
            % Extract minimal features (velocity and attitude)
            vel = current_state(4:6) / 5.0;  % Normalize by max velocity
            att = current_state(7:8) / (pi/4);  % Normalize roll/pitch, exclude yaw
            
            state_features = [vel; att];
        end
        
        function update_monitoring(obj, policy_command, safe_command, current_time)
            %% Update Performance Monitoring
            
            % Update execution statistics (already computed in main loop)
            
            % Check for safety overrides
            if norm(policy_command - safe_command) > 1e-6
                fprintf('Safety override at t=%.3f: Policy=[%.3f %.3f %.3f %.3f] -> Safe=[%.3f %.3f %.3f %.3f]\n', ...
                        current_time, policy_command, safe_command);
            end
        end
        
        function store_command_history(obj, policy_command, safe_command, timestamp)
            %% Store Command History for Analysis
            
            idx = obj.command_history.index;
            obj.command_history.timestamps(idx) = timestamp;
            obj.command_history.policy_outputs(:, idx) = policy_command;
            obj.command_history.commands(:, idx) = safe_command;
            obj.command_history.safety_overrides(idx) = norm(policy_command - safe_command) > 1e-6;
            
            obj.command_history.index = mod(idx, length(obj.command_history.timestamps)) + 1;
            obj.command_history.count = min(obj.command_history.count + 1, length(obj.command_history.timestamps));
        end
        
        function save_deployment_log(obj, filename)
            %% Save Deployment Log for Analysis
            
            log_data = struct();
            log_data.execution_stats = obj.execution_stats;
            log_data.command_history = obj.command_history;
            log_data.params = obj.params.rl;
            log_data.policy_type = obj.policy_type;
            
            save(filename, 'log_data');
            fprintf('Deployment log saved to: %s\n', filename);
        end
        
        function print_status(obj)
            %% Print Current Status
            
            fprintf('=== Deployment Interface Status ===\n');
            fprintf('Policy type: %s\n', obj.policy_type);
            if obj.emergency_stop
                fprintf('Emergency stop: ACTIVE\n');
            else
                fprintf('Emergency stop: Normal\n');
            end
            fprintf('Loop frequency: %.1f Hz\n', obj.execution_stats.loop_frequency);
            fprintf('Commands sent: %d\n', obj.execution_stats.total_commands_sent);
            fprintf('Emergency stops: %d\n', obj.execution_stats.emergency_stops);
            
            if ~isempty(obj.ekf_estimator)
                uncertainty = obj.get_current_uncertainty();
                fprintf('State uncertainty: %.3f\n', uncertainty);
            end
            
            fprintf('Sensor buffer: %d/%d samples\n', obj.sensor_buffer.count, length(obj.sensor_buffer.timestamps));
        end
    end
end
