%% rl_ekf_integration.m - Integration Layer between RL and EKF Systems
% PURPOSE
% Provides seamless integration between the RL-based navigation system
% and the existing EKF state estimation framework. Handles sensor simulation,
% state estimation updates, and realistic navigation dynamics.
%
% FEATURES
% - EKF state estimation with RL navigation commands
% - Realistic sensor modeling with failures and noise
% - GPS outage simulation for robust training
% - Uncertainty-aware navigation decisions
% - Integration with existing drone dynamics
%
% USAGE
%   integration = rl_ekf_integration(params);
%   [state_estimate, uncertainty] = integration.update_ekf(measurements);

classdef rl_ekf_integration < handle
    properties
        params
        
        % EKF state and covariance
        ekf_state           % [pos(3); vel(3); att(3)]
        ekf_covariance      % 9x9 covariance matrix
        
        % Sensor simulation
        sensor_states       % Internal sensor states and biases
        gps_available       % GPS availability flag
        gps_outage_timer    % Timer for GPS outage simulation
        mag_interference    % Magnetometer interference flag
        
        % State history for RL observations
        state_history       % Recent state estimates
        uncertainty_history % Recent uncertainty estimates
        measurement_history % Recent measurements
        
        % Integration with main simulation
        current_time
        last_update_time
        integration_dt
        
        % Performance monitoring
        estimation_errors   % Tracking estimation accuracy
        innovation_stats    % Innovation monitoring
        filter_health       % Overall filter health metrics
        
        % Professional flight controller
        flight_controller   % autonomous_flight_controller instance
        last_control_info   % Last control computation info
    end
    
    methods
        function obj = rl_ekf_integration(params)
            %% Initialize RL-EKF Integration
            
            obj.params = params;
            obj.integration_dt = params.Ts.physics;
            obj.current_time = 0;
            obj.last_update_time = 0;
            
            % Add necessary paths for drone dynamics functions
            [script_path, ~, ~] = fileparts(mfilename('fullpath'));
            parent_path = fileparts(script_path);
            addpath(parent_path);
            
            % Initialize filter health (robust 1x1 struct)
            obj.filter_health = struct( ...
                'overall_score', 1.0, ...
                'pos_rms', NaN, ...
                'att_rms', NaN, ...
                'status', "init", ...
                'last_update', 0, ...
                'warnings', {});

            % Initialize EKF state
            obj.initialize_ekf();
            
            % Initialize sensor simulation
            obj.initialize_sensors();
            
            % Initialize history buffers
            obj.initialize_history_buffers();
            
            % Initialize autonomous flight controller
            obj.flight_controller = autonomous_flight_controller(params);
            
            % Performance monitoring
            obj.estimation_errors = [];
            obj.innovation_stats = struct();
            obj.filter_health = struct('overall_score', 1.0, 'warnings', {});
            
            fprintf('RL-EKF Integration initialized\n');
        end
        
        function initialize_ekf(obj)
            %% Initialize EKF with Default State
            
            % Start with reasonable initial state
            obj.ekf_state = zeros(9, 1);  % [pos; vel; att]
            
            % Initial covariance (moderate uncertainty)
            obj.ekf_covariance = diag([
                1.0^2, 1.0^2, 1.0^2,        % Position uncertainty (m^2)
                0.5^2, 0.5^2, 0.5^2,        % Velocity uncertainty (m/s)^2
                deg2rad(5)^2, deg2rad(5)^2, deg2rad(10)^2  % Attitude uncertainty (rad^2)
            ]);
        end
        
        function initialize_sensors(obj)
            %% Initialize Sensor Simulation States
            
            obj.sensor_states = struct();
            
            % IMU biases (persistent)
            obj.sensor_states.accel_bias = obj.params.IMU.accel_bias_instab * randn(3,1);
            obj.sensor_states.gyro_bias = obj.params.IMU.gyro_bias_instab * randn(3,1);
            
            % GPS availability
            obj.gps_available = true;
            obj.gps_outage_timer = 0;
            
            % Magnetometer interference
            obj.mag_interference = false;
            
            % Previous measurements for finite differences
            obj.sensor_states.prev_vel = zeros(3,1);
            obj.sensor_states.prev_att = zeros(3,1);
            obj.sensor_states.prev_time = 0;
        end
        
        function initialize_history_buffers(obj)
            %% Initialize History Buffers for RL Observations
            
            buffer_length = 50;  % Keep last 50 updates
            
            obj.state_history = zeros(9, buffer_length);
            obj.uncertainty_history = zeros(9, buffer_length);
            obj.measurement_history = struct();
            obj.measurement_history.imu = zeros(6, buffer_length);
            obj.measurement_history.gps = NaN(3, buffer_length);
            obj.measurement_history.baro = NaN(1, buffer_length);
            obj.measurement_history.mag = NaN(1, buffer_length);
        end
        
        function [state_estimate, uncertainty_info] = step(obj, true_state, rl_velocity_command, dt)
            %% Main Integration Step
            
            obj.current_time = obj.current_time + dt;
            
            % 1) Apply RL velocity command to generate control inputs
            control_inputs = obj.convert_rl_command_to_control(rl_velocity_command, true_state);
            
            % 2) Simulate true dynamics with control inputs
            updated_true_state = obj.simulate_dynamics(true_state, control_inputs, dt);
            
            % 3) Generate sensor measurements from true state
            measurements = obj.generate_realistic_measurements(updated_true_state);
            
            % 4) Update EKF with measurements
            [state_estimate, uncertainty_info] = obj.update_ekf_with_measurements(measurements, dt);
            
            % 5) Update history buffers
            obj.update_history_buffers(state_estimate, uncertainty_info, measurements);
            
            % 6) Monitor filter performance
            obj.monitor_filter_performance(true_state, state_estimate, measurements);
            
            obj.last_update_time = obj.current_time;
        end
        
        function control_inputs = convert_rl_command_to_control(obj, rl_velocity_command, current_state)
            %% Convert RL Velocity Commands to Drone Control Inputs
            % RL outputs: [v_x, v_y, v_z, yaw_rate] normalized in [-1,1]
            % Converts to proper thrust and torque commands for 9-DOF drone dynamics
            
            % Use autonomous flight controller for better control
            control_inputs = obj.convert_rl_command_with_autonomous_controller(rl_velocity_command, current_state);
        end
        
        function control_inputs = convert_rl_command_with_autonomous_controller(obj, rl_velocity_command, current_state)
            %% Convert RL Commands Using Autonomous Flight Controller
            % Uses the professional autonomous flight controller for better control performance
            
            % Scale RL actions to actual velocity commands
            max_vel = obj.params.rl.max_velocity;        % e.g., 5.0 m/s
            max_yaw_rate = obj.params.rl.max_yaw_rate;   % e.g., 45 deg/s
            
            % Convert normalized RL commands to physical commands
            vel_cmd_ned = [
                rl_velocity_command(1) * max_vel;
                rl_velocity_command(2) * max_vel;
                rl_velocity_command(3) * max_vel
            ];
            yaw_rate_cmd = rl_velocity_command(4) * max_yaw_rate;
            
            % Create target state for autonomous controller
            current_pos = current_state(1:3);
            current_vel = current_state(4:6);
            current_att = current_state(7:9);
            
            % Target state: maintain current position but with desired velocity
            target_pos = current_pos;  % Hold position
            target_vel = vel_cmd_ned;  % Desired velocity from RL
            target_att = [0; 0; current_att(3) + yaw_rate_cmd * obj.integration_dt];  % Desired attitude
            
            target_state = [target_pos; target_vel; target_att];
            current_state_vec = current_state;
            
            % Use autonomous flight controller
            [control_inputs, control_info] = obj.flight_controller.compute_control(...
                current_state_vec, target_state, obj.integration_dt);
            
            % Store control info for analysis
            obj.last_control_info = control_info;
            
            % Ensure control inputs are finite
            if any(~isfinite(control_inputs))
                control_inputs = [obj.params.mass * abs(obj.params.g(3)); 0; 0; 0];
            end
        end
        
        function control_inputs = convert_rl_command_to_control_legacy(obj, rl_velocity_command, current_state)
            %% Legacy RL Command Conversion (for comparison)
            
            % Extract current state
            current_vel = current_state(4:6);  % NED velocity
            current_att = current_state(7:9);  % [roll, pitch, yaw]
            
            % Scale RL actions to actual velocity commands
            max_vel = obj.params.rl.max_velocity;        % e.g., 5.0 m/s
            max_yaw_rate = obj.params.rl.max_yaw_rate;   % e.g., 45 deg/s
            
            % Convert normalized RL commands to physical commands
            vel_cmd_ned = [
                rl_velocity_command(1) * max_vel;
                rl_velocity_command(2) * max_vel;
                rl_velocity_command(3) * max_vel
            ];
            yaw_rate_cmd = rl_velocity_command(4) * max_yaw_rate;
            
            % Enhanced velocity controller with feedforward and feedback
            Kp_vel = [1.2; 1.2; 1.5];  % Proportional gains
            Kd_vel = [0.3; 0.3; 0.4];  % Derivative gains (damping)
            
            % Store previous velocity error for derivative term
            persistent prev_vel_error
            if isempty(prev_vel_error)
                prev_vel_error = zeros(3,1);
            end
            
            vel_error = vel_cmd_ned - current_vel;
            vel_error_dot = (vel_error - prev_vel_error) / obj.integration_dt;
            prev_vel_error = vel_error;
            
            % PD velocity control
            accel_cmd = Kp_vel .* vel_error + Kd_vel .* vel_error_dot;
            
            % Add gravity compensation (NED frame)
            accel_cmd(3) = accel_cmd(3) + abs(obj.params.g(3));
            
            % Convert to thrust magnitude and direction
            thrust_magnitude = obj.params.mass * norm(accel_cmd);
            
            % Thrust limits based on physical constraints
            max_thrust = 2.5 * obj.params.mass * abs(obj.params.g(3));  % 2.5g max
            min_thrust = 0.3 * obj.params.mass * abs(obj.params.g(3));  % 0.3g min
            thrust_magnitude = max(min(thrust_magnitude, max_thrust), min_thrust);
            
            % Desired thrust direction (unit vector)
            if norm(accel_cmd) > 1e-6
                z_body_des = accel_cmd / norm(accel_cmd);
            else
                z_body_des = [0; 0; 1];  % Default pointing down
            end
            
            % Compute desired roll and pitch from thrust direction
            % Limit maximum tilt angle for stability
            max_tilt = deg2rad(25);  % 25 degree max tilt
            
            roll_des = asin(max(min(-z_body_des(2), sin(max_tilt)), -sin(max_tilt)));
            
            if abs(cos(roll_des)) > 1e-6
                pitch_des = asin(max(min(z_body_des(1) / cos(roll_des), sin(max_tilt)), -sin(max_tilt)));
            else
                pitch_des = 0;
            end
            
            % Yaw control - integrate yaw rate command
            yaw_des = current_att(3) + yaw_rate_cmd * obj.integration_dt;
            yaw_des = wrapToPi(yaw_des);
            
            % Attitude PD controller
            att_des = [roll_des; pitch_des; yaw_des];
            att_error = att_des - current_att;
            att_error(3) = atan2(sin(att_error(3)), cos(att_error(3)));  % Wrap yaw error
            
            % Store previous attitude error for derivative term
            persistent prev_att_error
            if isempty(prev_att_error)
                prev_att_error = zeros(3,1);
            end
            
            att_error_dot = (att_error - prev_att_error) / obj.integration_dt;
            prev_att_error = att_error;
            
            % PD attitude gains (tuned for stability)
            Kp_att = [0.8; 0.8; 0.5];  % Proportional gains
            Kd_att = [0.2; 0.2; 0.1];  % Derivative gains
            
            % Compute desired angular accelerations
            angular_accel_des = Kp_att .* att_error + Kd_att .* att_error_dot;
            
            % Convert to torque commands using inertia matrix
            torque_cmd = obj.params.I * angular_accel_des;
            
            % Torque limits for safety and realism
            max_torque = 0.15;  % Nm - Conservative limit
            torque_cmd = max(min(torque_cmd, max_torque), -max_torque);
            
            % Assemble final control vector [thrust; torque_x; torque_y; torque_z]
            control_inputs = [thrust_magnitude; torque_cmd];
            
            % Validation check
            if any(~isfinite(control_inputs))
                warning('Invalid control inputs generated, using safe defaults');
                control_inputs = [obj.params.mass * abs(obj.params.g(3)); 0; 0; 0];
            end
        end
        
        function updated_state = simulate_dynamics(obj, current_state, control_inputs, dt)
            %% Simulate True Drone Dynamics
            
            % Use existing drone dynamics
            state_dot = drone_dynamics(obj.current_time, current_state, control_inputs, obj.params);
            updated_state = current_state + state_dot * dt;
            
            % Wrap attitudes
            updated_state(7:9) = wrapToPi(updated_state(7:9));
            
            % Add some process noise for realism
            if obj.params.rl.ekf_integration.inject_ekf_noise
                process_noise = sqrt(dt) * [
                    0.01 * randn(3,1);  % Position noise
                    0.05 * randn(3,1);  % Velocity noise
                    deg2rad(0.5) * randn(3,1)  % Attitude noise
                ];
                updated_state = updated_state + process_noise;
                updated_state(7:9) = wrapToPi(updated_state(7:9));
            end
        end
        
        function measurements = generate_realistic_measurements(obj, true_state)
            %% Generate Realistic Sensor Measurements with Failures
            
            measurements = struct();
            
            % 1) IMU measurements (always available)
            measurements.imu = obj.generate_imu_measurements(true_state);
            
            % 2) GPS measurements (with outage simulation)
            measurements.gps = obj.generate_gps_measurements(true_state);
            
            % 3) Barometer measurements
            measurements.baro = obj.generate_baro_measurements(true_state);
            
            % 4) Magnetometer measurements (with interference)
            measurements.mag = obj.generate_mag_measurements(true_state);
            
            % 5) Update sensor availability flags
            obj.update_sensor_availability();
        end
        
        function imu_meas = generate_imu_measurements(obj, true_state)
            %% Generate IMU Measurements
            
            % Use existing sensor model but with enhanced realism
            dt_sensor = obj.integration_dt;
            
            % Get true acceleration via finite differences
            if obj.sensor_states.prev_time > 0
                true_accel = (true_state(4:6) - obj.sensor_states.prev_vel) / dt_sensor;
            else
                true_accel = zeros(3,1);
            end
            
            % Get true angular rates via finite differences
            if obj.sensor_states.prev_time > 0
                att_diff = true_state(7:9) - obj.sensor_states.prev_att;
                att_diff(3) = atan2(sin(att_diff(3)), cos(att_diff(3)));  % Wrap yaw difference
                true_ang_rates = att_diff / dt_sensor;
            else
                true_ang_rates = zeros(3,1);
            end
            
            % Convert acceleration to body frame
            R = rotation_matrix(true_state(7), true_state(8), true_state(9));
            accel_body = R' * (true_accel - obj.params.g);
            
            % Add IMU noise and biases
            accel_noise = obj.params.IMU.accel_noise_density * randn(3,1) / sqrt(dt_sensor);
            gyro_noise = obj.params.IMU.gyro_noise_density * randn(3,1) / sqrt(dt_sensor);
            
            % Random walk of biases
            bias_noise_scale = sqrt(dt_sensor);
            obj.sensor_states.accel_bias = obj.sensor_states.accel_bias + ...
                0.001 * bias_noise_scale * randn(3,1);
            obj.sensor_states.gyro_bias = obj.sensor_states.gyro_bias + ...
                0.0001 * bias_noise_scale * randn(3,1);
            
            % Final IMU measurements
            accel_meas = accel_body + obj.sensor_states.accel_bias + accel_noise;
            gyro_meas = true_ang_rates + obj.sensor_states.gyro_bias + gyro_noise;
            
            imu_meas = [accel_meas; gyro_meas];
            
            % Update previous values
            obj.sensor_states.prev_vel = true_state(4:6);
            obj.sensor_states.prev_att = true_state(7:9);
            obj.sensor_states.prev_time = obj.current_time;
        end
        
        function gps_meas = generate_gps_measurements(obj, true_state)
            %% Generate GPS Measurements with Outage Simulation
            
            if ~obj.gps_available
                gps_meas = [];
                return;
            end
            
            % GPS position measurements
            gps_noise = [
                obj.params.GPS.sigma_xy * randn();
                obj.params.GPS.sigma_xy * randn();
                obj.params.GPS.sigma_z * randn()
            ];
            
            gps_meas = true_state(1:3) + gps_noise;
        end
        
        function baro_meas = generate_baro_measurements(obj, true_state)
            %% Generate Barometer Measurements
            
            % Barometer measures altitude = -z in NED
            baro_noise = obj.params.Baro.sigma_z * randn();
            baro_meas = -true_state(3) + baro_noise;
        end
        
        function mag_meas = generate_mag_measurements(obj, true_state)
            %% Generate Magnetometer Measurements
            
            if obj.mag_interference
                mag_meas = [];
                return;
            end
            
            % Magnetometer measures yaw angle
            mag_noise = obj.params.Mag.sigma_rad * randn();
            mag_meas = true_state(9) + mag_noise;
        end
        
        function update_sensor_availability(obj)
            %% Update Sensor Availability Based on Failure Models
            
            % GPS outage simulation
            if obj.gps_available && rand() < obj.params.rl.ekf_integration.gps_outage_prob
                obj.gps_available = false;
                obj.gps_outage_timer = 5.0 + 10.0 * rand();  % 5-15 second outage
                fprintf('GPS outage started (duration: %.1f s)\n', obj.gps_outage_timer);
            elseif ~obj.gps_available
                obj.gps_outage_timer = obj.gps_outage_timer - obj.integration_dt;
                if obj.gps_outage_timer <= 0
                    obj.gps_available = true;
                    fprintf('GPS restored\n');
                end
            end
            
            % Magnetometer interference simulation
            if ~obj.mag_interference && rand() < obj.params.rl.ekf_integration.mag_interference_prob
                obj.mag_interference = true;
                fprintf('Magnetometer interference detected\n');
            elseif obj.mag_interference && rand() < 0.1  % 10% chance to clear each step
                obj.mag_interference = false;
                fprintf('Magnetometer interference cleared\n');
            end
        end
        
        function [state_estimate, uncertainty_info] = update_ekf_with_measurements(obj, measurements, dt)
            %% Update EKF with Sensor Measurements
            
            % Ensure correct dimensions (safety check)
            if any(size(obj.ekf_state) ~= [9, 1]) || any(size(obj.ekf_covariance) ~= [9, 9])
                % Reset to correct dimensions if corrupted
                obj.ekf_state = zeros(9, 1);
                obj.ekf_covariance = diag([1, 1, 1, 0.5, 0.5, 0.5, 0.1, 0.1, 0.1]);
            end
            
            if any(size(measurements.imu) ~= [6, 1])
                measurements.imu = zeros(6, 1);
            end
            
            % Prediction step (using IMU)
            [obj.ekf_state, obj.ekf_covariance] = ekf_sensor_only(...
                obj.ekf_state, obj.ekf_covariance, measurements.imu, [], obj.params, dt, 'IMU');
            
            % Update steps for each available sensor
            if ~isempty(measurements.gps)
                [obj.ekf_state, obj.ekf_covariance] = ekf_sensor_only(...
                    obj.ekf_state, obj.ekf_covariance, measurements.imu, measurements.gps, obj.params, 0, 'GPS');
            end
            
            if ~isempty(measurements.baro)
                [obj.ekf_state, obj.ekf_covariance] = ekf_sensor_only(...
                    obj.ekf_state, obj.ekf_covariance, measurements.imu, measurements.baro, obj.params, 0, 'Baro');
            end
            
            if ~isempty(measurements.mag)
                [obj.ekf_state, obj.ekf_covariance] = ekf_sensor_only(...
                    obj.ekf_state, obj.ekf_covariance, measurements.imu, measurements.mag, obj.params, 0, 'Mag');
            end
            
            % Return state estimate and uncertainty information
            state_estimate = obj.ekf_state;
            
            uncertainty_info = struct();
            uncertainty_info.covariance = obj.ekf_covariance;
            uncertainty_info.position_std = sqrt(diag(obj.ekf_covariance(1:3, 1:3)));
            uncertainty_info.velocity_std = sqrt(diag(obj.ekf_covariance(4:6, 4:6)));
            uncertainty_info.attitude_std = sqrt(diag(obj.ekf_covariance(7:9, 7:9)));
            uncertainty_info.total_uncertainty = trace(obj.ekf_covariance);
            uncertainty_info.gps_available = obj.gps_available;
            uncertainty_info.mag_available = ~obj.mag_interference;
        end
        
        function update_history_buffers(obj, state_estimate, uncertainty_info, measurements)
            %% Update History Buffers for RL Observations
            
            % Shift existing data
            obj.state_history = circshift(obj.state_history, [0, 1]);
            obj.uncertainty_history = circshift(obj.uncertainty_history, [0, 1]);
            obj.measurement_history.imu = circshift(obj.measurement_history.imu, [0, 1]);
            obj.measurement_history.gps = circshift(obj.measurement_history.gps, [0, 1]);
            obj.measurement_history.baro = circshift(obj.measurement_history.baro, [0, 1]);
            obj.measurement_history.mag = circshift(obj.measurement_history.mag, [0, 1]);
            
            % Add new data
            obj.state_history(:, 1) = state_estimate;
            obj.uncertainty_history(:, 1) = sqrt(diag(uncertainty_info.covariance));
            obj.measurement_history.imu(:, 1) = measurements.imu;
            
            if ~isempty(measurements.gps)
                obj.measurement_history.gps(:, 1) = measurements.gps;
            else
                obj.measurement_history.gps(:, 1) = NaN;
            end
            
            if ~isempty(measurements.baro)
                obj.measurement_history.baro(1) = measurements.baro;
            else
                obj.measurement_history.baro(1) = NaN;
            end
            
            if ~isempty(measurements.mag)
                obj.measurement_history.mag(1) = measurements.mag;
            else
                obj.measurement_history.mag(1) = NaN;
            end
        end
        
        function monitor_filter_performance(obj, true_state, estimated_state, measurements)
            %% Monitor EKF Filter Performance
            
            % Calculate estimation error
            error = true_state - estimated_state;
            error(7:9) = atan2(sin(error(7:9)), cos(error(7:9)));  % Wrap angle errors
            
            % Store error statistics
            obj.estimation_errors = [obj.estimation_errors, error];
            
            % Keep only recent errors (last 1000 samples)
            if size(obj.estimation_errors, 2) > 1000
                obj.estimation_errors = obj.estimation_errors(:, end-999:end);
            end
            
            % Calculate filter health metrics
            % Ensure filter_health struct is initialized before use (robust)
            if isempty(obj.filter_health) || ~isstruct(obj.filter_health)
                obj.filter_health = struct('overall_score', 1.0, 'pos_rms', NaN, 'att_rms', NaN, 'status', "init", 'last_update', 0, 'warnings', {});
            elseif ~isfield(obj.filter_health, 'overall_score') || ~isfield(obj.filter_health, 'warnings')
                obj.filter_health.overall_score = 1.0;
                if ~isfield(obj.filter_health, 'pos_rms'); obj.filter_health.pos_rms = NaN; end
                if ~isfield(obj.filter_health, 'att_rms'); obj.filter_health.att_rms = NaN; end
                if ~isfield(obj.filter_health, 'status'); obj.filter_health.status = "init"; end
                if ~isfield(obj.filter_health, 'last_update'); obj.filter_health.last_update = 0; end
                obj.filter_health.warnings = {};
            end
            if size(obj.estimation_errors, 2) >= 10
                recent_errors = obj.estimation_errors(:, end-9:end);
                
                % Position RMS error
                pos_rms = sqrt(mean(sum(recent_errors(1:3, :).^2, 1)));
                
                % Attitude RMS error (degrees)
                att_rms = rad2deg(sqrt(mean(sum(recent_errors(7:9, :).^2, 1))));
                
                % Build health record
                score = exp(-pos_rms/5) * exp(-att_rms/10);
                status = "ok";
                rec = struct('overall_score', score, 'pos_rms', pos_rms, 'att_rms', att_rms, ...
                             'status', status, 'last_update', obj.current_time, 'warnings', {{}});
                % Warnings
                if pos_rms > 5.0;  rec.warnings{end+1} = 'High position error'; end
                if att_rms > 15.0; rec.warnings{end+1} = 'High attitude error'; end
                if ~obj.gps_available; rec.warnings{end+1} = 'GPS unavailable'; end

                % Store as single latest snapshot (overwrite last) using indexed assignment
                if isempty(obj.filter_health)
                    obj.filter_health = rec;
                else
                    obj.filter_health(end) = rec;
                end
            end
        end
        
        function obs_data = get_rl_observation_data(obj)
            %% Get Observation Data for RL Agent
            
            obs_data = struct();
            
            % Current EKF state estimate
            obs_data.state_estimate = obj.ekf_state;
            
            % Uncertainty information
            obs_data.uncertainty = sqrt(diag(obj.ekf_covariance));
            
            % Recent state history
            obs_data.state_history = obj.state_history;
            
            % Sensor availability
            obs_data.gps_available = obj.gps_available;
            obs_data.mag_available = ~obj.mag_interference;
            
            % Filter health
            try
                if isstruct(obj.filter_health) && isfield(obj.filter_health, 'overall_score')
                    obs_data.filter_health = obj.filter_health.overall_score;
                else
                    obs_data.filter_health = 1.0;  % Default healthy score
                end
            catch
                obs_data.filter_health = 1.0;  % Default healthy score
            end
        end
        
        function reset(obj, initial_state)
            %% Reset Integration System
            
            if nargin > 1
                obj.ekf_state = initial_state;
            else
                obj.ekf_state = zeros(9, 1);
            end
            
            obj.initialize_ekf();
            obj.initialize_sensors();
            obj.initialize_history_buffers();
            
            obj.current_time = 0;
            obj.last_update_time = 0;
            obj.estimation_errors = [];
            % Reinitialize filter health to valid 1x1 struct
            obj.filter_health = struct( ...
                'overall_score', 1.0, ...
                'pos_rms', NaN, ...
                'att_rms', NaN, ...
                'status', "reset", ...
                'last_update', 0, ...
                'warnings', {});
        end
        
        function save_performance_data(obj, filename)
            %% Save Integration Performance Data
            
            perf_data = struct();
            perf_data.estimation_errors = obj.estimation_errors;
            perf_data.filter_health = obj.filter_health;
            perf_data.sensor_availability_log = struct();
            perf_data.sensor_availability_log.gps_available = obj.gps_available;
            perf_data.sensor_availability_log.mag_available = ~obj.mag_interference;
            
            save(filename, 'perf_data');
            fprintf('Integration performance data saved to: %s\n', filename);
        end
    end
end
