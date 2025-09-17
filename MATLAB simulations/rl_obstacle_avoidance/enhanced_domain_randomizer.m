%% enhanced_domain_randomizer.m - Enhanced Domain Randomization System
% PURPOSE
% Implements comprehensive domain randomization for robust sim-to-real transfer
% as specified in the PDF requirements. Randomizes physics, sensors, and
% environment parameters during training.

classdef enhanced_domain_randomizer < handle
    properties
        params
        
        % Randomization parameters
        physics_randomization
        sensor_randomization
        environment_randomization
        
        % Current randomized values
        current_physics
        current_sensor_params
        current_environment
        
        % Randomization statistics
        randomization_log
    end
    
    methods
        function obj = enhanced_domain_randomizer(params)
            obj.params = params;
            
            % Initialize randomization parameters
            obj.setup_physics_randomization();
            obj.setup_sensor_randomization();
            obj.setup_environment_randomization();
            
            % Initialize current values
            obj.reset_to_defaults();
            
            % Initialize logging
            obj.randomization_log = struct();
            obj.randomization_log.episodes = [];
            
            fprintf('Enhanced domain randomizer initialized\n');
        end
        
        function setup_physics_randomization(obj)
            %% Setup Physics Parameter Randomization
            
            obj.physics_randomization = struct();
            
            if obj.params.rl.advanced.domain_randomization && isfield(obj.params.rl, 'domain_rand')
                dr = obj.params.rl.domain_rand;
                
                % Gravity randomization
                obj.physics_randomization.gravity_scale = dr.gravity_range;
                
                % Mass randomization
                obj.physics_randomization.mass_scale = dr.mass_range;
                
                % Inertia randomization (scaled with mass)
                obj.physics_randomization.inertia_scale = [0.8, 1.2];
                
                % Aerodynamic parameters
                obj.physics_randomization.drag_coeff_scale = [0.5, 1.5];
                
                % Motor/actuator delays
                obj.physics_randomization.actuator_delay_range = dr.actuator_delay_range;
                
                % Motor response time variation
                obj.physics_randomization.motor_time_constant_scale = [0.8, 1.2];
                
                % Thrust/torque scaling
                obj.physics_randomization.thrust_scale = [0.9, 1.1];
                obj.physics_randomization.torque_scale = [0.9, 1.1];
                
            else
                % Minimal randomization if disabled
                obj.physics_randomization.gravity_scale = [1.0, 1.0];
                obj.physics_randomization.mass_scale = [1.0, 1.0];
                obj.physics_randomization.inertia_scale = [1.0, 1.0];
                obj.physics_randomization.drag_coeff_scale = [1.0, 1.0];
                obj.physics_randomization.actuator_delay_range = [0, 0];
                obj.physics_randomization.motor_time_constant_scale = [1.0, 1.0];
                obj.physics_randomization.thrust_scale = [1.0, 1.0];
                obj.physics_randomization.torque_scale = [1.0, 1.0];
            end
        end
        
        function setup_sensor_randomization(obj)
            %% Setup Sensor Parameter Randomization
            
            obj.sensor_randomization = struct();
            
            if obj.params.rl.advanced.domain_randomization && isfield(obj.params.rl, 'domain_rand')
                dr = obj.params.rl.domain_rand;
                
                % Sensor noise scaling
                obj.sensor_randomization.noise_scale = dr.sensor_noise_scale;
                
                % IMU parameters
                obj.sensor_randomization.imu_accel_noise_scale = [0.5, 2.0];
                obj.sensor_randomization.imu_gyro_noise_scale = [0.5, 2.0];
                obj.sensor_randomization.imu_bias_drift_scale = [0.5, 3.0];
                
                % GPS parameters
                obj.sensor_randomization.gps_noise_scale = [0.5, 3.0];
                obj.sensor_randomization.gps_outage_prob = [0.0, 0.1];   % 0-10% outage probability
                obj.sensor_randomization.gps_outage_duration = [2.0, 20.0]; % 2-20 second outages
                
                % Barometer parameters
                obj.sensor_randomization.baro_noise_scale = [0.5, 2.0];
                obj.sensor_randomization.baro_bias_scale = [0.0, 2.0];
                
                % Magnetometer parameters
                obj.sensor_randomization.mag_noise_scale = [0.5, 3.0];
                obj.sensor_randomization.mag_interference_prob = [0.0, 0.05]; % 0-5% interference
                
                % Limited sensor parameters (lidar/depth)
                obj.sensor_randomization.range_noise_scale = [0.5, 2.0];
                obj.sensor_randomization.beam_dropout_prob = [0.0, 0.1];  % 0-10% beam dropout
                obj.sensor_randomization.quantization_noise = [8, 16];    % 8-16 bit quantization
                obj.sensor_randomization.sensor_latency = [0.0, 0.1];     % 0-100ms latency
                
            else
                % No randomization
                obj.sensor_randomization.noise_scale = [1.0, 1.0];
                obj.sensor_randomization.imu_accel_noise_scale = [1.0, 1.0];
                obj.sensor_randomization.imu_gyro_noise_scale = [1.0, 1.0];
                obj.sensor_randomization.imu_bias_drift_scale = [1.0, 1.0];
                obj.sensor_randomization.gps_noise_scale = [1.0, 1.0];
                obj.sensor_randomization.gps_outage_prob = [0.0, 0.0];
                obj.sensor_randomization.gps_outage_duration = [5.0, 5.0];
                obj.sensor_randomization.baro_noise_scale = [1.0, 1.0];
                obj.sensor_randomization.baro_bias_scale = [1.0, 1.0];
                obj.sensor_randomization.mag_noise_scale = [1.0, 1.0];
                obj.sensor_randomization.mag_interference_prob = [0.0, 0.0];
                obj.sensor_randomization.range_noise_scale = [1.0, 1.0];
                obj.sensor_randomization.beam_dropout_prob = [0.0, 0.0];
                obj.sensor_randomization.quantization_noise = [12, 12];
                obj.sensor_randomization.sensor_latency = [0.0, 0.0];
            end
        end
        
        function setup_environment_randomization(obj)
            %% Setup Environment Parameter Randomization
            
            obj.environment_randomization = struct();
            
            if obj.params.rl.advanced.domain_randomization && isfield(obj.params.rl, 'domain_rand')
                dr = obj.params.rl.domain_rand;
                
                % Wind parameters
                obj.environment_randomization.wind_speed_max = dr.wind_speed_max;
                obj.environment_randomization.wind_direction_range = [0, 2*pi]; % Full 360°
                obj.environment_randomization.wind_variability = [0.5, 2.0];   % Wind fluctuation
                obj.environment_randomization.wind_gust_prob = [0.0, 0.1];     % Gust probability
                
                % Lighting/visual conditions (affects visual sensors)
                obj.environment_randomization.lighting_scale = [0.3, 1.5];
                obj.environment_randomization.contrast_scale = [0.5, 2.0];
                
                % Temperature effects (affects electronics)
                obj.environment_randomization.temperature_range = [-20, 50]; % Celsius
                
                % Map variations
                obj.environment_randomization.obstacle_density_scale = [0.7, 1.3];
                obj.environment_randomization.obstacle_size_scale = [0.8, 1.2];
                
            else
                % No environmental randomization
                obj.environment_randomization.wind_speed_max = 0.0;
                obj.environment_randomization.wind_direction_range = [0, 0];
                obj.environment_randomization.wind_variability = [1.0, 1.0];
                obj.environment_randomization.wind_gust_prob = [0.0, 0.0];
                obj.environment_randomization.lighting_scale = [1.0, 1.0];
                obj.environment_randomization.contrast_scale = [1.0, 1.0];
                obj.environment_randomization.temperature_range = [20, 20];
                obj.environment_randomization.obstacle_density_scale = [1.0, 1.0];
                obj.environment_randomization.obstacle_size_scale = [1.0, 1.0];
            end
        end
        
        function randomize_episode(obj)
            %% Randomize Parameters for New Episode
            
            % Randomize physics
            obj.randomize_physics_parameters();
            
            % Randomize sensors
            obj.randomize_sensor_parameters();
            
            % Randomize environment
            obj.randomize_environment_parameters();
            
            % Log randomization
            obj.log_current_randomization();
        end
        
        function randomize_physics_parameters(obj)
            %% Randomize Physics Parameters
            
            pr = obj.physics_randomization;
            
            % Gravity
            obj.current_physics.gravity_scale = obj.sample_uniform(pr.gravity_scale);
            
            % Mass and inertia
            obj.current_physics.mass_scale = obj.sample_uniform(pr.mass_scale);
            obj.current_physics.inertia_scale = obj.sample_uniform(pr.inertia_scale);
            
            % Aerodynamics
            obj.current_physics.drag_coeff_scale = obj.sample_uniform(pr.drag_coeff_scale);
            
            % Actuator parameters
            obj.current_physics.actuator_delay = obj.sample_uniform(pr.actuator_delay_range);
            obj.current_physics.motor_time_constant_scale = obj.sample_uniform(pr.motor_time_constant_scale);
            
            % Control effectiveness
            obj.current_physics.thrust_scale = obj.sample_uniform(pr.thrust_scale);
            obj.current_physics.torque_scale = obj.sample_uniform(pr.torque_scale);
        end
        
        function randomize_sensor_parameters(obj)
            %% Randomize Sensor Parameters
            
            sr = obj.sensor_randomization;
            
            % Overall noise scaling
            obj.current_sensor_params.noise_scale = obj.sample_uniform(sr.noise_scale);
            
            % IMU
            obj.current_sensor_params.imu_accel_noise_scale = obj.sample_uniform(sr.imu_accel_noise_scale);
            obj.current_sensor_params.imu_gyro_noise_scale = obj.sample_uniform(sr.imu_gyro_noise_scale);
            obj.current_sensor_params.imu_bias_drift_scale = obj.sample_uniform(sr.imu_bias_drift_scale);
            
            % GPS
            obj.current_sensor_params.gps_noise_scale = obj.sample_uniform(sr.gps_noise_scale);
            obj.current_sensor_params.gps_outage_prob = obj.sample_uniform(sr.gps_outage_prob);
            obj.current_sensor_params.gps_outage_duration = obj.sample_uniform(sr.gps_outage_duration);
            
            % Barometer
            obj.current_sensor_params.baro_noise_scale = obj.sample_uniform(sr.baro_noise_scale);
            obj.current_sensor_params.baro_bias_scale = obj.sample_uniform(sr.baro_bias_scale);
            
            % Magnetometer
            obj.current_sensor_params.mag_noise_scale = obj.sample_uniform(sr.mag_noise_scale);
            obj.current_sensor_params.mag_interference_prob = obj.sample_uniform(sr.mag_interference_prob);
            
            % Limited sensors
            obj.current_sensor_params.range_noise_scale = obj.sample_uniform(sr.range_noise_scale);
            obj.current_sensor_params.beam_dropout_prob = obj.sample_uniform(sr.beam_dropout_prob);
            obj.current_sensor_params.quantization_bits = round(obj.sample_uniform(sr.quantization_noise));
            obj.current_sensor_params.sensor_latency = obj.sample_uniform(sr.sensor_latency);
        end
        
        function randomize_environment_parameters(obj)
            %% Randomize Environment Parameters
            
            er = obj.environment_randomization;
            
            % Wind
            wind_speed = obj.sample_uniform([0, er.wind_speed_max]);
            wind_direction = obj.sample_uniform(er.wind_direction_range);
            obj.current_environment.wind_vector = wind_speed * [cos(wind_direction); sin(wind_direction); 0];
            obj.current_environment.wind_variability = obj.sample_uniform(er.wind_variability);
            obj.current_environment.wind_gust_prob = obj.sample_uniform(er.wind_gust_prob);
            
            % Visual conditions
            obj.current_environment.lighting_scale = obj.sample_uniform(er.lighting_scale);
            obj.current_environment.contrast_scale = obj.sample_uniform(er.contrast_scale);
            
            % Temperature
            obj.current_environment.temperature = obj.sample_uniform(er.temperature_range);
            
            % Map variations
            obj.current_environment.obstacle_density_scale = obj.sample_uniform(er.obstacle_density_scale);
            obj.current_environment.obstacle_size_scale = obj.sample_uniform(er.obstacle_size_scale);
        end
        
        function apply_physics_randomization(obj, params)
            %% Apply Physics Randomization to Parameters
            
            if isempty(obj.current_physics)
                return;
            end
            
            cp = obj.current_physics;
            
            % Apply gravity scaling
            params.g = params.g * cp.gravity_scale;
            
            % Apply mass scaling
            params.mass = params.mass * cp.mass_scale;
            
            % Apply inertia scaling
            params.I = params.I * cp.inertia_scale;
            
            % Apply drag coefficient scaling
            if isfield(params, 'drag_coeff')
                params.drag_coeff = params.drag_coeff * cp.drag_coeff_scale;
            end
        end
        
        function apply_sensor_randomization(obj, sensor_params)
            %% Apply Sensor Randomization to Sensor Parameters
            
            if isempty(obj.current_sensor_params)
                return;
            end
            
            csp = obj.current_sensor_params;
            
            % Apply IMU noise scaling
            sensor_params.IMU.accel_noise_density = sensor_params.IMU.accel_noise_density * csp.imu_accel_noise_scale;
            sensor_params.IMU.gyro_noise_density = sensor_params.IMU.gyro_noise_density * csp.imu_gyro_noise_scale;
            sensor_params.IMU.accel_bias_instab = sensor_params.IMU.accel_bias_instab * csp.imu_bias_drift_scale;
            sensor_params.IMU.gyro_bias_instab = sensor_params.IMU.gyro_bias_instab * csp.imu_bias_drift_scale;
            
            % Apply GPS noise scaling
            sensor_params.GPS.sigma_xy = sensor_params.GPS.sigma_xy * csp.gps_noise_scale;
            sensor_params.GPS.sigma_z = sensor_params.GPS.sigma_z * csp.gps_noise_scale;
            
            % Apply barometer noise scaling
            sensor_params.Baro.sigma_z = sensor_params.Baro.sigma_z * csp.baro_noise_scale;
            
            % Apply magnetometer noise scaling
            sensor_params.Mag.sigma_rad = sensor_params.Mag.sigma_rad * csp.mag_noise_scale;
            
            % Update failure probabilities (these would be used by the EKF integration)
            if isfield(sensor_params, 'rl') && isfield(sensor_params.rl, 'ekf_integration')
                sensor_params.rl.ekf_integration.gps_outage_prob = csp.gps_outage_prob;
                sensor_params.rl.ekf_integration.mag_interference_prob = csp.mag_interference_prob;
            end
        end
        
        function apply_environment_randomization(obj, environment)
            %% Apply Environment Randomization to Environment
            
            if isempty(obj.current_environment)
                return;
            end
            
            ce = obj.current_environment;
            
            % Apply wind effects (this would need to be integrated into the dynamics)
            environment.wind_vector = ce.wind_vector;
            environment.wind_variability = ce.wind_variability;
            
            % Apply visual effects (for limited sensors)
            environment.lighting_scale = ce.lighting_scale;
            environment.contrast_scale = ce.contrast_scale;
            
            % Apply temperature effects (could affect sensor noise)
            environment.temperature = ce.temperature;
        end
        
        function wind_force = calculate_wind_force(obj, velocity, params)
            %% Calculate Wind Force Effects
            
            if isempty(obj.current_environment) || norm(obj.current_environment.wind_vector) < 0.1
                wind_force = [0; 0; 0];
                return;
            end
            
            % Relative wind velocity
            wind_relative = obj.current_environment.wind_vector - velocity;
            wind_speed = norm(wind_relative);
            
            if wind_speed < 0.1
                wind_force = [0; 0; 0];
                return;
            end
            
            % Simple drag model: F = 0.5 * ρ * Cd * A * v²
            air_density = 1.225; % kg/m³ at sea level
            drag_coefficient = 0.1; % Simplified
            reference_area = 0.1; % m² (simplified drone cross-section)
            
            % Wind force
            wind_direction = wind_relative / wind_speed;
            wind_force = 0.5 * air_density * drag_coefficient * reference_area * wind_speed^2 * wind_direction;
            
            % Add some variability
            if rand() < obj.current_environment.wind_gust_prob
                gust_scale = 1.0 + obj.current_environment.wind_variability * (2*rand() - 1);
                wind_force = wind_force * gust_scale;
            end
        end
        
        function log_current_randomization(obj)
            %% Log Current Randomization for Analysis
            
            log_entry = struct();
            log_entry.physics = obj.current_physics;
            log_entry.sensors = obj.current_sensor_params;
            log_entry.environment = obj.current_environment;
            log_entry.timestamp = datetime('now');
            
            obj.randomization_log.episodes = [obj.randomization_log.episodes, log_entry];
        end
        
        function reset_to_defaults(obj)
            %% Reset All Parameters to Default Values
            
            obj.current_physics = struct();
            obj.current_physics.gravity_scale = 1.0;
            obj.current_physics.mass_scale = 1.0;
            obj.current_physics.inertia_scale = 1.0;
            obj.current_physics.drag_coeff_scale = 1.0;
            obj.current_physics.actuator_delay = 0.0;
            obj.current_physics.motor_time_constant_scale = 1.0;
            obj.current_physics.thrust_scale = 1.0;
            obj.current_physics.torque_scale = 1.0;
            
            obj.current_sensor_params = struct();
            obj.current_sensor_params.noise_scale = 1.0;
            obj.current_sensor_params.imu_accel_noise_scale = 1.0;
            obj.current_sensor_params.imu_gyro_noise_scale = 1.0;
            obj.current_sensor_params.imu_bias_drift_scale = 1.0;
            obj.current_sensor_params.gps_noise_scale = 1.0;
            obj.current_sensor_params.gps_outage_prob = 0.02;
            obj.current_sensor_params.gps_outage_duration = 5.0;
            obj.current_sensor_params.baro_noise_scale = 1.0;
            obj.current_sensor_params.baro_bias_scale = 1.0;
            obj.current_sensor_params.mag_noise_scale = 1.0;
            obj.current_sensor_params.mag_interference_prob = 0.01;
            obj.current_sensor_params.range_noise_scale = 1.0;
            obj.current_sensor_params.beam_dropout_prob = 0.05;
            obj.current_sensor_params.quantization_bits = 12;
            obj.current_sensor_params.sensor_latency = 0.0;
            
            obj.current_environment = struct();
            obj.current_environment.wind_vector = [0; 0; 0];
            obj.current_environment.wind_variability = 1.0;
            obj.current_environment.wind_gust_prob = 0.0;
            obj.current_environment.lighting_scale = 1.0;
            obj.current_environment.contrast_scale = 1.0;
            obj.current_environment.temperature = 20.0;
            obj.current_environment.obstacle_density_scale = 1.0;
            obj.current_environment.obstacle_size_scale = 1.0;
        end
        
        function value = sample_uniform(obj, range)
            %% Sample Uniformly from Range
            
            if length(range) ~= 2
                error('Range must have exactly 2 elements [min, max]');
            end
            
            value = range(1) + (range(2) - range(1)) * rand();
        end
        
        function save_randomization_log(obj, filename)
            %% Save Randomization Log for Analysis
            
            randomization_data = struct();
            randomization_data.log = obj.randomization_log;
            randomization_data.parameters = obj.params.rl;
            
            save(filename, 'randomization_data');
            fprintf('Domain randomization log saved to: %s\n', filename);
        end
    end
end
