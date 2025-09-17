classdef autonomous_flight_controller < handle
    %% autonomous_flight_controller - Professional Flight Control Using MATLAB Toolboxes
    % PURPOSE
    % High-performance autonomous flight controller using MATLAB Aerospace,
    % Control System, and Navigation toolboxes for superior performance.
    
    properties
        % Control System Toolbox - PID Controllers
        pos_controller_x
        pos_controller_y  
        pos_controller_z
        vel_controller_x
        vel_controller_y
        vel_controller_z
        att_controller_roll
        att_controller_pitch
        att_controller_yaw
        
        % Navigation Toolbox - Waypoint Following
        waypoint_follower
        path_planner
        
        % Aerospace Toolbox - Flight Dynamics
        flight_dynamics
        atmospheric_model
        
        % Control parameters
        control_gains
        limits
        
        % State
        current_waypoint
        target_waypoint
        waypoint_tolerance
        
        % PID Controller State (for simulation)
        pos_integrator
        pos_previous_error
        vel_integrator
        vel_previous_error
        att_integrator
        att_previous_error
        
        % Safety and performance
        max_integral_windup
        control_smoothing_factor
        last_control_inputs
        sample_time
        max_thrust_slew
        max_torque_slew
    end
    
    methods
        function obj = autonomous_flight_controller(params)
            %% Initialize Professional Flight Controller
            
            obj.control_gains = struct();
            obj.limits = struct();
            obj.current_waypoint = 1;
            obj.target_waypoint = [];
            obj.waypoint_tolerance = 2.0;  % meters
            
            % Initialize PID controller state
            obj.pos_integrator = zeros(3, 1);
            obj.pos_previous_error = zeros(3, 1);
            obj.vel_integrator = zeros(3, 1);
            obj.vel_previous_error = zeros(3, 1);
            obj.att_integrator = zeros(3, 1);
            obj.att_previous_error = zeros(3, 1);
            
            % Safety parameters
            obj.max_integral_windup = 10.0;  % Maximum integral accumulation
            obj.control_smoothing_factor = 0.8;  % Smoothing factor for control inputs
            % Start from hover thrust to avoid long ramp-up on first steps
            obj.last_control_inputs = [params.mass * abs(params.g(3)); 0; 0; 0];
            obj.max_thrust_slew = 8.0;           % N/s (can be tuned per airframe)
            obj.max_torque_slew = [2; 2; 1];     % Nm/s per axis
            
            % Initialize control gains
            obj.setup_control_gains(params);
            
            % Initialize PID controllers using Control System Toolbox
            obj.sample_time = params.Ts.physics;
            obj.setup_pid_controllers();
            
            % Initialize waypoint follower using Navigation Toolbox
            obj.setup_waypoint_follower();
            
            % Initialize flight dynamics using Aerospace Toolbox
            obj.setup_flight_dynamics(params);
            
            fprintf('✓ Professional flight controller initialized\n');
            fprintf('  - Control System Toolbox: PID controllers\n');
            fprintf('  - UAV Toolbox: Waypoint following\n');
            fprintf('  - Aerospace Toolbox: Flight dynamics\n');
        end
        
        function setup_control_gains(obj, params)
            %% Setup Control Gains for Different Flight Phases
            
            % Position control gains (outer loop) - BALANCED
            obj.control_gains.pos = struct();
            obj.control_gains.pos.kp = [0.8, 0.8, 1.2];  % [x, y, z] - Balanced
            obj.control_gains.pos.ki = [0.02, 0.02, 0.05];  % Moderate integral gains
            obj.control_gains.pos.kd = [0.15, 0.15, 0.25];  % Moderate derivative gains
            
            % Velocity control gains (middle loop) - BALANCED
            obj.control_gains.vel = struct();
            obj.control_gains.vel.kp = [0.6, 0.6, 0.8];  % Balanced
            obj.control_gains.vel.ki = [0.01, 0.01, 0.02];  % Moderate integral gains
            obj.control_gains.vel.kd = [0.08, 0.08, 0.12];  % Moderate derivative gains
            
            % Attitude control gains (inner loop) - BALANCED
            obj.control_gains.att = struct();
            obj.control_gains.att.kp = [1.2, 1.2, 0.8];  % [roll, pitch, yaw] - Balanced
            obj.control_gains.att.ki = [0.03, 0.03, 0.01];  % Moderate integral gains
            obj.control_gains.att.kd = [0.2, 0.2, 0.1];  % Moderate derivative gains
            
            % Control limits - Balanced
            obj.limits.max_velocity = 2.5;      % m/s - Balanced
            obj.limits.max_acceleration = 2.5;  % m/s² - Slightly higher to track trajectory
            obj.limits.max_tilt_angle = deg2rad(25);  % allow more tilt for lateral control
            obj.limits.max_yaw_rate = deg2rad(30);    % 30 deg/s - Balanced
            obj.limits.max_thrust = 1.8 * params.mass * abs(params.g(3));  % 1.8g - Balanced
            obj.limits.min_thrust = 0.0;  % allow full cut when needed
        end
        
        function setup_pid_controllers(obj)
            %% Setup PID Controllers Using Control System Toolbox
            
            % Position controllers
            obj.pos_controller_x = pid(obj.control_gains.pos.kp(1), ...
                                      obj.control_gains.pos.ki(1), ...
                                      obj.control_gains.pos.kd(1));
            obj.pos_controller_y = pid(obj.control_gains.pos.kp(2), ...
                                      obj.control_gains.pos.ki(2), ...
                                      obj.control_gains.pos.kd(2));
            obj.pos_controller_z = pid(obj.control_gains.pos.kp(3), ...
                                      obj.control_gains.pos.ki(3), ...
                                      obj.control_gains.pos.kd(3));
            
            % Velocity controllers
            obj.vel_controller_x = pid(obj.control_gains.vel.kp(1), ...
                                      obj.control_gains.vel.ki(1), ...
                                      obj.control_gains.vel.kd(1));
            obj.vel_controller_y = pid(obj.control_gains.vel.kp(2), ...
                                      obj.control_gains.vel.ki(2), ...
                                      obj.control_gains.vel.kd(2));
            obj.vel_controller_z = pid(obj.control_gains.vel.kp(3), ...
                                      obj.control_gains.vel.ki(3), ...
                                      obj.control_gains.vel.kd(3));
            
            % Attitude controllers
            obj.att_controller_roll = pid(obj.control_gains.att.kp(1), ...
                                         obj.control_gains.att.ki(1), ...
                                         obj.control_gains.att.kd(1));
            obj.att_controller_pitch = pid(obj.control_gains.att.kp(2), ...
                                          obj.control_gains.att.ki(2), ...
                                          obj.control_gains.att.kd(2));
            obj.att_controller_yaw = pid(obj.control_gains.att.kp(3), ...
                                        obj.control_gains.att.ki(3), ...
                                        obj.control_gains.att.kd(3));
            
            % Set controller sample time
            sample_time = obj.sample_time;
            obj.pos_controller_x.Ts = sample_time;
            obj.pos_controller_y.Ts = sample_time;
            obj.pos_controller_z.Ts = sample_time;
            obj.vel_controller_x.Ts = sample_time;
            obj.vel_controller_y.Ts = sample_time;
            obj.vel_controller_z.Ts = sample_time;
            obj.att_controller_roll.Ts = sample_time;
            obj.att_controller_pitch.Ts = sample_time;
            obj.att_controller_yaw.Ts = sample_time;
        end
        
        function setup_waypoint_follower(obj)
            %% Setup Waypoint Follower Using UAV Toolbox
            
            % Create waypoint follower using UAV Toolbox
            obj.waypoint_follower = uavWaypointFollower('UAVType', 'multirotor', ...
                                                       'TransitionRadius', 5.0, ...
                                                       'MinLookaheadDistance', 2.0);
            
            % Create path planner for obstacle avoidance (simplified for now)
            obj.path_planner = [];  % Will be initialized when needed
        end
        
        function setup_flight_dynamics(obj, params)
            %% Setup Flight Dynamics Using Aerospace Toolbox
            
            % Create flight dynamics model
            obj.flight_dynamics = struct();
            obj.flight_dynamics.mass = params.mass;
            obj.flight_dynamics.inertia = params.I;
            obj.flight_dynamics.gravity = params.g;
            
            % Atmospheric model for realistic flight
            obj.atmospheric_model = atmoscoesa(0);  % Standard atmosphere at sea level
        end
        
        function [control_inputs, control_info] = compute_control(obj, current_state, target_state, dt)
            %% Compute Control Inputs (uses external velocity feed-forward)
            % States
            p  = current_state(1:3);
            v  = current_state(4:6);
            e  = current_state(7:9);          % [roll; pitch; yaw]

            p_ref  = target_state(1:3);
            v_ref  = target_state(4:6);       % <-- external (VFH-blended) velocity
            e_ref  = target_state(7:9);       % yaw reference in e_ref(3)

            % Info
            control_info = struct();
            control_info.position_error = p_ref - p;

            % 1) Velocity setpoint = external (VFH/trajectory) + small position PID correction
            %    This keeps the guidance vector authoritative, while ensuring convergence.
            pos_err = p_ref - p;
            v_corr  = [ ...
                obj.control_gains.pos.kp(1)*pos_err(1); ...
                obj.control_gains.pos.kp(2)*pos_err(2); ...
                obj.control_gains.pos.kp(3)*pos_err(3)  ...
            ];
            % (light damping on v error baked into next loop; keep v_corr modest)
            v_des = v_ref + 0.5 * v_corr;  % 0.5 = correction weight
            v_des = max(min(v_des, obj.limits.max_velocity), -obj.limits.max_velocity);
            control_info.velocity_command = v_des;

            % 2) Velocity control -> acceleration command (PID on (v_des - v))
            vel_err = v_des - v;
            accel_cmd = zeros(3,1);
            for i = 1:3
                obj.vel_integrator(i) = max(min(obj.vel_integrator(i) + vel_err(i)*dt, obj.max_integral_windup), -obj.max_integral_windup);
                dvel = (vel_err(i) - obj.vel_previous_error(i)) / max(dt,1e-6);
                accel_cmd(i) = ...
                    obj.control_gains.vel.kp(i) * vel_err(i) + ...
                    obj.control_gains.vel.ki(i) * obj.vel_integrator(i) + ...
                    obj.control_gains.vel.kd(i) * dvel;
                obj.vel_previous_error(i) = vel_err(i);
            end
            accel_cmd = max(min(accel_cmd, obj.limits.max_acceleration), -obj.limits.max_acceleration);
            control_info.acceleration_command = accel_cmd;

            % 3) Attitude + thrust mapping
            %    Prefer the yaw you supplied; if it's NaN, align with v_des.
            if all(isfinite(e_ref(3)))
                desired_yaw = e_ref(3);
            else
                if norm(v_des(1:2)) > 0.2
                    desired_yaw = atan2(v_des(2), v_des(1));
                else
                    desired_yaw = e(3);
                end
            end
            [att_cmd, thrust_cmd] = obj.compute_attitude_control(accel_cmd, desired_yaw, e);
            control_info.attitude_command = att_cmd;
            control_info.thrust_command   = thrust_cmd;

            % 4) Attitude tracking (PID on attitude error)
            att_err = att_cmd - e;
            att_err(3) = atan2(sin(att_err(3)), cos(att_err(3)));
            torque_cmd = zeros(3,1);
            for i = 1:3
                obj.att_integrator(i) = max(min(obj.att_integrator(i) + att_err(i)*dt, obj.max_integral_windup), -obj.max_integral_windup);
                datt = (att_err(i) - obj.att_previous_error(i)) / max(dt,1e-6);
                torque_cmd(i) = ...
                    obj.control_gains.att.kp(i) * att_err(i) + ...
                    obj.control_gains.att.ki(i) * obj.att_integrator(i) + ...
                    obj.control_gains.att.kd(i) * datt;
                obj.att_previous_error(i) = att_err(i);
            end
            torque_cmd = obj.limit_torque(torque_cmd);
            control_info.torque_command = torque_cmd;

            % 5) Slew limiting scaled by dt (so halving dt doesn't 5x throttle the response)
            if ~isfield(obj,'max_thrust_slew') || isempty(obj.last_control_inputs)
                obj.max_thrust_slew = 80.0;       % N/s (more reasonable default)
                obj.max_torque_slew = [20;20;10]; % Nm/s
                if isempty(obj.last_control_inputs)
                    obj.last_control_inputs = [obj.flight_dynamics.mass*abs(obj.flight_dynamics.gravity(3)); 0; 0; 0];
                end
            end
            baseTs = max(obj.sample_time, 1e-3);         % nominal tuning step (0.01)
            scale  = max(baseTs / max(dt,1e-6), 1.0);    % if dt<baseTs, allow proportionally more per-step delta
            slew_per_step = [obj.max_thrust_slew; obj.max_torque_slew] .* (dt * scale);

            u_raw   = [thrust_cmd; torque_cmd];
            delta   = max(min(u_raw - obj.last_control_inputs, slew_per_step), -slew_per_step);
            control_inputs = obj.last_control_inputs + delta;
            obj.last_control_inputs = control_inputs;

            control_info.control_inputs = control_inputs;
        end
        
        function [att_cmd, thrust_cmd] = compute_attitude_control(obj, accel_cmd, desired_yaw, current_att)
            %% Compute Attitude Commands from Acceleration Commands
            
            % Limit acceleration commands to prevent excessive tilting
            accel_cmd = max(min(accel_cmd, obj.limits.max_acceleration), -obj.limits.max_acceleration);
            
            % Gravity and mass
            g = obj.flight_dynamics.gravity;  % NED [0;0;+g]
            m = obj.flight_dynamics.mass;

            % Required world thrust to achieve accel_cmd: FT_w = m*(a_cmd - g)
            FT_w = m * (accel_cmd - g);
            FT_norm = max(norm(FT_w), 1e-3);

            % Thrust magnitude
            thrust_cmd = FT_norm;
            thrust_cmd = min(max(thrust_cmd, obj.limits.min_thrust), obj.limits.max_thrust);

            % Desired body z aligns opposite to thrust (thrust along -z_b)
            z_b_des = -FT_w / FT_norm;
            x_c = [cos(desired_yaw); sin(desired_yaw); 0];
            y_b_des = cross(z_b_des, x_c); y_b_des = y_b_des / max(norm(y_b_des),1e-6);
            x_b_des = cross(y_b_des, z_b_des);
            R_des = [x_b_des y_b_des z_b_des];

            % Convert to ZYX Euler -> [roll; pitch; yaw]
            eulZyx = rotm2eul(R_des,'ZYX'); % [yaw pitch roll]
            att_cmd = [eulZyx(3); eulZyx(2); eulZyx(1)];
            % Constrain tilt
            att_cmd(1:2) = max(min(att_cmd(1:2), obj.limits.max_tilt_angle), -obj.limits.max_tilt_angle);
        end
        
        function vel_cmd = limit_velocity(obj, vel_cmd)
            %% Limit Velocity Commands
            vel_cmd = max(min(vel_cmd, obj.limits.max_velocity), -obj.limits.max_velocity);
        end
        
        function accel_cmd = limit_acceleration(obj, accel_cmd)
            %% Limit Acceleration Commands
            accel_cmd = max(min(accel_cmd, obj.limits.max_acceleration), -obj.limits.max_acceleration);
        end
        
        function torque_cmd = limit_torque(obj, torque_cmd)
            %% Limit Torque Commands
            max_torque = 0.30;  % Nm - more authority for attitude control
            torque_cmd = max(min(torque_cmd, max_torque), -max_torque);
        end
        
        function reset_pid_controllers(obj)
            %% Reset PID Controller State
            obj.pos_integrator = zeros(3, 1);
            obj.pos_previous_error = zeros(3, 1);
            obj.vel_integrator = zeros(3, 1);
            obj.vel_previous_error = zeros(3, 1);
            obj.att_integrator = zeros(3, 1);
            obj.att_previous_error = zeros(3, 1);
            obj.last_control_inputs = zeros(4, 1);
        end
        
        function set_waypoints(obj, waypoints)
            %% Set Waypoints for Following
            obj.target_waypoint = waypoints;
            obj.current_waypoint = 1;
        end
        
        function target_state = lookahead_target(obj, current_pos, v_des, Ld)
            %% Look-ahead target along current segment with velocity feed-forward
            if isempty(obj.target_waypoint) || obj.current_waypoint > size(obj.target_waypoint,2)
                target_state = [current_pos; zeros(6,1)]; return;
            end
            wp = obj.target_waypoint;
            i = max(1, min(obj.current_waypoint, size(wp,2)-1));
            p0 = wp(:,i); p1 = wp(:,i+1);
            seg = p1 - p0; segL = norm(seg);
            if segL < 1e-6
                target_state = [p1; zeros(6,1)]; return;
            end
            s = dot(current_pos - p0, seg) / (segL^2);
            s = max(0,min(1,s));
            sL = s*segL + Ld;
            s2 = min(sL/segL, 1);
            p_des = p0 + s2*seg;
            v_dir = seg/segL;
            v_des_vec = v_des * v_dir;
            yaw_ff = atan2(v_des_vec(2), v_des_vec(1));
            target_state = [p_des; v_des_vec; 0; 0; yaw_ff];
        end
        
        function [target_state, waypoint_reached] = get_next_target(obj, current_pos)
            %% Get Next Target State Using Navigation Toolbox
            
            if isempty(obj.target_waypoint)
                target_state = [current_pos; zeros(6, 1)];
                waypoint_reached = false;
                return;
            end
            
            if obj.current_waypoint > size(obj.target_waypoint, 2)
                target_state = [current_pos; zeros(6, 1)];
                waypoint_reached = true;
                return;
            end
            
            % Get current target waypoint
            target_pos = obj.target_waypoint(:, obj.current_waypoint);
            
            % Check if waypoint reached
            distance = norm(current_pos - target_pos);
            if distance < obj.waypoint_tolerance
                obj.current_waypoint = obj.current_waypoint + 1;
                waypoint_reached = true;
            else
                waypoint_reached = false;
            end
            
            % Generate target state
            target_state = [target_pos; zeros(6, 1)];  % Zero velocity and attitude for now
        end
        
        function reset_controllers(obj)
            %% Reset All PID Controllers
            obj.pos_controller_x.reset();
            obj.pos_controller_y.reset();
            obj.pos_controller_z.reset();
            obj.vel_controller_x.reset();
            obj.vel_controller_y.reset();
            obj.vel_controller_z.reset();
            obj.att_controller_roll.reset();
            obj.att_controller_pitch.reset();
            obj.att_controller_yaw.reset();
        end
        
        function tune_controllers(obj, performance_data)
            %% Tune Controllers Based on Performance Data
            % This could use Control System Toolbox optimization functions
            
            % For now, implement basic gain scheduling
            if isfield(performance_data, 'position_error')
                pos_error_rms = rms(performance_data.position_error);
                if pos_error_rms > 1.0  % Increase gains if error is large
                    obj.control_gains.pos.kp = obj.control_gains.pos.kp * 1.1;
                elseif pos_error_rms < 0.5  % Decrease gains if error is small
                    obj.control_gains.pos.kp = obj.control_gains.pos.kp * 0.9;
                end
            end
        end
    end
end
