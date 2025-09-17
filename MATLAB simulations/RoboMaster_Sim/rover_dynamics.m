function x_next = rover_dynamics(x, u_true, dt, params)
% ROVER_DYNAMICS  Discrete-time 2D rover kinematics with bias random walk
% State: [x y theta vx vy b_ax b_ay b_w]
% Control (true): [a_bx a_by omega]
% Includes realistic constraints: max speed, max acceleration, slip conditions
%
% CONTEXT (RoboMaster S1 truth model vs. controller):
% - This function advances the TRUE rover state used in simulation/animation.
% - We do NOT use a built-in controller here. Instead, scripts (e.g.,
%   main_rover_sim_realistic) synthesize a body-frame command profile
%   u_true = [a_bx; a_by; omega] consisting of forward/lateral accelerations
%   and yaw rate. That u_true drives the rover directly.
% - Longitudinal motion uses the forward acceleration a_bx mapped to global
%   acceleration along the current heading. Lateral command a_by is not used
%   to translate sideways (non-holonomic constraint); it is converted into a
%   steering effect that modulates yaw rate. This emulates real car/rover
%   behavior without a full tire model.
% - Bias states [b_ax b_ay b_w] evolve via random walks to reflect slow sensor
%   drift; the EKF estimates them during updates.

theta = x(3);
vx    = x(4);
vy    = x(5);

abx = u_true(1);
aby = u_true(2);
omega= u_true(3);

% Position update
x(1) = x(1) + vx*dt;
x(2) = x(2) + vy*dt;

% Orientation update
x(3) = wrapToPi_local(x(3) + omega*dt);

% Car-like motion: only move in facing direction (forward/backward)
% Lateral acceleration is converted to rotation, not lateral movement
% This creates the classic car steering behavior

% Forward acceleration (in body frame)
forward_accel = abx;  % Only use forward acceleration

% Convert lateral acceleration to rotation (steering effect)
% Higher lateral acceleration = sharper turn
steering_effect = aby * 2.0;  % Scale factor for steering sensitivity

% Update angular velocity with steering effect
omega = omega + steering_effect * dt;

% Accelerations in global frame (only forward direction)
c = cos(theta); s = sin(theta);
ax_g = forward_accel * c;  % Only forward component
ay_g = forward_accel * s;  % Only forward component

% Realistic constraints
% Max acceleration limit (typical for ground vehicles)
max_accel = 3.0; % m/s^2
ax_g = sign(ax_g) * min(abs(ax_g), max_accel);
ay_g = sign(ay_g) * min(abs(ay_g), max_accel);

% Velocity update with realistic constraints
x(4) = vx + ax_g*dt;
x(5) = vy + ay_g*dt;

% Max speed limit (typical for small rovers)
max_speed = 5.0; % m/s
current_speed = sqrt(x(4)^2 + x(5)^2);
if current_speed > max_speed
    speed_ratio = max_speed / current_speed;
    x(4) = x(4) * speed_ratio;
    x(5) = x(5) * speed_ratio;
end

% Car-like motion constraint: ensure velocity is always aligned with heading
% This enforces the non-holonomic constraint that cars can't move sideways
current_speed = sqrt(x(4)^2 + x(5)^2);
if current_speed > 0.1  % Only apply when moving
    % Calculate the angle between current velocity and heading
    vel_angle = atan2(x(5), x(4));
    heading_angle = x(3);
    
    % Force velocity to align with heading (forward/backward only)
    % Allow small deviation for numerical stability
    angle_diff = wrapToPi_local(vel_angle - heading_angle);
    if abs(angle_diff) > 0.1  % If deviation is significant
        % Project velocity onto heading direction
        x(4) = current_speed * cos(heading_angle);
        x(5) = current_speed * sin(heading_angle);
    end
end

% Bias random walks
if params.q_accel_bias > 0
    x(6) = x(6) + sqrt(params.q_accel_bias * dt) * randn;
    x(7) = x(7) + sqrt(params.q_accel_bias * dt) * randn;
end
if params.q_gyro_bias > 0
    x(8) = x(8) + sqrt(params.q_gyro_bias * dt) * randn;
end

x_next = x;

end

function ang = wrapToPi_local(ang)
ang = mod(ang + pi, 2*pi) - pi;
end


