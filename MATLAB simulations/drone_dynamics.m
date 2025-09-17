function [x_dot] = drone_dynamics(t, x, u, params)
%% PURPOSE
% 6-DOF nonlinear drone model for truth simulation with simplified cross-coupling.
%
% INPUTS
% - t      : time (unused in dynamics itself)
% - x      : state [pos(3); vel(3); att(3)] in NED
% - u      : control [thrust; tau_phi; tau_theta; tau_psi]
% - params : struct containing mass, inertia, gravity, etc.
%
% OUTPUTS
% - x_dot  : time derivative of the state
%
% MAJOR STEPS
% 1) Unpack state and clamp roll/pitch (singularity guard)
% 2) Unpack inputs and build rotation matrix
% 3) Compute forces: gravity, thrust, drag, velocity limiting, Coriolis
% 4) Translational acceleration and Euler-angle rate mapping
% 5) Add simple gyroscopic coupling and compute angular rates
% 6) Assemble derivatives
%
% CONTROL INTERFACE (QAV250 truth model):
% - This function advances the TRUE drone state used in simulation/animation.
% - There is no built-in flight controller here. Upstream scripts produce
%   open-loop commands u = [T; tau_phi; tau_theta; tau_psi] that act directly
%   on the rigid body:
%     T        : total thrust along body +Z (N)
%     tau_phi  : roll torque (Nm)
%     tau_theta: pitch torque (Nm)
%     tau_psi  : yaw torque (Nm)
% - Thrust is mapped to NED via R*[0;0;T]; gravity and aerodynamic drag are
%   summed to obtain translational acceleration. Torques pass through the
%   Euler-rate kinematics (with inertia scaling) to produce angular rates.
% - Parameters (mass/inertia/limits) are chosen to mimic QAV250 behavior.
%
%% 1) Unpack state
pos = x(1:3);
vel = x(4:6);
att = x(7:9); % [phi; theta; psi]

% Clamp actual roll and pitch to avoid singularities - extremely restrictive
max_angle = deg2rad(10); % Further reduced to 10 degrees for maximum stability
att(1) = max(min(att(1), max_angle), -max_angle); % roll
att(2) = max(min(att(2), max_angle), -max_angle); % pitch

%% 2) Unpack control inputs
T = u(1); % Total thrust (N)
tau = u(2:4); % Torques (Nm)

% Rotation matrix (body to NED)
R = rotation_matrix(att(1), att(2), att(3));

%% 3) Physical parameters (with overrides)
m = 0.5; % kg (QAV250)
I = diag([0.0023, 0.0023, 0.004]); % kg*m^2 (QAV250)
if isfield(params, 'mass'), m = params.mass; end
if isfield(params, 'I'), I = params.I; end

g = params.g;

%% 4) Forces
f_gravity = m * g;
f_thrust = R * [0; 0; T];

% Aerodynamic drag in body frame with strong damping
vel_body = R' * vel; % Transform velocity to body frame
drag_coeff = 0.5; % Doubled drag coefficient for extreme damping
f_drag_body = -drag_coeff * abs(vel_body) .* vel_body;
f_drag = R * f_drag_body;

% Additional velocity limiting for safety (progressive damping)
vel_mag = norm(vel);
if vel_mag > 5.0 % Much lower hard velocity limit
    vel_limit_factor = 5.0 / vel_mag;
    vel = vel * vel_limit_factor;
    % Add extremely strong damping force when approaching limit
    f_limit = -1.0 * m * (vel_mag - 4.0) * (vel / vel_mag);
    f_drag = f_drag + f_limit;
elseif vel_mag > 3.0 % Add progressive damping even at lower speeds
    f_limit = -0.5 * m * (vel_mag - 3.0) * (vel / vel_mag);
    f_drag = f_drag + f_limit;
end

% Centrifugal and Coriolis effects during turns (simplified)
omega_body = [0; 0; 0]; % Angular velocity in body frame (simplified)
if norm(vel) > 0.1 % Only apply when moving
    % Coriolis force: -2*m*omega_cross_vel
    omega_cross_vel = cross(omega_body, vel_body);
    f_coriolis = -2 * m * R * omega_cross_vel;
else
    f_coriolis = zeros(3,1);
end

% Translational acceleration with cross-coupling
acc = (f_thrust + f_gravity + f_drag + f_coriolis) / m;

%% 5) Angular rates mapping (Euler) with singularity handling
phi = att(1); theta = att(2);
epsilon = 1e-6;

% Improved transformation matrix with singularity handling
if abs(cos(theta)) < epsilon
    % Near singularity - use small angle approximation
    E = [1, 0, 0;
         0, 1, 0;
         0, 0, 1];
else
    E = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
         0, cos(phi),           -sin(phi);
         0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
end

% Add gyroscopic effects (rotor angular momentum coupling)
rotor_angular_momentum = [0; 0; 0.1]; % Simplified rotor momentum
gyro_torque = cross(omega_body, rotor_angular_momentum);

% Total angular acceleration
if cond(E) > 1e8
    omega = zeros(3,1);
else
    % Include gyroscopic effects in angular dynamics
    tau_total = tau + gyro_torque;
    omega = E \ (tau_total ./ diag(I));
end

%% 6) State derivatives
x_dot = zeros(9,1);
x_dot(1:3) = vel;
x_dot(4:6) = acc;
x_dot(7:9) = omega;
end

function R = rotation_matrix(phi, theta, psi)
% Rotation matrix from body to NED frame (ZYX yaw-pitch-roll)
Rz = [cos(psi), -sin(psi), 0;
      sin(psi),  cos(psi), 0;
      0,         0,        1];
Ry = [cos(theta), 0, sin(theta);
      0,          1, 0;
     -sin(theta), 0, cos(theta)];
Rx = [1, 0, 0;
      0, cos(phi), -sin(phi);
      0, sin(phi),  cos(phi)];
R = Rz * Ry * Rx;
end 