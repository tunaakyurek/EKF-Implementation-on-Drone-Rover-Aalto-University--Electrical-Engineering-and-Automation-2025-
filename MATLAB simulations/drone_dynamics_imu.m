function [x_dot] = drone_dynamics_imu(t, x, imu, params)
%% PURPOSE
% 6-DOF strapdown mechanization used for EKF prediction, driven only by IMU.
%
% INPUTS
% - t      : time (unused in mechanization)
% - x      : state [pos(3); vel(3); att(3)] in NED, attitude [roll pitch yaw] (rad)
% - imu    : [accel_meas(3); gyro_meas(3)] in body frame
% - params : struct with fields including gravity g and optional limits
%
% OUTPUTS
% - x_dot  : time derivative of state for integration
%
% MAJOR STEPS
% 1) Compute attitude rates from gyro with Euler kinematics and singularity guard
% 2) Rotate measured specific force to NED and add gravity to get vel_dot
% 3) pos_dot = vel, then assemble x_dot
%
% Frames: Body (b) and NED (n). Rotation R = R_bn maps body→NED.
%
% CONTROL/MECHANIZATION CONTEXT (EKF prediction):
% - This function is NOT a controller. It integrates the state using the
%   measured specific force and angular rate from the IMU (body frame).
% - In our simulations, the true drone state evolves via drone_dynamics with
%   thrust/torque commands. The IMU is synthesized from that truth and fed
%   here for prediction, keeping the EKF decoupled from the controller.
%
%% STEP 1: STATE AND MEASUREMENT EXTRACTION
% Unpack 9-state vector and IMU measurements for mechanization
pos = x(1:3);                    % Position in NED frame [x, y, z] (m)
vel = x(4:6);                    % Velocity in NED frame [vx, vy, vz] (m/s)
att = x(7:9);                    % Attitude Euler angles [φ, θ, ψ] (rad)

% Extract IMU measurements in body frame
accel_meas = imu(1:3);           % Accelerometer measurements (body frame, m/s²)
gyro_meas = imu(4:6);            % Gyroscope measurements (body frame, rad/s)

% Extract gravity vector for NED frame
g = params.g;

%% STEP 2: ATTITUDE KINEMATICS (EULER ANGLE RATES)
% Compute attitude rates from gyroscope measurements using Euler kinematics
% Mathematical: [φ̇, θ̇, ψ̇] = E(φ,θ) · ω_b
% where E is the transformation matrix from body rates to Euler angle rates

phi = att(1); theta = att(2); psi = att(3);

% Guard against gimbal lock singularity (cos(θ) ≈ 0)
% This prevents division by zero when pitch approaches ±90°
cos_theta = cos(theta);
if abs(cos_theta) < 1e-6
    cos_theta = 1e-6 * sign(cos_theta + (cos_theta==0));
end

% Trigonometric functions for Euler angle rate transformation
sin_phi = sin(phi); cos_phi = cos(phi);
tan_theta = sin(theta)/cos_theta;

% Euler angle rate transformation matrix E(φ,θ)
% Maps body angular rates to Euler angle rates
E = [1, sin_phi*tan_theta,  cos_phi*tan_theta;
     0, cos_phi,            -sin_phi;
     0, sin_phi/cos_theta,   cos_phi/cos_theta];

% Compute attitude rates from gyroscope measurements
att_dot = E * gyro_meas;

% Apply rate limiting for numerical stability and physical realism
% Prevents unrealistic angular rates that could cause numerical issues
if isfield(params, 'max_angular_rate')
    max_rate = params.max_angular_rate;
else
    max_rate = deg2rad(200);  % Default: 200 deg/s maximum
end
att_dot = max(min(att_dot, max_rate), -max_rate);

%% STEP 3: VELOCITY DYNAMICS (STRAPDOWN MECHANIZATION)
% Compute velocity rates using accelerometer measurements and gravity
% Mathematical: v̇ = R_bn · a_b + g_n
% where R_bn rotates body-frame accelerometer to NED frame, then add gravity

% Compute body-to-NED rotation matrix using current attitude
R = rotation_matrix(phi, theta, psi);

% Ensure gravity vector follows NED convention (positive down)
% In NED frame, gravity should point in +z direction (downward)
if isscalar(params.g)
    g_n = [0; 0; params.g];  % Scalar gravity magnitude
else
    gv = params.g(:);
    if numel(gv) == 3
        if gv(3) < 0
            gv = -gv; % Flip sign to ensure +g down (NED convention)
        end
        g_n = gv;
    else
        g_n = [0; 0; 9.81];  % Default gravity vector
    end
end

% Strapdown mechanization: rotate body accelerometer to NED and add gravity
% This gives the true acceleration in NED frame
vel_dot = R * accel_meas + g_n;

%% STEP 4: POSITION DYNAMICS (KINEMATIC INTEGRATION)
% Position derivative is simply the current velocity
% Mathematical: ṗ = v
pos_dot = vel;

%% STEP 5: STATE DERIVATIVE ASSEMBLY
% Combine all state derivatives into 9×1 vector for integration
% This represents the complete state dynamics: ẋ = f(x, IMU)
x_dot = zeros(9,1);
x_dot(1:3) = pos_dot;    % Position rates
x_dot(4:6) = vel_dot;    % Velocity rates  
x_dot(7:9) = att_dot;    % Attitude rates
end 