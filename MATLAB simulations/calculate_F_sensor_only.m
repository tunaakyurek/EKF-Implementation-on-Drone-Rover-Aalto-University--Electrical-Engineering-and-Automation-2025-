function F = calculate_F_sensor_only(x, imu, dt)
%% EKF JACOBIAN MATRIX COMPUTATION - 9-State Drone
% Computes the Jacobian matrix F = ∂f/∂x for the EKF prediction step
% State: x = [p(3); v(3); eul(3)] = [x,y,z,vx,vy,vz,φ,θ,ψ] (ZYX Euler angles)
% IMU:   imu = [a_b(3); omega_b(3)] in body frame
%
% MATHEMATICAL BACKGROUND:
% The Jacobian F captures how small changes in state affect the dynamics
% F = ∂f/∂x where f(x,u) is the nonlinear state transition function
% For 9-state drone: F is 9×9 matrix with specific structure

%% STEP 1: EXTRACT STATE AND IMU MEASUREMENTS
% Unpack current state estimate and IMU measurements
phi = x(7); theta = x(8); psi = x(9);  % Euler angles [φ, θ, ψ]
a_b = imu(1:3);                        % Accelerometer measurements (body frame)
omega_b = imu(4:6);                    % Gyroscope measurements (body frame)

%% STEP 2: COMPUTE ROTATION MATRIX AND ITS PARTIAL DERIVATIVES
% Body-to-NED rotation matrix R_bn using ZYX Euler sequence
% This matrix transforms vectors from body frame to NED frame

% Trigonometric functions for current attitude
cphi = cos(phi); sphi = sin(phi);
cth  = cos(theta); sth = sin(theta);
cps  = cos(psi); sps = sin(psi);

% ZYX rotation matrix: R = Rz(ψ) * Ry(θ) * Rx(φ)
R = [ cth*cps,            cps*sth*sphi - sps*cphi,  cps*sth*cphi + sps*sphi;...
      cth*sps,            sps*sth*sphi + cps*cphi,  sps*sth*cphi - cps*sphi;...
      -sth,               cth*sphi,                 cth*cphi ];

% Partial derivatives of rotation matrix with respect to Euler angles
% These capture how attitude changes affect the rotation matrix

% ∂R/∂φ (partial derivative with respect to roll)
dR_dphi = [ 0,                 cps*sth*cphi + sps*sphi,  -cps*sth*sphi + sps*cphi;...
            0,                 sps*sth*cphi - cps*sphi,  -sps*sth*sphi - cps*cphi;...
            0,                 cth*cphi,                 -cth*sphi ];

% ∂R/∂θ (partial derivative with respect to pitch)
dR_dth = [ -sth*cps,           cps*cth*sphi,             cps*cth*cphi;...
           -sth*sps,           sps*cth*sphi,             sps*cth*cphi;...
           -cth,               -sth*sphi,                -sth*cphi ];

% ∂R/∂ψ (partial derivative with respect to yaw)
dR_dpsi = [ -cth*sps,          -sps*sth*sphi - cps*cphi, -sps*sth*cphi + cps*sphi;...
             cth*cps,           cps*sth*sphi - sps*cphi,  cps*sth*cphi + sps*sphi;...
             0,                 0,                        0 ];

%% STEP 3: VELOCITY-ATTITUDE COUPLING MATRIX
% Compute how attitude changes affect velocity through rotation of accelerometer
% A = ∂(R·a_b)/∂[φ,θ,ψ] - 3×3 matrix showing velocity sensitivity to attitude
A = [dR_dphi*a_b, dR_dth*a_b, dR_dpsi*a_b];

%% STEP 4: EULER ANGLE KINEMATICS AND ITS PARTIAL DERIVATIVES
% Euler angle rates: [φ̇, θ̇, ψ̇] = T(φ,θ) · ω_b
% T is the transformation matrix from body angular rates to Euler angle rates

% Guard against gimbal lock (cos(θ) ≈ 0)
costh = cth;
if abs(costh) < 1e-6, costh = 1e-6*sign(costh + (costh==0)); end
tanth = sth/costh; sec2 = 1/(costh^2);

% Euler angle rate transformation matrix T(φ,θ)
T = [ 1,      sphi*tanth,          cphi*tanth;...
      0,      cphi,                -sphi;...
      0,      sphi/costh,          cphi/costh ];

% Partial derivatives of T matrix with respect to Euler angles
% These capture how attitude changes affect the Euler angle rate transformation

% ∂T/∂φ (partial derivative with respect to roll)
dT_dphi = [ 0,      cphi*tanth,        -sphi*tanth;...
            0,     -sphi,              -cphi;...
            0,      cphi/costh,        -sphi/costh ];

% ∂T/∂θ (partial derivative with respect to pitch)
dT_dth = [ 0,      sphi*sec2,          cphi*sec2;...
           0,      0,                  0;...
           0,      sphi*sth/(costh^2), cphi*sth/(costh^2) ];

%% STEP 5: ATTITUDE-ATTITUDE COUPLING MATRIX
% Compute how attitude changes affect attitude rates through Euler kinematics
% B = ∂(T·ω_b)/∂[φ,θ,ψ] - 3×3 matrix (note: no ψ dependence in T)
B = [dT_dphi*omega_b, dT_dth*omega_b, [0;0;0]];

%% STEP 6: ASSEMBLE JACOBIAN MATRIX F
% Construct the 9×9 Jacobian matrix with specific block structure
F = eye(9);

% Position-velocity coupling: ∂(p + v·dt)/∂v = dt·I₃
F(1:3,4:6)   = dt*eye(3);      % p_{k+1} = p_k + v_k·dt

% Velocity-attitude coupling: ∂(v + R·a_b·dt)/∂[φ,θ,ψ] = dt·A
F(4:6,7:9)   = dt*A;           % v_{k+1} sensitivity to attitude via R(φ,θ,ψ)·a_b

% Attitude-attitude coupling: ∂([φ,θ,ψ] + T·ω_b·dt)/∂[φ,θ,ψ] = I₃ + dt·B
F(7:9,7:9)   = eye(3) + dt*B;  % Euler angle kinematics via T(φ,θ)·ω_b

%% STEP 7: NUMERICAL REGULARIZATION
% Add small regularization to prevent numerical singularity
% This ensures the Jacobian remains invertible for covariance propagation
F = F + 1e-12*eye(9);
end