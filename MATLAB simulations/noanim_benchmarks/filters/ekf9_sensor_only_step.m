function [x_out, P_out, F_out, x_pred, P_pred] = ekf9_sensor_only_step(x_in, P_in, imu, params, dt)
%% EKF PREDICTION STEP - IMU-Driven State Estimation
% Implements the prediction phase of Extended Kalman Filter for 9-state drone
% State vector: x = [pos(3); vel(3); att(3)] = [x,y,z,vx,vy,vz,φ,θ,ψ]
%
% MATHEMATICAL OPERATIONS:
% 1) State Prediction: x̂(k|k-1) = f(x̂(k-1|k-1), u(k-1), w(k-1))
% 2) Jacobian Computation: F = ∂f/∂x (linearization around predicted state)
% 3) Covariance Prediction: P(k|k-1) = F·P(k-1|k-1)·F' + Q
% 4) Numerical Conditioning: Ensure P remains positive definite

%% STEP 1: NONLINEAR STATE PREDICTION
% Predict next state using IMU mechanization (strapdown integration)
% x̂(k|k-1) = x̂(k-1|k-1) + f(x̂, IMU) * dt
x_pred = x_in + drone_dynamics_imu(0, x_in, imu, params) * dt;
% Wrap attitude angles to [-π, π] range for numerical stability
x_pred(7:9) = wrapToPi(x_pred(7:9));

%% STEP 2: JACOBIAN MATRIX COMPUTATION
% Linearize nonlinear dynamics around predicted state for covariance propagation
% F = ∂f/∂x evaluated at x̂(k|k-1) - 9×9 matrix
F_out = calculate_F_sensor_only(x_pred, imu, dt);

%% STEP 3: PROCESS NOISE SCALING
% Scale process noise matrix by time step for discrete-time implementation
% Q_discrete = Q_continuous * dt
Q = params.Q * dt;

%% STEP 4: COVARIANCE PREDICTION
% Propagate uncertainty through nonlinear dynamics using Jacobian
% P(k|k-1) = F·P(k-1|k-1)·F' + Q
P_pred = F_out * P_in * F_out' + Q;

%% STEP 5: NUMERICAL CONDITIONING
% Ensure covariance matrix remains positive definite and well-conditioned
% SVD-based conditioning prevents numerical instability
[U, S, V] = svd(P_pred);
S = max(S, 1e-12);  % Prevent singular values from becoming too small
S = min(S, 1e6);    % Prevent singular values from becoming too large
P_pred = U * S * V';

%% OUTPUT ASSIGNMENT
% Return predicted state and covariance for measurement update phase
x_out = x_pred;
P_out = P_pred;
end


