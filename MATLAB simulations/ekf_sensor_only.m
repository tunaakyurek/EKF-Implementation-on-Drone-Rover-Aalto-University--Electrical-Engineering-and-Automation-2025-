function [x_est, P] = ekf_sensor_only(x_est, P, imu, z, params, dt, sensor_type)
%% PURPOSE
% Extended Kalman Filter that uses IMU-only prediction and selected sensor updates.
% This version does NOT use control inputs; suitable when actuators are unknown.
%
% INPUTS
% - x_est      : prior state estimate [pos(3); vel(3); att(3)]
% - P          : prior covariance (9x9)
% - imu        : [accel_meas(3); gyro_meas(3)] in body frame
% - z          : measurement vector/scalar (depends on sensor_type)
% - params     : struct with Q, R matrices and other config
% - dt         : time step (s)
% - sensor_type: 'IMU' | 'GPS' | 'Baro' | 'Mag'
%
% OUTPUTS
% - x_est, P   : posterior state and covariance after predict/update
%
% MAJOR STEPS
% 1) Validate and clamp inputs to safe ranges
% 2) Predict state using IMU mechanization; build Jacobian F and process noise Q
% 3) Predict covariance and condition it
% 4) Sensor-specific update step (optional for 'IMU')
% 5) Final validation, clamping, and conditioning of outputs

%% STEP 1: INPUT VALIDATION AND STATE BOUNDING
% Ensure input state and covariance are finite and within reasonable bounds
% This prevents numerical instability and maintains physical realism

% Check for NaN/Inf values in state or covariance
if any(~isfinite(x_est)) || any(~isfinite(P(:)))
    warning('EKF: Invalid input state or covariance. Using safe defaults.');
    x_est = zeros(9,1);
    P = 0.1*eye(9);
end

% Define physical bounds for state variables
pos_lim = 1000;    % Position limit (m) - reasonable flight envelope
vel_lim = 50;      % Velocity limit (m/s) - typical drone speeds
roll_pitch_lim = pi/2; % roll/pitch limit (rad) - prevent gimbal lock

% Apply bounds to maintain physical realism and numerical stability
x_est(1:3) = max(min(x_est(1:3), pos_lim), -pos_lim);  % Position bounds
x_est(4:6) = max(min(x_est(4:6), vel_lim), -vel_lim);  % Velocity bounds
% Limit roll/pitch to prevent gimbal lock; wrap yaw to [-π, π]
x_est(7:8) = max(min(x_est(7:8), roll_pitch_lim), -roll_pitch_lim);
x_est(9) = wrapToPi(x_est(9));

%% STEP 2: NONLINEAR STATE PREDICTION
% Predict next state using IMU mechanization (strapdown integration)
% Mathematical: x̂(k|k-1) = x̂(k-1|k-1) + f(x̂, IMU) * dt
try
    x_pred = x_est + drone_dynamics_imu(0, x_est, imu, params) * dt;
catch ME
    warning('EKF: Error in IMU mechanization. Using simple integration.');
    % Fallback: simple kinematic integration if mechanization fails
    x_pred = x_est + [x_est(4:6); zeros(3,1); zeros(3,1)] * dt;
end

% Apply attitude bounds to prevent gimbal lock and maintain stability
x_pred(7:8) = max(min(x_pred(7:8), deg2rad(60)), -deg2rad(60));
x_pred(9) = wrapToPi(x_pred(9));

% Validate prediction for numerical stability
if any(~isfinite(x_pred))
    warning('EKF: Invalid prediction. Using previous state.');
    x_pred = x_est;
end

%% STEP 3: JACOBIAN MATRIX COMPUTATION
% Linearize nonlinear dynamics around predicted state for covariance propagation
% Mathematical: F = ∂f/∂x evaluated at x̂(k|k-1) - 9×9 matrix
F = calculate_F_sensor_only(x_pred, imu, dt);

%% STEP 4: PROCESS NOISE PREPARATION
% Scale continuous-time process noise for discrete-time implementation
% Mathematical: Q_discrete = Q_continuous * dt
Q = params.Q * dt;

% Ensure process noise matrix is positive definite via SVD conditioning
[U, S, V] = svd(Q);
S = max(S, 1e-12);  % Prevent singular values from becoming too small
Q = U * S * V';

%% STEP 5: COVARIANCE PREDICTION
% Propagate uncertainty through nonlinear dynamics using Jacobian
% Mathematical: P(k|k-1) = F·P(k-1|k-1)·F' + Q
P_pred = F*P*F' + Q;

% Apply numerical conditioning to maintain positive definiteness
[U, S, V] = svd(P_pred);
S = max(S, 1e-12); % Prevent singular values from becoming too small
S = min(S, 1e6);   % Prevent singular values from becoming too large
P_pred = U * S * V';

%% STEP 6: MEASUREMENT UPDATE (SENSOR-SPECIFIC)
% Apply Kalman filter measurement update based on sensor type
% Mathematical operations: y = z - H·x̂, S = H·P·H' + R, K = P·H'/S
%                         x̂(k|k) = x̂(k|k-1) + K·y, P(k|k) = (I - K·H)·P·(I - K·H)' + K·R·K'

switch sensor_type
    case 'IMU'
        %% IMU-ONLY PREDICTION (NO MEASUREMENT UPDATE)
        % IMU is used only for prediction; no direct measurement update
        % This maintains the prediction-only approach for sensor-only EKF
        x_est = x_pred;
        P = P_pred;
        
    case 'GPS'
        %% GPS POSITION MEASUREMENT UPDATE
        % GPS provides direct position measurements in NED frame
        % Measurement model: z_GPS = [x, y, z]' + noise
        
        % Measurement matrix: GPS measures position directly
        H = [eye(3), zeros(3,6)];  % H = [I₃ 0₃ₓ₆] - position measurement
        R = params.R_gps;          % GPS measurement noise covariance
        
        % Innovation (measurement residual)
        y = z - H*x_pred;          % y = z - H·x̂(k|k-1)
        
        % Innovation covariance
        S = H*P_pred*H' + R;       % S = H·P(k|k-1)·H' + R
        
        % Add regularization to prevent numerical singularity
        S = S + 1e-6 * eye(size(S));
        
        % Check innovation covariance conditioning
        if cond(S) > 1e12
            warning('EKF: GPS innovation covariance is ill-conditioned. Skipping update.');
            x_est = x_pred;
            P = P_pred;
        else
            % Kalman gain computation
            K = P_pred*H'/S;       % K = P(k|k-1)·H'·S⁻¹
            
            % State update
            x_est = x_pred + K*y;  % x̂(k|k) = x̂(k|k-1) + K·y
            
            % Covariance update (Joseph form for numerical stability)
            I9 = eye(9);
            P = (I9 - K*H)*P_pred*(I9 - K*H)' + K*R*K';
        end
    case 'Baro'
        %% BAROMETER ALTITUDE MEASUREMENT UPDATE
        % Barometer provides altitude measurement (positive up, NED z is negative)
        % Measurement model: z_baro = -z_NED + noise (altitude = -NED_z)
        
        % Measurement matrix: barometer measures altitude (-z component)
        H = [0 0 -1 zeros(1,6)];  % H = [0 0 -1 0 0 0 0 0 0] - altitude measurement
        R = params.R_baro;        % Barometer measurement noise variance
        
        % Innovation (measurement residual)
        y = z - H*x_pred;         % y = z_baro - (-z_NED_predicted)
        
        % Innovation covariance
        S = H*P_pred*H' + R;      % S = H·P(k|k-1)·H' + R (scalar for barometer)
        
        % Add regularization to prevent numerical singularity
        S = S + 1e-6;
        
        % Check innovation covariance conditioning
        if cond(S) > 1e12
            warning('EKF: Baro innovation covariance is ill-conditioned. Skipping update.');
            x_est = x_pred;
            P = P_pred;
        else
            % Kalman gain computation
            K = P_pred*H'/S;      % K = P(k|k-1)·H'·S⁻¹
            
            % State update
            x_est = x_pred + K*y; % x̂(k|k) = x̂(k|k-1) + K·y
            
            % Covariance update (Joseph form for numerical stability)
            I9 = eye(9);
            P = (I9 - K*H)*P_pred*(I9 - K*H)' + K*R*K';
        end
        
    case 'Mag'
        %% MAGNETOMETER YAW MEASUREMENT UPDATE
        % Magnetometer provides yaw angle measurement (heading from magnetic north)
        % Measurement model: z_mag = ψ + noise (yaw angle in radians)
        
        % Measurement matrix: magnetometer measures yaw angle (9th state element)
        H = [zeros(1,8), 1];      % H = [0 0 0 0 0 0 0 0 1] - yaw measurement
        R = params.R_mag;         % Magnetometer measurement noise variance
        
        % Innovation with angle wrapping to [-π, π]
        % Handle angle discontinuity by computing shortest angular distance
        y = atan2(sin(z - x_pred(9)), cos(z - x_pred(9)));
        
        % Innovation covariance
        S = H*P_pred*H' + R;      % S = H·P(k|k-1)·H' + R (scalar for magnetometer)
        
        % Add regularization to prevent numerical singularity
        S = S + 1e-6;
        
        % Check innovation covariance conditioning
        if cond(S) > 1e12
            warning('EKF: Mag innovation covariance is ill-conditioned. Skipping update.');
            x_est = x_pred;
            P = P_pred;
        else
            % Kalman gain computation
            K = P_pred*H'/S;      % K = P(k|k-1)·H'·S⁻¹
            
            % State update
            x_est = x_pred + K*y; % x̂(k|k) = x̂(k|k-1) + K·y
            
            % Covariance update (Joseph form for numerical stability)
            I9 = eye(9);
            P = (I9 - K*H)*P_pred*(I9 - K*H)' + K*R*K';
        end
    case 'AccelTilt'
        % Use accelerometer to correct roll/pitch assuming quasi-static or
        % centripetal-compensated specific force. Compute gravity direction
        % estimate from accel in NED and align roll/pitch accordingly.
        % Build a simple measurement model: z = [phi; theta] from accel.
        % 1) Rotate measured accel to NED using current yaw only to avoid
        %     coupling errors; treat gravity magnitude as known.
        g = params.g;
        g_norm = norm(g);
        if g_norm < 1e-6, g_norm = 9.81; end
        psi = x_pred(9);
        Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
        a_b = imu(1:3);
        a_n_partial = Rz * a_b; % approximate body->NED using yaw only
        % Compensate gravity sign: specific force f_b = R'*(a - g).
        % After partial rotation, estimate gravity direction:
        g_n_est = -a_n_partial; % sign so that at rest a_b ~ -R'*g
        % Convert gravity direction to roll/pitch
        gx = g_n_est(1); gy = g_n_est(2); gz = g_n_est(3);
        % Normalize
        s = sqrt(gx^2 + gy^2 + gz^2);
        if s > 1e-6
            gx = gx/s; gy = gy/s; gz = gz/s;
            % Roll and pitch from gravity vector
            phi_meas = atan2(gy, gz);
            theta_meas = -asin(max(min(gx,1),-1));
            z_tilt = [phi_meas; theta_meas];
            H = [zeros(2,6), eye(2), zeros(2,1)]; % measure [phi; theta]
            % Use tuned tilt noise if provided
            if isfield(params, 'Tilt') && isfield(params.Tilt, 'R_deg')
                rdeg = params.Tilt.R_deg;
            else
                rdeg = 3.0;
            end
            Rtilt = diag([deg2rad(rdeg)^2, deg2rad(rdeg)^2]);
            y = z_tilt - H*x_pred;
            % Wrap small-angle residuals for roll/pitch implicitly via bounds later
            S = H*P_pred*H' + Rtilt + 1e-9*eye(2);
            K = P_pred*H'/S;
            x_est = x_pred + K*y;
            I9 = eye(9);
            P = (I9 - K*H)*P_pred*(I9 - K*H)' + K*Rtilt*K';
        else
            x_est = x_pred; P = P_pred;
        end
    otherwise
        error('Unknown sensor type');
end

%% STEP 7: FINAL VALIDATION AND OUTPUT CONDITIONING
% Ensure final state and covariance are numerically stable and physically valid
% This step prevents filter divergence and maintains estimation quality

% Check for numerical instability in final results
if any(~isfinite(x_est)) || any(~isfinite(P(:)))
    warning('EKF: Final state or covariance contains NaN/Inf. Resetting to prediction.');
    x_est = x_pred;
    P = P_pred;
end

% Apply final state bounds to maintain physical realism
x_est(1:3) = max(min(x_est(1:3), pos_lim), -pos_lim);  % Position bounds
x_est(4:6) = max(min(x_est(4:6), vel_lim), -vel_lim);  % Velocity bounds
% Limit roll/pitch to prevent gimbal lock; wrap yaw to [-π, π]
x_est(7:8) = max(min(x_est(7:8), roll_pitch_lim), -roll_pitch_lim);
x_est(9) = wrapToPi(x_est(9));

% Ensure final covariance matrix is positive definite and well-conditioned
% SVD-based conditioning maintains numerical stability for next iteration
[U, S, V] = svd(P);
S = max(S, 1e-12);  % Prevent singular values from becoming too small
S = min(S, 1e6);    % Prevent singular values from becoming too large
P = U * S * V';
end 