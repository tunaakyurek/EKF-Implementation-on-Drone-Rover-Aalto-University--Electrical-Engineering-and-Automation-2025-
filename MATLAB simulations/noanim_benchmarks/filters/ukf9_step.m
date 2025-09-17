function [x, P] = ukf9_step(x, P, imu, z, params, dt, sensor_type)
%% PURPOSE
% Enhanced Unscented Kalman Filter for 9-state drone model with improved robustness
% Updated to match latest EKF features including adaptive noise, innovation gating,
% and enhanced error handling for better performance.
%
% INPUTS
% - x          : prior state estimate [pos(3); vel(3); att(3)]
% - P          : prior covariance (9x9)
% - imu        : [accel_meas(3); gyro_meas(3)] in body frame
% - z          : measurement vector/scalar (depends on sensor_type)
% - params     : struct with Q, R matrices, adaptive noise settings
% - dt         : time step (s)
% - sensor_type: 'IMU' | 'GPS' | 'Baro' | 'Mag' | 'AccelTilt'
%
% OUTPUTS
% - x, P       : posterior state and covariance after predict/update

%% 1) Input validation and clamping (matching EKF robustness)
if any(~isfinite(x)) || any(~isfinite(P(:)))
    warning('UKF: Invalid input state or covariance. Using safe defaults.');
    x = zeros(9,1);
    P = 0.1*eye(9);
end

% Bounds for state variables (consistent with EKF)
pos_lim = 1000;    % Position limit (m)
vel_lim = 50;      % Velocity limit (m/s)
roll_pitch_lim = pi/2; % roll/pitch limit (rad)

% Clamp state to reasonable bounds
x(1:3) = max(min(x(1:3), pos_lim), -pos_lim);  % Position
x(4:6) = max(min(x(4:6), vel_lim), -vel_lim);  % Velocity
x(7:8) = max(min(x(7:8), roll_pitch_lim), -roll_pitch_lim);
x(9) = wrapToPi(x(9));

n = 9;
% UKF scaling parameters (optimized for stability)
alpha = 0.3;   % broader spread → better conditioning
beta  = 2;     % optimal for Gaussian
kappa = 0;
lambda = alpha^2*(n + kappa) - n;
c = n + lambda;
Wm = [lambda/c, repmat(1/(2*c), 1, 2*n)];
Wc = Wm; Wc(1) = Wc(1) + (1 - alpha^2 + beta);

% Sigma points
% Ensure P is symmetric positive definite before factorization
P = (P + P')/2;
jitter = 1e-12;
S = [];
for attempt = 1:6
    M = (c)*(P + jitter*eye(n)); M = (M + M')/2;
    try
        S = chol(M, 'lower');
        break;
    catch
        jitter = jitter * 10;
    end
end
if isempty(S)
    % Eigenvalue fallback to construct a valid square-root factor
    [U,D] = eig(M);
    d = real(diag(D));
    d(d < 1e-12) = 1e-12;
    S = U * diag(sqrt(d));
end
X = zeros(n, 2*n+1);
X(:,1) = x;
for i=1:n
    X(:,i+1)   = x + S(:,i);
    X(:,i+1+n) = x - S(:,i);
end

% Process function: f(x, imu)
X_pred = zeros(size(X));
for i=1:size(X,2)
    xd = X(:,i) + drone_dynamics_imu(0, X(:,i), imu, params) * dt;
    xd(7:9) = wrapToPi(xd(7:9));
    X_pred(:,i) = xd;
end

% Predicted mean and covariance
x_pred = X_pred * Wm';
P_pred = zeros(n);
for i=1:2*n+1
    dx = X_pred(:,i) - x_pred;
    % ensure small-angle consistency for attitude residual
    dx(7:9) = atan2(sin(dx(7:9)), cos(dx(7:9)));
    P_pred = P_pred + Wc(i) * (dx * dx');
end

%% 2) Adaptive Process Noise (matching EKF adaptive features)
% Base process noise
Q_base = params.Q * dt;

% Adaptive noise scaling based on flight dynamics (if enabled)
noise_scale = 1.0;
if isfield(params, 'adaptive_noise') && params.adaptive_noise
    gyro_rate = norm(imu(4:6));
    turn_threshold = params.turn_threshold;
    
    if gyro_rate > turn_threshold
        % Increased noise during turns for better tracking
        noise_scale = params.noise_scale_turn;
    else
        % Reduced noise during normal flight for smoother estimates
        noise_scale = params.noise_scale_normal;
    end
end

Q = Q_base * noise_scale;
P_pred = (P_pred + P_pred')/2 + Q;

switch sensor_type
    case 'IMU'
        % Condition P to be SPD
        P = (P_pred + P_pred')/2 + 1e-12*eye(n);
        x = x_pred; 
        return;
    case 'GPS'
        h = @(x) x(1:3);
        R = params.R_gps;
        m = 3;
    case 'Baro'
        h = @(x) -x(3);
        R = params.R_baro;
        m = 1;
    case 'Mag'
        h = @(x) x(9);
        R = params.R_mag;
        m = 1;
    case 'AccelTilt'
        % Accelerometer-derived tilt measurement: z = [phi_meas; theta_meas]
        % Gate on near-1g and low angular rate to avoid dynamic acceleration
        a_b = imu(1:3);
        % Use NED gravity vector consistently
        if isscalar(params.g)
            g_n = [0; 0; params.g];
        else
            gv = params.g(:);
            if numel(gv) == 3
                if gv(3) < 0
                    gv = -gv; % enforce positive-down NED gravity
                end
                g_n = gv;
            else
                g_n = [0; 0; 9.81];
            end
        end
        gmag = norm(g_n);
        amag = norm(a_b);
        gyro_rate = norm(imu(4:6));
        % Use tuned gates from params if available
        if isfield(params, 'Tilt')
            accel_tol = params.Tilt.accel_tol;
            rate_gate = params.Tilt.rate_gate;
        else
            accel_tol = 5.0;  % default
            rate_gate = deg2rad(100);
        end
        if abs(amag - gmag) > accel_tol || gyro_rate > rate_gate
            % Skip update under high dynamics
            P = (P_pred + P_pred')/2 + 1e-12*eye(n);
            x = x_pred; 
            return;
        end
        % Use ZYX notation: phi=roll (about x), theta=pitch (about y), psi=yaw (about z)
        % For body frame accel [ax; ay; az], gravity direction gives:
        % phi = atan2(ay, az)  (roll from y-z components)
        % theta = -asin(ax/|a|) (pitch from x component, negative for NED)
        phi_meas = atan2(a_b(2), a_b(3));  % roll from ay, az
        theta_meas = -asin(max(min(a_b(1)/max(amag,1e-6),1),-1));  % pitch from ax
        z = [phi_meas; theta_meas];
        h = @(x) x(7:8);
        % Use tuned tilt noise if available
        if isfield(params, 'Tilt') && isfield(params.Tilt, 'R_deg')
            rdeg = params.Tilt.R_deg;
        else
            rdeg = 3.0;
        end
        R = diag([deg2rad(rdeg)^2, deg2rad(rdeg)^2]);
        m = 2;
    otherwise
        error('Unknown sensor type');
end

% Transform sigma points
Z = zeros(m, 2*n+1);
for i=1:2*n+1
    Z(:,i) = h(X_pred(:,i));
end

% Predicted measurement mean
if strcmp(sensor_type, 'Mag')
    % Circular mean for yaw
    ssum = 0; csum = 0;
    for i=1:2*n+1
        zi = Z(1,i);
        ssum = ssum + Wm(i) * sin(zi);
        csum = csum + Wm(i) * cos(zi);
    end
    z_pred = atan2(ssum, csum);
else
    z_pred = Z * Wm';
end
Pzz = zeros(m);
Pxz = zeros(n, m);
for i=1:2*n+1
    dz = Z(:,i) - z_pred; if m==1, dz = dz(:); end
    if strcmp(sensor_type, 'Mag')
        dz(1) = atan2(sin(dz(1)), cos(dz(1)));
    end
    dx = X_pred(:,i) - x_pred;
    dx(7:9) = atan2(sin(dx(7:9)), cos(dx(7:9)));
    Pzz = Pzz + Wc(i) * (dz * dz');
    Pxz = Pxz + Wc(i) * (dx * dz');
end
Pzz = (Pzz + Pzz')/2 + R + 1e-12*eye(m);
% Ensure innovation covariance is SPD via adaptive jitter if needed
meas_jitter = 1e-12;
for attempt = 1:6
    Mzz = (Pzz + meas_jitter*eye(m)); Mzz = (Mzz + Mzz')/2;
    try
        chol(Mzz);
        Pzz = Mzz;
        break;
    catch
        meas_jitter = meas_jitter * 10;
    end
end
%% Innovation Gating (matching EKF robustness features)
innov = z - z_pred;
if strcmp(sensor_type, 'Mag')
    innov = atan2(sin(innov), cos(innov));
end

% Innovation gating to reject outliers
innovation_rejected = false;
if isfield(params, 'innovation_gate_gps') && strcmp(sensor_type, 'GPS')
    gate_threshold = params.innovation_gate_gps;
    if norm(innov) > gate_threshold
        warning('UKF: GPS innovation rejected (norm = %.2f > %.2f)', norm(innov), gate_threshold);
        innovation_rejected = true;
    end
elseif isfield(params, 'innovation_gate_baro') && strcmp(sensor_type, 'Baro')
    gate_threshold = params.innovation_gate_baro;
    if abs(innov) > gate_threshold
        warning('UKF: Baro innovation rejected (|innov| = %.2f > %.2f)', abs(innov), gate_threshold);
        innovation_rejected = true;
    end
elseif isfield(params, 'innovation_gate_mag') && strcmp(sensor_type, 'Mag')
    gate_threshold = params.innovation_gate_mag;
    if abs(innov) > gate_threshold
        warning('UKF: Mag innovation rejected (|innov| = %.2f > %.2f)', abs(innov), gate_threshold);
        innovation_rejected = true;
    end
end

% Apply update only if innovation passes gate
if innovation_rejected
    % Skip update, return prediction
    x = x_pred;
    P = P_pred;
else
    % Standard UKF update
    K = Pxz / Pzz;
    x = x_pred + K * innov;
    P = P_pred - K * Pzz * K';
end

% Final conditioning and validation
% Symmetrize and add small jitter to maintain SPD
P = (P + P')/2 + 1e-12*eye(n);

% Clamp updated state to reasonable bounds
x(1:3) = max(min(x(1:3), pos_lim), -pos_lim);  % Position
x(4:6) = max(min(x(4:6), vel_lim), -vel_lim);  % Velocity
x(7:8) = max(min(x(7:8), roll_pitch_lim), -roll_pitch_lim);
x(7:9) = wrapToPi(x(7:9));

% Final validation of outputs
if any(~isfinite(x)) || any(~isfinite(P(:)))
    warning('UKF: Invalid final state or covariance. Using prediction.');
    x = x_pred;
    P = P_pred + 1e-6*eye(n);
end

end


