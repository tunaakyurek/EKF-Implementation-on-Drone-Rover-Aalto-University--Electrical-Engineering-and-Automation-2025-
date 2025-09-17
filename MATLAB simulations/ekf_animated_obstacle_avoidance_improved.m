function [x, P] = ekf_animated_obstacle_avoidance_improved(x, P, imu, z, params, dt, sensor_type)
% State: x = [px; py; pz; vx; vy; vz; roll; pitch; yaw] (NED, +Z down)
% Covariance: P (9x9)
% imu = [ax_b; ay_b; az_b; gx_b; gy_b; gz_b]
% z   = measurement (depends on sensor_type)
% sensor_type in {'IMU','GPS','Baro','Mag'}

% --- Hard shape guards (will catch any accidental overwrite immediately)
assert(isequal(size(x), [9,1]), 'x must be 9x1');
assert(isequal(size(P), [9,9]), 'P must be 9x9');

I9 = eye(9);

% ===== Prediction (IMU) =====
if strcmpi(sensor_type,'IMU') && dt > 0
    % Unpack
    p = x(1:3);  v = x(4:6);  e = x(7:9);  % roll pitch yaw
    R_wb = eul2rotm([e(3), e(2), e(1)], 'ZYX');   % yaw,pitch,roll
    a_b = imu(1:3);
    w_b = imu(4:6);

    gW = [0;0;params.g(3)];          % NED (+Z down)
    a_w = R_wb * a_b + gW;           % specific force -> world accel

    % Discrete motion model
    p = p + v*dt + 0.5*a_w*dt^2;
    v = v + a_w*dt;
    % Proper Euler-angle kinematics: e_dot = T(e)*omega_b
    cphi = cos(e(1)); sphi = sin(e(1));
    cth  = cos(e(2)); sth  = sin(e(2));
    if abs(cth) < 1e-6, cth = 1e-6*sign(cth + (cth==0)); end
    T_e = [ 1,      sphi*(sth/cth),   cphi*(sth/cth);
            0,      cphi,             -sphi;
            0,      sphi/cth,         cphi/cth ];
    e = e + T_e * w_b * dt;
    e(3) = atan2(sin(e(3)), cos(e(3)));

    x = [p; v; e];

    % IMPROVED: Better Jacobian calculation for turning maneuvers
    F = calculate_improved_jacobian(x, imu, dt);

    % IMPROVED: Enhanced process noise for maneuvering scenarios
    Q = calculate_maneuver_process_noise(x, imu, dt, params);

    P = F*P*F.' + Q;
    
    % IMPROVED: Numerical conditioning to prevent divergence
    P = condition_covariance(P);
end

% ===== Measurement updates =====
switch lower(sensor_type)
    case 'gps'     % z = [px; py; pz] (NED)
        H = [eye(3), zeros(3,6)];
        R = params.R_gps;             % 3x3
        y = z - x(1:3);
        S = H*P*H.' + R;
        
        % IMPROVED: Better numerical conditioning for GPS updates
        S = S + 1e-6 * eye(size(S));
        if cond(S) > 1e12
            warning('EKF: GPS innovation covariance ill-conditioned. Skipping update.');
            return;
        end
        
        K = P*H.'/S;
        x = x + K*y;
        P = (I9 - K*H)*P;
        P = condition_covariance(P);

    case 'baro'    % z = altitude (up); you generate z = -pz + noise
        H = [0 0 -1  0 0 0  0 0 0];   % h(x) = -pz
        R = params.R_baro;            % scalar
        y = z - (-x(3));
        S = H*P*H.' + R;
        
        % IMPROVED: Regularization for baro updates
        S = S + 1e-6;
        if cond(S) > 1e12
            warning('EKF: Baro innovation covariance ill-conditioned. Skipping update.');
            return;
        end
        
        K = P*H.'/S;
        x = x + K*y;
        P = (I9 - K*H)*P;
        P = condition_covariance(P);

    case 'mag'     % z = yaw (rad)
        H = [0 0 0  0 0 0  0 0 1];
        R = params.R_mag;             % scalar
        innov = atan2(sin(z - x(9)), cos(z - x(9)));
        S = H*P*H.' + R;
        
        % IMPROVED: Regularization for mag updates
        S = S + 1e-6;
        if cond(S) > 1e12
            warning('EKF: Mag innovation covariance ill-conditioned. Skipping update.');
            return;
        end
        
        K = P*H.'/S;
        x = x + K*innov;
        P = (I9 - K*H)*P;
        P = condition_covariance(P);

    case 'imu'
        % no measurement update here (prediction already done)
    otherwise
        error('Unknown sensor_type: %s', sensor_type);
end

% Wrap yaw for safety
x(9) = atan2(sin(x(9)), cos(x(9)));
end

function F = calculate_improved_jacobian(x, imu, dt)
% IMPROVED: Better Jacobian calculation that properly handles turning maneuvers
% Based on the robust implementation from calculate_F_sensor_only.m

phi = x(7); theta = x(8); psi = x(9);
a_b = imu(1:3);
omega_b = imu(4:6);

% Rotation matrix and its partial derivatives (ZYX, body->nav)
cphi = cos(phi); sphi = sin(phi);
cth  = cos(theta); sth = sin(theta);
cps  = cos(psi); sps = sin(psi);

R = [ cth*cps,            cps*sth*sphi - sps*cphi,  cps*sth*cphi + sps*sphi;...
      cth*sps,            sps*sth*sphi + cps*cphi,  sps*sth*cphi - cps*sphi;...
      -sth,               cth*sphi,                 cth*cphi ];

% dR/dphi
dR_dphi = [ 0,                 cps*sth*cphi + sps*sphi,  -cps*sth*sphi + sps*cphi;...
            0,                 sps*sth*cphi - cps*sphi,  -sps*sth*sphi - cps*cphi;...
            0,                 cth*cphi,                 -cth*sphi ];

% dR/dtheta
dR_dth = [ -sth*cps,           cps*cth*sphi,             cps*cth*cphi;...
           -sth*sps,           sps*cth*sphi,             sps*cth*cphi;...
           -cth,               -sth*sphi,                -sth*cphi ];

% dR/dpsi
dR_dpsi = [ -cth*sps,          -sps*sth*sphi - cps*cphi, -sps*sth*cphi + cps*sphi;...
             cth*cps,           cps*sth*sphi - sps*cphi,  cps*sth*cphi + sps*sphi;...
             0,                 0,                        0 ];

% A = ∂(R a)/∂[phi,theta,psi]  (3x3)
A = [dR_dphi*a_b, dR_dth*a_b, dR_dpsi*a_b];

% Euler-rate matrix T(φ,θ) and its partials
costh = cth;  % guard if needed
if abs(costh) < 1e-6, costh = 1e-6*sign(costh + (costh==0)); end
tanth = sth/costh; sec2 = 1/(costh^2);

T = [ 1,      sphi*tanth,          cphi*tanth;...
      0,      cphi,                -sphi;...
      0,      sphi/costh,          cphi/costh ];

% dT/dphi
dT_dphi = [ 0,      cphi*tanth,        -sphi*tanth;...
            0,     -sphi,              -cphi;...
            0,      cphi/costh,        -sphi/costh ];

% dT/dtheta
dT_dth = [ 0,      sphi*sec2,          cphi*sec2;...
           0,      0,                  0;...
           0,      sphi*sth/(costh^2), cphi*sth/(costh^2) ];

% B = ∂(T ω)/∂[phi,theta,psi]  (3x3)  (note: no psi dependence)
B = [dT_dphi*omega_b, dT_dth*omega_b, [0;0;0]];

% Assemble F with improved structure
F = eye(9);
F(1:3,4:6)   = dt*eye(3);      % p_k = p + v dt
F(4:6,7:9)   = dt*A;           % v_k sensitivity to angles via R(angles)*a
F(7:9,7:9)   = eye(3) + dt*B;  % angles kinematics via T(angles)*omega

% Small regularization for numerical safety
F = F + 1e-12*eye(9);
end

function Q = calculate_maneuver_process_noise(x, imu, dt, params)
% IMPROVED: Enhanced process noise calculation for maneuvering scenarios

% Base process noise from parameters - ensure it's 9x9
Q_base = params.Q * dt;
if size(Q_base,1) ~= 9 || size(Q_base,2) ~= 9
    % If params.Q is not 9x9, create a proper 9x9 matrix
    Q_diag = diag(params.Q);
    if length(Q_diag) == 9
        Q_base = diag(Q_diag) * dt;
    else
        % Fallback: create default 9x9 process noise
        Q_base = diag([0.02, 0.02, 0.02, 0.15, 0.15, 0.18, 0.08, 0.08, 0.10]) * dt;
    end
end

% Detect maneuvering conditions
vel_mag = norm(x(4:6));
omega_mag = norm(imu(4:6));
accel_mag = norm(imu(1:3));

% Increase process noise during maneuvers
maneuver_factor = 1.0;

% High angular rates indicate turning
if omega_mag > deg2rad(30)  % > 30 deg/s
    maneuver_factor = maneuver_factor * (1 + omega_mag / deg2rad(100));
end

% High accelerations indicate aggressive maneuvers
if accel_mag > 2.0  % > 2 m/s^2
    maneuver_factor = maneuver_factor * (1 + accel_mag / 5.0);
end

% High velocities with changes indicate maneuvering
if vel_mag > 3.0  % > 3 m/s
    maneuver_factor = maneuver_factor * (1 + vel_mag / 10.0);
end

% Apply maneuver factor to velocity and attitude process noise
Q = Q_base;
Q(4:6, 4:6) = Q_base(4:6, 4:6) * maneuver_factor;  % Velocity uncertainty
Q(7:9, 7:9) = Q_base(7:9, 7:9) * maneuver_factor;  % Attitude uncertainty

% Ensure Q remains positive definite
[U, S, V] = svd(Q);
S = max(S, 1e-12);
Q = U * S * V';
end

function P = condition_covariance(P)
% IMPROVED: Robust covariance conditioning to prevent numerical issues

% SVD-based conditioning
[U, S, V] = svd(P);

% Prevent singular values from becoming too small or too large
S = max(S, 1e-12);  % Prevent singularity
S = min(S, 1e6);    % Prevent explosion

% Reconstruct conditioned covariance
P = U * S * V';

% Ensure symmetry (numerical errors can break symmetry)
P = (P + P') / 2;
end