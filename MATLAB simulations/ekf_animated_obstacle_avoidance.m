function [x, P] = ekf_animated_obstacle_avoidance(x, P, imu, z, params, dt, sensor_type)
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
    e = e + w_b*dt;                   % small-angle Euler integration
    e(3) = atan2(sin(e(3)), cos(e(3)));

    x = [p; v; e];

    % Linearization (simple/stable)
    F = I9;
    F(1:3,4:6) = dt*eye(3);           % dp/dv

    % Noise mapping (accel -> p,v ; gyro -> att)
    G = zeros(9,6);
    G(1:3,1:3) = 0.5*dt^2 * eye(3);   % p <- accel
    G(4:6,1:3) = dt * eye(3);         % v <- accel
    G(7:9,4:6) = dt * eye(3);         % e <- gyro

    % IMU continuous PSDs (tuned / from params)
    sigma_a = max(1e-4, params.IMU.accel_noise_density); % m/s^2/√Hz
    sigma_g = max(1e-5, params.IMU.gyro_noise_density);  % rad/s/√Hz
    Qc = diag([ (sigma_a^2)*ones(1,3), (sigma_g^2)*ones(1,3) ]);
    Q  = G * Qc * G.';                % discrete approx

    P = F*P*F.' + Q;
end

% ===== Measurement updates =====
switch lower(sensor_type)
    case 'gps'     % z = [px; py; pz] (NED)
        H = [eye(3), zeros(3,6)];
        R = params.R_gps;             % 3x3
        y = z - x(1:3);
        S = H*P*H.' + R;
        K = P*H.'/S;
        x = x + K*y;
        P = (I9 - K*H)*P;

    case 'baro'    % z = altitude (up); you generate z = -pz + noise
        H = [0 0 -1  0 0 0  0 0 0];   % h(x) = -pz
        R = params.R_baro;            % scalar
        y = z - (-x(3));
        S = H*P*H.' + R;
        K = P*H.'/S;
        x = x + K*y;
        P = (I9 - K*H)*P;

    case 'mag'     % z = yaw (rad)
        H = [0 0 0  0 0 0  0 0 1];
        R = params.R_mag;             % scalar
        innov = atan2(sin(z - x(9)), cos(z - x(9)));
        S = H*P*H.' + R;
        K = P*H.'/S;
        x = x + K*innov;
        P = (I9 - K*H)*P;

    case 'imu'
        % no measurement update here (prediction already done)
    otherwise
        error('Unknown sensor_type: %s', sensor_type);
end

% Wrap yaw for safety
x(9) = atan2(sin(x(9)), cos(x(9)));
end
