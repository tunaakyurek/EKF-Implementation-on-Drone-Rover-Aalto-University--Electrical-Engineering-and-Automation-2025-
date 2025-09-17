function [x_est, P] = ekf_sensor_only_oa(x_est, P, imu, z, params, dt, sensor_type)
%% OA-specific EKF (sensor-only): predict once at IMU rate; no re-predict on updates

% Input validation and clamping
if any(~isfinite(x_est)) || any(~isfinite(P(:)))
    x_est = zeros(9,1);
    P = 0.1*eye(9);
end
pos_lim = 1000; vel_lim = 50; roll_pitch_lim = pi/2;
x_est(1:3) = max(min(x_est(1:3), pos_lim), -pos_lim);
x_est(4:6) = max(min(x_est(4:6), vel_lim), -vel_lim);
x_est(7:8) = max(min(x_est(7:8), roll_pitch_lim), -roll_pitch_lim);
x_est(9) = wrapToPi(x_est(9));

I9 = eye(9);

% Prediction only on IMU tick
if strcmpi(sensor_type,'IMU')
    try
        x_pred = x_est + drone_dynamics_imu(0, x_est, imu, params) * dt;
    catch
        x_pred = x_est + [x_est(4:6); zeros(3,1); zeros(3,1)] * dt;
    end
    x_pred(7:8) = max(min(x_pred(7:8), deg2rad(60)), -deg2rad(60));
    x_pred(9) = wrapToPi(x_pred(9));
    if any(~isfinite(x_pred)), x_pred = x_est; end

    F = calculate_F_sensor_only(x_pred, imu, dt);
    Q = build_Q_9state(params, dt, imu);
    % Condition Q
    [Uq, Sq, Vq] = svd(Q); Sq = max(Sq, 1e-12); Q = Uq*Sq*Vq';
    P_pred = F*P*F' + Q;
    % Condition P
    [Up, Sp, Vp] = svd(P_pred); Sp = max(Sp, 1e-12); Sp = min(Sp, 1e6); P_pred = Up*Sp*Vp';
else
    x_pred = x_est;
    P_pred = P;
end

% Measurement update
switch sensor_type
    case 'IMU'
        x_est = x_pred; P = P_pred;
    case 'GPS'
        H = [eye(3), zeros(3,6)]; R = params.R_gps; y = z - H*x_pred;
        S = H*P_pred*H' + R; S = S + 1e-6*eye(size(S));
        if cond(S) > 1e12
            x_est = x_pred; P = P_pred;
        else
            K = P_pred*H'/S; x_est = x_pred + K*y; P = (I9 - K*H)*P_pred*(I9 - K*H)' + K*R*K';
        end
    case 'Baro'
        H = [0 0 -1 zeros(1,6)]; R = params.R_baro; y = z - H*x_pred;
        S = H*P_pred*H' + R; S = S + 1e-6;
        if cond(S) > 1e12
            x_est = x_pred; P = P_pred;
        else
            K = P_pred*H'/S; x_est = x_pred + K*y; P = (I9 - K*H)*P_pred*(I9 - K*H)' + K*R*K';
        end
    case 'Mag'
        H = [zeros(1,8), 1]; R = params.R_mag; y = atan2(sin(z - x_pred(9)), cos(z - x_pred(9)));
        S = H*P_pred*H' + R; S = S + 1e-6;
        if cond(S) > 1e12
            x_est = x_pred; P = P_pred;
        else
            K = P_pred*H'/S; x_est = x_pred + K*y; P = (I9 - K*H)*P_pred*(I9 - K*H)' + K*R*K';
        end
    otherwise
        error('Unknown sensor type');
end

% Final conditioning and bounds
if any(~isfinite(x_est)) || any(~isfinite(P(:)))
    x_est = x_pred; P = P_pred;
end
x_est(1:3) = max(min(x_est(1:3), pos_lim), -pos_lim);
x_est(4:6) = max(min(x_est(4:6), vel_lim), -vel_lim);
x_est(7:8) = max(min(x_est(7:8), roll_pitch_lim), -roll_pitch_lim);
x_est(9) = wrapToPi(x_est(9));
[Ue, Se, Ve] = svd(P); Se = max(Se, 1e-12); Se = min(Se, 1e6); P = Ue*Se*Ve';

end

function Q = build_Q_9state(params, dt, imu)
% Construct 9x9 process noise with pos–vel coupling and attitude RW
if isfield(params,'IMU') && isfield(params.IMU,'accel_noise_density')
    sigma_a = params.IMU.accel_noise_density; % m/s^2 / sqrt(Hz)
else
    sigma_a = 0.02;
end
if isfield(params,'IMU') && isfield(params.IMU,'gyro_noise_density')
    sigma_w = params.IMU.gyro_noise_density; % rad/s / sqrt(Hz)
else
    sigma_w = deg2rad(0.5);
end
sa2 = sigma_a^2; sw2 = sigma_w^2;
Qa = [ dt^4/4, dt^3/2; dt^3/2, dt^2 ] * sa2; % per-axis [pos; vel]
Qpv = blkdiag(Qa, Qa, Qa); % 6x6
Qatt = (dt^2) * sw2 * eye(3);
Q = blkdiag(Qpv, Qatt);

% Adaptive inflation during aggressive maneuvers
try
    omega_mag = norm(imu(4:6));
    accel_mag = norm(imu(1:3));
    gmag = isfield(params,'g') * abs(params.g(3));
    if isempty(gmag) || gmag == 0, gmag = 9.81; end
    % relative excess specific force over gravity
    accel_excess = max(0, abs(accel_mag - gmag)) / gmag; % 0 when near static
    % scale factors
    scale_vel = 1 + 0.6*accel_excess + 0.4*min(1, omega_mag/deg2rad(120));
    scale_att = 1 + 0.3*accel_excess + 0.8*min(1, omega_mag/deg2rad(120));
    S = blkdiag(scale_vel*eye(6), scale_att*eye(3));
    Q = S * Q * S;  % inflate along blocks
catch
end
end



