function [xn, Pe] = eskf15_step(xn, Pe, imu, z, params, dt, sensor_type)
% eskf15_step - Error-State Kalman Filter (15-state: position, velocity, attitude, biases)
% Nominal state xn = [pos; vel; att; b_a; b_g]
% Error state delta = [dpos; dvel; dtheta; dba; dbg]

% Unpack nominal
pos = xn(1:3); vel = xn(4:6); att = xn(7:9);
b_a = xn(10:12); b_g = xn(13:15);

% Bias-compensated IMU
accel = imu(1:3) - b_a;
gyro  = imu(4:6) - b_g;

% Nominal propagation
x9 = [pos; vel; att];
x9_dot = drone_dynamics_imu(0, x9, [accel; gyro], params);
x9 = x9 + x9_dot * dt; x9(7:9) = wrapToPi(x9(7:9));

% Bias dynamics (random walk with slow decay)
tau_ba = 500; tau_bg = 1000;
b_a = b_a + (-1/tau_ba) * b_a * dt;
b_g = b_g + (-1/tau_bg) * b_g * dt;

xn = [x9; b_a; b_g];

% Linear error dynamics (simplified, reuse F9 for structure)
F9 = calculate_F_sensor_only([pos; vel; att], [accel; gyro], dt);
F = blkdiag(F9, eye(3) - (dt/tau_ba)*eye(3), eye(3) - (dt/tau_bg)*eye(3));
Q = blkdiag(params.Q*dt, (1e-4)*eye(3)*dt, (1e-6)*eye(3)*dt);

% Error covariance predict
Pe = F*Pe*F' + Q;

switch sensor_type
    case 'IMU'
        return;
    case 'GPS'
        H = [eye(3), zeros(3,12)];
        R = params.R_gps;
        y = z - (xn(1:3));
    case 'Baro'
        H = [0 0 -1 zeros(1,12)];
        R = params.R_baro;
        y = z - (-xn(3));
    case 'Mag'
        H = [zeros(1,8) 1 zeros(1,6)];
        R = params.R_mag;
        % Wrap yaw residual into [-pi, pi]
        y = atan2(sin(z - xn(9)), cos(z - xn(9)));
    otherwise
        error('Unknown sensor type');
end

S = H*Pe*H' + R + 1e-9*eye(size(H,1));
K = Pe*H'/S;
delta = K * y;
Pe = (eye(15) - K*H) * Pe;

% Inject error into nominal
xn(1:3) = xn(1:3) + delta(1:3);
xn(4:6) = xn(4:6) + delta(4:6);
% Small-angle attitude correction
dtheta = delta(7:9);
xn(7:9) = wrapToPi(xn(7:9) + dtheta);
xn(10:12) = xn(10:12) + delta(10:12);
xn(13:15) = xn(13:15) + delta(13:15);

end


