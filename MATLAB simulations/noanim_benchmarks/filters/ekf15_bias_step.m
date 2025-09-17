function [x, P] = ekf15_bias_step(x, P, imu, z, params, dt, sensor_type)
% ekf15_bias_step - 15-state EKF with accel/gyro biases
% x = [pos(3); vel(3); att(3); b_a(3); b_g(3)]

% Unpack
pos = x(1:3); vel = x(4:6); att = x(7:9);
b_a = x(10:12); b_g = x(13:15);

accel = imu(1:3) - b_a;  % bias-compensated
gyro  = imu(4:6) - b_g;

% Nonlinear predict on full state (use same mechanization but with bias-comp IMU)
x9 = [pos; vel; att];
x9_dot = drone_dynamics_imu(0, x9, [accel; gyro], params);
x9_pred = x9 + x9_dot * dt;
x9_pred(7:9) = wrapToPi(x9_pred(7:9));

% Bias random walk
tau_ba = 500; tau_bg = 1000; % correlation times (s)
b_a_pred = b_a + (-1/tau_ba)*b_a*dt;
b_g_pred = b_g + (-1/tau_bg)*b_g*dt;

xp = [x9_pred; b_a_pred; b_g_pred];

% Jacobian F (block form)
F9 = calculate_F_sensor_only(x9, [accel; gyro], dt);
F = blkdiag(F9, eye(3) - (dt/tau_ba)*eye(3), eye(3) - (dt/tau_bg)*eye(3));

% Process noise
Q9 = params.Q * dt;
Qb = blkdiag((1e-4)*eye(3)*dt, (1e-6)*eye(3)*dt);
Q = blkdiag(Q9, Qb);

% Covariance predict
Pp = F*P*F' + Q;

% Measurement updates
switch sensor_type
    case 'IMU'
        x = xp; P = Pp; return;
    case 'GPS'
        H = [eye(3), zeros(3,12)];
        R = params.R_gps;
    case 'Baro'
        H = [0 0 -1 zeros(1,12)];
        R = params.R_baro;
    case 'Mag'
        H = [zeros(1,8), 1, zeros(1,6)];
        R = params.R_mag;
    otherwise
        error('Unknown sensor type');
end

% Angle wrapping for yaw measurement residual
if strcmp(sensor_type, 'Mag')
    y = atan2(sin(z - xp(9)), cos(z - xp(9)));
else
    y = z - H*xp;
end
S = H*Pp*H' + R + 1e-9*eye(size(H,1));
K = Pp*H'/S;
x = xp + K*y;
P = (eye(15) - K*H) * Pp;

% Bounds
x(7:9) = wrapToPi(x(7:9));

end


