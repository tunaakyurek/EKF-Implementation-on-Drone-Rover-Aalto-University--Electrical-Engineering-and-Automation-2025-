function sensors = sensor_model(x, params, t)
%% PURPOSE
% Generate synthetic sensor readings (IMU, GPS, barometer, magnetometer)
% from the true state, including noise and slowly-varying biases.
%
% INPUTS
% - x      : true state [pos(3); vel(3); att(3)]
% - params : parameters struct with sensor noise settings
% - t      : current time (s), used for discrete-time updates
%
% OUTPUTS
% - sensors: struct with fields accel, gyro, gps, baro, mag
%
% MAJOR STEPS
% 1) Initialize persistent biases and previous values
% 2) Compute true specific force and body rates from finite differences
% 3) Add noise and biases to form sensor outputs
% 4) Update persistent memory for next call

persistent accel_bias gyro_bias prev_t prev_vel prev_att
if isempty(accel_bias)
    accel_bias = params.IMU.accel_bias_instab * randn(3,1);
    gyro_bias  = params.IMU.gyro_bias_instab  * randn(3,1);
    prev_t     = t;
    prev_vel   = x(4:6);
    prev_att   = x(7:9);
end

%% 1) Unpack true values
pos = x(1:3);
vel = x(4:6);
att = x(7:9);
 g = params.g;

%% 2) Finite-difference step aligned to IMU period
dt_fd = params.Ts.IMU;

%% 3) IMU (body frame) mechanization
R = rotation_matrix(att(1), att(2), att(3));

% True linear acceleration in NED via finite difference
a_true_ned = (vel - prev_vel) / dt_fd;

% Specific force in body frame: f_b = R' * (a - g)
f_body = R' * (a_true_ned - g);

% True Euler angle rates via finite difference (with angle wrapping)
ang_diff = [
	atan2(sin(att(1) - prev_att(1)), cos(att(1) - prev_att(1)));...
	atan2(sin(att(2) - prev_att(2)), cos(att(2) - prev_att(2)));...
	atan2(sin(att(3) - prev_att(3)), cos(att(3) - prev_att(3)))
];
att_dot = ang_diff / dt_fd;

% Map Euler angle rates to body rates: att_dot = E * omega_body
phi = att(1); theta = att(2);
E = [1, sin(phi)*tan(theta),  cos(phi)*tan(theta);
     0, cos(phi),            -sin(phi);
     0, sin(phi)/cos(theta),  cos(phi)/cos(theta)];

if rcond(E) > 1e-6
    omega_body = E \ att_dot;
else
    omega_body = zeros(3,1);
end

%% 4) Add sensor noises and biases
% IMU noise per sample (discrete-time) using noise densities
accel_noise = params.IMU.accel_noise_density / sqrt(params.Ts.IMU) * randn(3,1);
gyro_noise  = params.IMU.gyro_noise_density  / sqrt(params.Ts.IMU) * randn(3,1);

% Compose IMU measurements
sensors.accel = f_body + accel_bias + accel_noise;
sensors.gyro  = omega_body + gyro_bias  + gyro_noise;

% GPS (position)
gps_noise = [params.GPS.sigma_xy*randn(2,1); params.GPS.sigma_z*randn(1,1)];
sensors.gps = pos + gps_noise;

% Barometer (altitude = -z in NED)
baro_noise = params.Baro.sigma_z * randn;
sensors.baro = -pos(3) + baro_noise;

% Magnetometer (yaw/heading)
mag_noise = params.Mag.sigma_rad * randn;
sensors.mag = att(3) + mag_noise;

%% 5) Update persistence
prev_t = t;
prev_vel = vel;
prev_att = att;

end 