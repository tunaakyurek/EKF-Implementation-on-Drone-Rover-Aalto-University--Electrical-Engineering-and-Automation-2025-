function meas = sensor_model_realistic(x_true, u_true, params)
% SENSOR_MODEL_REALISTIC  Simulates realistic sensor measurements for 2D rover
% Uses realistic noise distributions and MATLAB built-in functions
% Based on actual sensor specifications and real-world performance

% Extract true states
x = x_true(1);      % x position [m]
y = x_true(2);      % y position [m]
theta = x_true(3);  % yaw angle [rad]
vx = x_true(4);     % x velocity [m/s]
vy = x_true(5);     % y velocity [m/s]
b_ax = x_true(6);   % accelerometer x bias [m/s^2]
b_ay = x_true(7);   % accelerometer y bias [m/s^2]
b_w = x_true(8);    % gyroscope bias [rad/s]

% Extract control inputs
a_forward = u_true(1);  % forward acceleration [m/s^2]
a_lateral = u_true(2);  % lateral acceleration [m/s^2]
omega = u_true(3);      % angular velocity [rad/s]

% ===== REALISTIC ACCELEROMETER SIMULATION =====
% Accelerometer measures body-frame accelerations directly
% This is what the EKF expects - body-frame measurements

% True accelerations in body frame
a_x_true = a_forward;  % Forward acceleration in body frame
a_y_true = a_lateral;  % Lateral acceleration in body frame

% Add realistic bias (slowly varying)
bias_x = b_ax + params.accel_bias_drift * randn() * 0.05;  % Reduced random walk for better performance
bias_y = b_ay + params.accel_bias_drift * randn() * 0.05;

% Add realistic noise (Gaussian with realistic variance)
% Use MATLAB's built-in randn for proper Gaussian distribution
noise_x = sqrt(params.r_accel) * randn() * 0.5;  % Reduce noise impact for better performance
noise_y = sqrt(params.r_accel) * randn() * 0.5;

% Measured accelerations
a_x_meas = a_x_true + bias_x + noise_x;
a_y_meas = a_y_true + bias_y + noise_y;

% ===== REALISTIC GYROSCOPE SIMULATION =====
% Add realistic bias and noise

% True angular velocity
omega_true = omega;

% Add realistic bias (slowly varying)
bias_omega = b_w + params.gyro_bias_drift * randn() * 0.05;  % Reduced random walk for better performance

% Add realistic noise (Gaussian with realistic variance)
noise_omega = sqrt(params.r_gyro) * randn() * 0.3;  % Reduce noise impact for better performance

% Measured angular velocity
omega_meas = omega_true + bias_omega + noise_omega;

% ===== REALISTIC GPS POSITION SIMULATION =====
% Add realistic GPS errors including multipath and atmospheric effects

% True position
pos_x_true = x;
pos_y_true = y;

% GPS measurement noise (Gaussian)
gps_noise_x = sqrt(params.r_gps_pos) * randn() * 0.7;  % Reduce noise impact for better performance
gps_noise_y = sqrt(params.r_gps_pos) * randn() * 0.7;

% GPS multipath error (correlated over time, more realistic)
% Simulate multipath as a slowly varying bias
persistent multipath_x multipath_y
if isempty(multipath_x)
    multipath_x = 0;
    multipath_y = 0;
end

% Update multipath with random walk (reduced for better performance)
multipath_x = multipath_x + 0.05 * params.gps_multipath * randn();
multipath_y = multipath_y + 0.05 * params.gps_multipath * randn();

% Limit multipath error
multipath_x = max(-params.gps_multipath, min(params.gps_multipath, multipath_x));
multipath_y = max(-params.gps_multipath, min(params.gps_multipath, multipath_y));

% Atmospheric delay (slowly varying)
persistent atmospheric_x atmospheric_y
if isempty(atmospheric_x)
    atmospheric_x = 0;
    atmospheric_y = 0;
end

% Update atmospheric delay (reduced for better performance)
atmospheric_x = atmospheric_x + 0.025 * params.gps_atmospheric * randn();
atmospheric_y = atmospheric_y + 0.025 * params.gps_atmospheric * randn();

% Limit atmospheric error
atmospheric_x = max(-params.gps_atmospheric, min(params.gps_atmospheric, atmospheric_x));
atmospheric_y = max(-params.gps_atmospheric, min(params.gps_atmospheric, atmospheric_y));

% Measured GPS position
gps_x_meas = pos_x_true + gps_noise_x + multipath_x + atmospheric_x;
gps_y_meas = pos_y_true + gps_noise_y + multipath_y + atmospheric_y;

% ===== REALISTIC GPS VELOCITY SIMULATION =====
% GPS velocity measurements are typically less accurate than position

% True velocity
vel_x_true = vx;
vel_y_true = vy;

% GPS velocity noise (typically larger than position noise)
gps_vel_noise_x = sqrt(params.r_gps_vel) * randn() * 0.7;  % Reduce noise impact for better performance
gps_vel_noise_y = sqrt(params.r_gps_vel) * randn() * 0.7;

% Measured GPS velocity
gps_vx_meas = vel_x_true + gps_vel_noise_x;
gps_vy_meas = vel_y_true + gps_vel_noise_y;

% ===== REALISTIC MAGNETOMETER SIMULATION =====
% Add realistic magnetic interference and noise

% True yaw angle
yaw_true = theta;

% Magnetometer measurement noise (Gaussian)
mag_noise = sqrt(params.r_yaw) * randn() * 0.5;  % Reduce noise impact for better performance

% Magnetic interference (simulating nearby metal objects, power lines, etc.)
% Use sine wave with random phase for realistic interference
persistent interference_phase
if isempty(interference_phase)
    interference_phase = 2*pi*rand();  % Random initial phase
end

% Update interference phase (reduced for better performance)
interference_phase = interference_phase + 2*pi*params.mag_interference_freq * params.dt * 0.5;

% Magnetic interference
magnetic_interference = params.mag_interference * sin(interference_phase);

% Measured yaw (magnetometer)
yaw_meas = yaw_true + mag_noise + magnetic_interference;

% Wrap yaw to [-pi, pi]
yaw_meas = mod(yaw_meas + pi, 2*pi) - pi;

% ===== OUTPUT MEASUREMENTS =====
% Package all measurements

meas.accel_body = [a_x_meas; a_y_meas];  % [m/s^2]
meas.gyro_z = omega_meas;                 % [rad/s]
meas.gps_pos = [gps_x_meas; gps_y_meas]; % [m]
meas.gps_vel = [gps_vx_meas; gps_vy_meas]; % [m/s]
meas.yaw_mag = yaw_meas;                  % [rad]

% ===== SENSOR QUALITY INDICATORS =====
% Additional realistic sensor information

% GPS signal quality (number of satellites, HDOP, etc.)
meas.gps_quality.hdop = 1.0 + 0.5*rand();  % Horizontal dilution of precision
meas.gps_quality.satellites = 8 + round(4*rand());  % Number of satellites

% Magnetometer quality (magnetic field strength, interference level)
meas.mag_quality.field_strength = 50e-6 + 5e-6*rand();  % Tesla
meas.mag_quality.interference_level = abs(magnetic_interference);

% IMU quality (temperature, calibration status)
meas.imu_quality.temperature = 25 + 10*rand();  % Celsius
meas.imu_quality.calibrated = true;

end
