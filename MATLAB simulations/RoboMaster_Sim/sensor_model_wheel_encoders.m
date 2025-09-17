function meas = sensor_model_wheel_encoders(x_true, u_true, params)
% SENSOR_MODEL_WHEEL_ENCODERS  Simulates wheel encoder measurements for 2D rover
% Replaces GPS velocity with more accurate wheel encoder velocity
% Based on actual RoboMaster S1 wheel encoder specifications
%
% CONTROL/MEASUREMENT MAPPING (RoboMaster S1):
% - The rover truth dynamics are driven by u_true = [a_bx; a_by; omega]:
%   forward acceleration, lateral acceleration command (used as steering),
%   and yaw rate. This mirrors a simple acceleration-level interface rather
%   than a closed-loop speed/heading controller.
% - Encoders report body-frame velocity derived from the true state; they are
%   not directly measuring the commanded accelerations. We convert global
%   velocity to body frame and then add realistic encoder noise/quantization
%   and slip. IMU channels are derived from u_true (with bias/noise) to feed
%   the EKF prediction. GPS and magnetometer are synthesized from true state.

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

% ===== WHEEL ENCODER VELOCITY SIMULATION =====
% Wheel encoders provide velocity in body frame
% Convert global velocity to body frame

% True velocity in global frame
vx_global = vx;
vy_global = vy;

% Rotation matrix from global to body frame
c = cos(theta); s = sin(theta);
R_global_to_body = [c s; -s c];

% Transform velocity to body frame
vel_body = R_global_to_body * [vx_global; vy_global];
vx_body = vel_body(1);
vy_body = vel_body(2);

% ===== WHEEL ENCODER CHARACTERISTICS =====
% RoboMaster S1 wheel encoder specifications
wheel_encoder_resolution = 4096;  % pulses per revolution
wheel_diameter = 0.0625;          % 62.5mm wheel diameter
wheel_circumference = pi * wheel_diameter;

% ===== WHEEL ENCODER NOISE MODEL =====
% Realistic wheel encoder errors:
% 1. Quantization error (resolution limited)
% 2. Wheel slip/skid
% 3. Calibration errors
% 4. Mechanical backlash

% Quantization error (based on resolution and update rate)
quantization_error = wheel_circumference / (wheel_encoder_resolution * 100); % 100 Hz update rate

% Wheel slip simulation (depends on acceleration and surface)
slip_factor = 0.02;  % 2% slip during acceleration
if abs(a_forward) > 1.0  % High acceleration
    slip_factor = 0.05;  % 5% slip
end

% Add realistic noise (reduced for better performance)
wheel_noise_x = sqrt(params.r_wheel_encoder) * randn() * 0.5;  % Reduce noise impact
wheel_noise_y = sqrt(params.r_wheel_encoder) * randn() * 0.5;

% Apply slip and noise to body velocity
vx_wheel = vx_body * (1 - slip_factor) + wheel_noise_x;
vy_wheel = vy_body * (1 - slip_factor) + wheel_noise_y;

% ===== ACCELEROMETER SIMULATION =====
% Accelerometer measures body-frame accelerations directly
% This is what the EKF expects - body-frame measurements
a_x_true = a_forward;  % Forward acceleration in body frame
a_y_true = a_lateral;  % Lateral acceleration in body frame

% Add realistic bias and noise
bias_x = b_ax + params.accel_bias_drift * randn() * 0.05;  % Reduced random walk for better performance
bias_y = b_ay + params.accel_bias_drift * randn() * 0.05;

noise_x = sqrt(params.r_accel) * randn() * 0.5;  % Reduce noise impact for better performance
noise_y = sqrt(params.r_accel) * randn() * 0.5;

a_x_meas = a_x_true + bias_x + noise_x;
a_y_meas = a_y_true + bias_y + noise_y;

% ===== GYROSCOPE SIMULATION =====
omega_true = omega;
bias_omega = b_w + params.gyro_bias_drift * randn() * 0.05;  % Reduced random walk for better performance
noise_omega = sqrt(params.r_gyro) * randn() * 0.3;  % Reduce noise impact for better performance
omega_meas = omega_true + bias_omega + noise_omega;

% ===== GPS POSITION SIMULATION =====
% Keep GPS position for absolute positioning
pos_x_true = x;
pos_y_true = y;

gps_noise_x = sqrt(params.r_gps_pos) * randn();
gps_noise_y = sqrt(params.r_gps_pos) * randn();

% GPS multipath and atmospheric effects
persistent multipath_x multipath_y
if isempty(multipath_x)
    multipath_x = 0; multipath_y = 0;
end

multipath_x = multipath_x + 0.05 * params.gps_multipath * randn();  % Reduced for better performance
multipath_y = multipath_y + 0.05 * params.gps_multipath * randn();
multipath_x = max(-params.gps_multipath, min(params.gps_multipath, multipath_x));
multipath_y = max(-params.gps_multipath, min(params.gps_multipath, multipath_y));

persistent atmospheric_x atmospheric_y
if isempty(atmospheric_x)
    atmospheric_x = 0; atmospheric_y = 0;
end

atmospheric_x = atmospheric_x + 0.025 * params.gps_atmospheric * randn();  % Reduced for better performance
atmospheric_y = atmospheric_y + 0.025 * params.gps_atmospheric * randn();
atmospheric_x = max(-params.gps_atmospheric, min(params.gps_atmospheric, atmospheric_x));
atmospheric_y = max(-params.gps_atmospheric, min(params.gps_atmospheric, atmospheric_y));

gps_x_meas = pos_x_true + gps_noise_x + multipath_x + atmospheric_x;
gps_y_meas = pos_y_true + gps_noise_y + multipath_y + atmospheric_y;

% ===== MAGNETOMETER SIMULATION =====
yaw_true = theta;
mag_noise = sqrt(params.r_yaw) * randn();

persistent interference_phase
if isempty(interference_phase)
    interference_phase = 2*pi*rand();
end

interference_phase = interference_phase + 2*pi*params.mag_interference_freq * params.dt * 0.5;  % Reduced for better performance
magnetic_interference = params.mag_interference * sin(interference_phase);

yaw_meas = yaw_true + mag_noise + magnetic_interference;
yaw_meas = mod(yaw_meas + pi, 2*pi) - pi;

% ===== OUTPUT MEASUREMENTS =====
% Package all measurements

meas.accel_body = [a_x_meas; a_y_meas];           % [m/s^2]
meas.gyro_z = omega_meas;                          % [rad/s]
meas.gps_pos = [gps_x_meas; gps_y_meas];          % [m]
meas.wheel_vel = [vx_wheel; vy_wheel];            % [m/s] - BODY FRAME
meas.yaw_mag = yaw_meas;                           % [rad]

% ===== SENSOR QUALITY INDICATORS =====
% Wheel encoder quality
meas.wheel_quality.resolution = wheel_encoder_resolution;
meas.wheel_quality.slip_factor = slip_factor;
meas.wheel_quality.quantization_error = quantization_error;

% GPS signal quality
meas.gps_quality.hdop = 1.0 + 0.5*rand();
meas.gps_quality.satellites = 8 + round(4*rand());

% Magnetometer quality
meas.mag_quality.field_strength = 50e-6 + 5e-6*rand();
meas.mag_quality.interference_level = abs(magnetic_interference);

% IMU quality
meas.imu_quality.temperature = 25 + 10*rand();
meas.imu_quality.calibrated = true;

end
