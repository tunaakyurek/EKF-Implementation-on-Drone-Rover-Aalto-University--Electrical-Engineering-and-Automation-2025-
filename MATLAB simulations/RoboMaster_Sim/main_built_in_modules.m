function main_built_in_modules()
% MAIN_BUILT_IN_MODULES  2D rover EKF using MATLAB built-in sensor models when available

% Params (use optimized parameters for better performance)
params = parameters_rm_optimized();

% Switch to wheel encoder sensor model for better velocity estimation
fprintf('Using wheel encoder sensor model instead of GPS velocity\n');
dt = params.dt;
N  = round(params.T / dt);

% Detect built-in sensors
hasImu = exist('imuSensor','class') == 8;
hasGps = exist('gpsSensor','class') == 8; %#ok<NASGU>

if ~hasImu
    fprintf('Built-in imuSensor not found. Falling back to code-based realistic sensors.\n');
    main_rover_sim_realistic();
    return;
end

% Ground truth state
x_true = params.x0_true;

% EKF (same implementation and improvements)
ekf = ekf8_init(params);

% ===== IMPROVED REALISTIC INITIALIZATION =====
% Get initial sensor measurements for proper initialization
u0 = [0;0;0];
meas0 = sensor_model_wheel_encoders(x_true, u0, params);

% Position: Use GPS if available, otherwise start with some uncertainty
if ~isempty(meas0.gps_pos) && all(isfinite(meas0.gps_pos))
    initial_pos = meas0.gps_pos;  % Use GPS position
    pos_uncertainty = 2.0;        % Increased uncertainty for built-in IMU
else
    initial_pos = [0; 0];         % Assume starting at origin
    pos_uncertainty = 10.0;       % Higher uncertainty if no GPS
end

% Heading: Use magnetometer if available, otherwise start with some uncertainty
if ~isempty(meas0.yaw_mag) && isfinite(meas0.yaw_mag)
    initial_heading = meas0.yaw_mag;  % Use magnetometer heading
    heading_uncertainty = 0.2;        % Increased uncertainty for built-in IMU
else
    initial_heading = 0;              % Assume starting north
    heading_uncertainty = 1.0;        % Higher uncertainty if no magnetometer
end

% Velocity: Assume starting from rest (typical for rover experiments)
initial_velocity = [0; 0];
velocity_uncertainty = 0.5;  % Increased uncertainty for built-in IMU

% Biases: Unknown initially, start with zero but high uncertainty
initial_bias_accel = [0; 0];
bias_accel_uncertainty = 0.5;  % Increased uncertainty for built-in IMU

initial_bias_gyro = 0;
bias_gyro_uncertainty = 0.05;  % Increased uncertainty for built-in IMU

ekf.x = [
    initial_pos(1);
    initial_pos(2);
    initial_heading;
    initial_velocity(1);
    initial_velocity(2);
    initial_bias_accel(1);
    initial_bias_accel(2);
    initial_bias_gyro
];
ekf.P = diag([ 
    pos_uncertainty^2; 
    pos_uncertainty^2; 
    heading_uncertainty^2; 
    velocity_uncertainty^2; 
    velocity_uncertainty^2; 
    bias_accel_uncertainty^2; 
    bias_accel_uncertainty^2; 
    bias_gyro_uncertainty^2 ]);

% Optional initial updates
if ~isempty(meas0.gps_pos) && all(isfinite(meas0.gps_pos))
    ekf = ekf8_update_gps_pos(ekf, meas0.gps_pos);
end
if ~isempty(meas0.yaw_mag) && isfinite(meas0.yaw_mag)
    ekf = ekf8_update_yaw(ekf, meas0.yaw_mag);
end
if ~isempty(meas0.wheel_vel) && all(isfinite(meas0.wheel_vel))
    ekf = ekf8_update_wheel_vel(ekf, meas0.wheel_vel);
end

% Configure built-in IMU (accelerometer + gyro + mag)
imu = imuSensor('accel-gyro-mag', 'SampleRate', 1/dt);
% Map our noise settings approximately when supported (version-dependent)
try
    if isprop(imu,'AccelerometerNoise')
        imu.AccelerometerNoise = sqrt(params.r_accel) * [1 1 1];
    end
    if isprop(imu,'GyroscopeNoise')
        imu.GyroscopeNoise = sqrt(params.r_gyro) * [1 1 1];
    end
    if isprop(imu,'MagnetometerNoise')
        imu.MagnetometerNoise = sqrt(params.r_yaw) * [1 1 1];
    end
catch
    % If properties are not available, proceed with defaults
end

% ===== COMPREHENSIVE LOGGING FOR FINE-TUNING =====
log.t = (0:N-1)' * dt;
log.x_true = zeros(N,8);
log.x_est  = zeros(N,8);

% Raw sensor measurements
log.gps_pos_raw = zeros(N,2);
log.wheel_vel_raw = zeros(N,2);  % Wheel encoder velocity (replaces GPS velocity)
log.yaw_raw = zeros(N,1);
log.accel_raw = zeros(N,2);
log.gyro_raw = zeros(N,1);

% EKF internal state for analysis
log.P_trace = zeros(N,1);           % Covariance trace for stability
log.K_norm = zeros(N,1);            % Kalman gain magnitude
log.innovation_norm = zeros(N,1);   % Innovation magnitude
log.residual_norm = zeros(N,1);     % Residual magnitude

% Sensor availability and update timing
log.sensor_available = zeros(N,4);  % [GPS_pos, Wheel_vel, Mag, IMU]
log.update_count = zeros(N,1);      % Number of updates per step
log.gps_update_step = zeros(N,1);   % GPS update step counter
log.mag_update_step = zeros(N,1);   % Magnetometer update step counter

% Bias estimation tracking
log.bias_accel_est = zeros(N,2);    % Estimated accelerometer biases
log.bias_gyro_est = zeros(N,1);     % Estimated gyroscope bias
log.bias_accel_true = zeros(N,2);   % True accelerometer biases
log.bias_gyro_true = zeros(N,1);    % True gyroscope bias

% Performance metrics
log.position_error = zeros(N,2);     % [x_err, y_err]
log.velocity_error = zeros(N,2);     % [vx_err, vy_err]
log.yaw_error = zeros(N,1);         % yaw error
log.bias_error = zeros(N,3);        % [bax_err, bay_err, bw_err]

% Schedules
gps_step = max(1, round(1/params.rates.gps_hz / dt));
mag_step = max(1, round(1/params.rates.mag_hz / dt));

fprintf('Running main_built_in_modules with built-in IMU, custom GPS.\n');
fprintf('  - dt = %.3f s, N = %d, IMU rate = %.1f Hz, GPS = %d Hz, Mag = %d Hz\n', dt, N, 1/dt, params.rates.gps_hz, params.rates.mag_hz);

% Control profile (reuse)
u_body = @(k) [ 
    1.2 + 0.3*sin(0.008*k) + 0.15*sin(0.025*k);
    0.6*sin(0.012*k) + 0.4*sin(0.006*k) + 0.2*sin(0.018*k) + 0.3*sin(0.003*k) .* (mod(k, 1000) < 500);
    0.05*sin(0.01*k) + 0.02*sin(0.04*k)
];

useQuaternion = (exist('quaternion','class') == 8);

for k = 1:N
    % True control
    u_true = u_body(k);

    % Advance true state
    x_true = rover_dynamics(x_true, u_true, dt, params);

    % Orientation from true yaw (2D rover; roll=pitch=0)
    yaw = x_true(3);

    % Prepare both quaternion and rotation matrix (for compatibility)
    if useQuaternion
        q = quaternion([yaw 0 0],'euler','ZYX','frame'); % 1x1 quaternion
    else
        q = []; % not used
    end
    c = cos(yaw); s = sin(yaw);
    R = [ c -s  0; s  c  0; 0  0  1];          % 3x3
    R3 = reshape(R,3,3,1);                      % 3x3x1

    % Angular velocity and specific force (body) as 1x3 row vectors
    omega_b = reshape([0 0 u_true(3)],1,3);
    a_body  = reshape([u_true(1) u_true(2) 0],1,3);

    % ---- Built-in IMU measurement (version-robust) ----
    if useQuaternion
        [accel_meas, gyro_meas, mag_meas] = call_imu_compat(imu, q, R3, omega_b, a_body);
    else
        [accel_meas, gyro_meas, mag_meas] = call_imu_compat(imu, R3, R3, omega_b, a_body);
    end
    % ---------------------------------------------------

    % ===== CORRECT SENSOR DATA EXTRACTION =====
    % imuSensor outputs are already in BODY frame (accelerometer, gyroscope, magnetometer)
    % Use them directly without additional rotation

    % Magnetometer (body frame) -> yaw measurement
    mx_b = mag_meas(1,1); my_b = mag_meas(1,2);
    % Use standard body-frame heading from magnetometer
    yaw_meas = atan2(my_b, mx_b);
    yaw_meas = mod(yaw_meas + pi, 2*pi) - pi;

    % Accelerometer (body frame)
    accel_body = accel_meas(1,1:2)';         % [ax_b; ay_b]

    % Gyroscope (body frame)
    gyro_body = gyro_meas(1,3);              % wz_b
    
    % Wheel encoder sensor model (replaces GPS velocity with wheel encoders)
    meas_wheel = sensor_model_wheel_encoders(x_true, u_true, params);

    % Compose measurement struct compatible with EKF usage
    meas.accel_body = accel_body;     % Now correctly in body frame
    meas.gyro_z     = gyro_body;      % Now correctly in body frame
    meas.yaw_mag    = yaw_meas;
    meas.gps_pos    = meas_wheel.gps_pos;
    meas.wheel_vel  = meas_wheel.wheel_vel;  % Wheel encoder velocity in body frame

    % Log sensor availability and validate measurements
    log.accel_raw(k,:) = meas.accel_body(:)';
    log.gyro_raw(k)    = meas.gyro_z;
    log.sensor_available(k,4) = 1;
    
    % Validate measurements are finite
    if ~all(isfinite(meas.accel_body)) || ~isfinite(meas.gyro_z)
        warning('Non-finite IMU measurements at step %d', k);
    end
    
    if mod(k, mag_step) == 0
        log.yaw_raw(k) = meas.yaw_mag; 
        log.sensor_available(k,3) = 1;
        
        % Validate magnetometer measurement
        if ~isfinite(meas.yaw_mag)
            warning('Non-finite magnetometer measurement at step %d', k);
        end
    end
    
    if mod(k, gps_step) == 0 && (k*dt > 1.0)
        log.gps_pos_raw(k,:) = meas.gps_pos(:)';
        log.wheel_vel_raw(k,:) = meas.wheel_vel(:)';  % Log wheel encoder velocity
        log.sensor_available(k,1:2) = 1;
        
        % Validate GPS and wheel encoder measurements
        if ~all(isfinite(meas.gps_pos)) || ~all(isfinite(meas.wheel_vel))
            warning('Non-finite GPS or wheel encoder measurements at step %d', k);
        end
    end

    % EKF predict
    ekf = ekf8_predict(ekf, dt, meas.accel_body, meas.gyro_z);

    % Constraints
    ekf = ekf8_apply_constraints(ekf);

    % ===== EKF UPDATES (GPS Velocity REMOVED, Wheel Encoders Added) =====
    % GPS Position: Provides absolute position reference
    if mod(k, gps_step) == 0 && (k*dt > 1.0)
        if all(isfinite(meas.gps_pos)) && all(abs(meas.gps_pos) < 1e4)
            ekf = ekf8_update_gps_pos(ekf, meas.gps_pos);
        end
        if all(isfinite(meas.wheel_vel)) && all(abs(meas.wheel_vel) < 100)
            ekf = ekf8_update_wheel_vel(ekf, meas.wheel_vel);  % Wheel encoder velocity update
        end
    end
    
    % Magnetometer: Provides absolute heading reference
    if mod(k, mag_step) == 0 && (k*dt > 1.0)
        if isfinite(meas.yaw_mag) && abs(meas.yaw_mag) < 2*pi
            ekf = ekf8_update_yaw(ekf, meas.yaw_mag);
        end
    end
    
    % Full sensor fusion: GPS (pos) + Wheel Encoders (vel) + Magnetometer + IMU provides robust observability
% Wheel encoders provide more accurate velocity than GPS velocity, improving EKF performance

    % ===== COMPREHENSIVE STATE LOGGING =====
    % True and estimated states
    log.x_true(k,:) = x_true(:)';
    log.x_est(k,:)  = ekf.x(:)';
    
    % EKF internal state analysis
    log.P_trace(k) = trace(ekf.P);
    if k == 1
        log.update_count(k) = ekf.update_count;
    else
        log.update_count(k) = ekf.update_count - sum(log.update_count(1:k-1));
    end
    
    % Bias tracking (true vs estimated)
    log.bias_accel_true(k,:) = x_true(6:7)';
    log.bias_gyro_true(k) = x_true(8);
    log.bias_accel_est(k,:) = ekf.x(6:7)';
    log.bias_gyro_est(k) = ekf.x(8);
    
    % Performance metrics
    log.position_error(k,:) = [x_true(1) - ekf.x(1), x_true(2) - ekf.x(2)];
    log.velocity_error(k,:) = [x_true(4) - ekf.x(4), x_true(5) - ekf.x(5)];
    log.yaw_error(k) = wrapToPi_local(x_true(3) - ekf.x(3));
    log.bias_error(k,:) = [x_true(6) - ekf.x(6), x_true(7) - ekf.x(7), x_true(8) - ekf.x(8)];
    
    % Update step tracking
    if mod(k, gps_step) == 0
        log.gps_update_step(k) = 1;
    end
    if mod(k, mag_step) == 0
        log.mag_update_step(k) = 1;
    end
end

% Simple trajectory plot
figure('Name','XY Trajectory - Built-in IMU');
plot(log.x_true(:,1), log.x_true(:,2), 'k-', 'LineWidth', 1.5); hold on;
plot(log.x_est(:,1),  log.x_est(:,2),  'r--', 'LineWidth', 1.2);
axis equal; grid on; xlabel('X [m]'); ylabel('Y [m]'); legend('True','EKF');

% ===== Additional consolidated plots (match regular simulations) =====
% XY + states
figure('Name','XY Trajectory & States - Built-in IMU');
subplot(2,2,1);
plot(log.x_true(:,1), log.x_true(:,2), 'k-', 'LineWidth', 1.5); hold on;
plot(log.x_est(:,1),  log.x_est(:,2),  'r--', 'LineWidth', 1.2);
axis equal; grid on; xlabel('X [m]'); ylabel('Y [m]'); legend('True','EKF'); title('Rover XY Trajectory');

subplot(2,2,2);
plot(log.t, log.x_true(:,1), 'k', log.t, log.x_est(:,1), 'r--'); grid on;
ylabel('x [m]'); title('Position X vs Time');

subplot(2,2,3);
plot(log.t, log.x_true(:,2), 'k', log.t, log.x_est(:,2), 'r--'); grid on;
ylabel('y [m]'); title('Position Y vs Time'); xlabel('t [s]');

subplot(2,2,4);
plot(log.t, wrapToPi_local(log.x_true(:,3)), 'k', log.t, wrapToPi_local(log.x_est(:,3)), 'r--'); grid on;
ylabel('\theta [rad]'); xlabel('t [s]'); title('Yaw vs Time');

% Raw sensor data
figure('Name','Raw Sensor Data vs Time - Built-in IMU');
subplot(2,2,1);
plot(log.t, log.accel_raw(:,1), 'b.', 'MarkerSize', 1); grid on; ylabel('a_x [m/s^2]'); title('Raw Accelerometer X');
subplot(2,2,2);
plot(log.t, log.accel_raw(:,2), 'b.', 'MarkerSize', 1); grid on; ylabel('a_y [m/s^2]'); title('Raw Accelerometer Y');
subplot(2,2,3);
plot(log.t, log.gyro_raw, 'b.', 'MarkerSize', 1); grid on; ylabel('\omega [rad/s]'); title('Raw Gyroscope'); xlabel('t [s]');
subplot(2,2,4);
plot(log.t, log.gps_pos_raw(:,1), 'g.', log.t, log.gps_pos_raw(:,2), 'g.', 'MarkerSize', 1); grid on;
ylabel('Position [m]'); xlabel('t [s]'); title('Raw GPS Position (X & Y)'); legend('GPS X','GPS Y');

% Sensor vs Estimated comparison
figure('Name','Sensor vs. Estimated Comparison - Built-in IMU');
subplot(2,2,1);
plot(log.t, log.gps_pos_raw(:,1), 'g.', log.t, log.x_est(:,1), 'r-', log.t, log.x_true(:,1), 'k-');
grid on; ylabel('x [m]'); legend('GPS Raw','EKF Est','True'); title('Position X vs Time');
subplot(2,2,2);
plot(log.t, log.gps_pos_raw(:,2), 'g.', log.t, log.x_est(:,2), 'r-', log.t, log.x_true(:,2), 'k-');
grid on; ylabel('y [m]'); legend('GPS Raw','EKF Est','True'); title('Position Y vs Time');
subplot(2,2,3);
plot(log.t, log.yaw_raw, 'm.', log.t, wrapToPi_local(log.x_est(:,3)), 'r-', log.t, wrapToPi_local(log.x_true(:,3)), 'k-');
grid on; ylabel('\theta [rad]'); legend('Mag Raw','EKF Est','True'); title('Yaw vs Time');
subplot(2,2,4);
plot(log.t, log.wheel_vel_raw(:,1), 'g.', log.t, log.x_est(:,4), 'r-', log.t, log.x_true(:,4), 'k-');
grid on; ylabel('v_x [m/s]'); legend('Wheel Vel X','EKF Est','True'); title('Velocity X vs Time'); xlabel('t [s]');

% State Errors
ex = log.x_true(:,1) - log.x_est(:,1);
ey = log.x_true(:,2) - log.x_est(:,2);
eth = arrayfun(@(a,b) wrapToPi_local(a-b), log.x_true(:,3), log.x_est(:,3));
evx = log.x_true(:,4) - log.x_est(:,4);
evy = log.x_true(:,5) - log.x_est(:,5);

figure('Name','State Errors vs Time - Built-in IMU');
subplot(2,2,1); plot(log.t, ex, 'b'); grid on; ylabel('x err [m]'); title('Position X Error');
subplot(2,2,2); plot(log.t, ey, 'b'); grid on; ylabel('y err [m]'); title('Position Y Error');
subplot(2,2,3); plot(log.t, eth, 'b'); grid on; ylabel('theta err [rad]'); title('Yaw Error');
subplot(2,2,4); plot(log.t, evx, 'b', log.t, evy, 'r'); grid on; ylabel('Vel err [m/s]'); legend('v_x err','v_y err'); xlabel('t [s]'); title('Velocity Errors');

% ===== FINE-TUNING ANALYSIS PLOTS =====
fprintf('Generating fine-tuning analysis plots...\n');

% Bias estimation analysis
figure('Name','Bias Estimation Analysis - Fine-Tuning');
subplot(2,2,1);
plot(log.t, log.bias_accel_true(:,1), 'k-', log.t, log.bias_accel_est(:,1), 'r--'); 
grid on; ylabel('b_ax [m/s^2]'); legend('True','Estimated'); title('Accel Bias X');
subplot(2,2,2);
plot(log.t, log.bias_accel_true(:,2), 'k-', log.t, log.bias_accel_est(:,2), 'r--'); 
grid on; ylabel('b_ay [m/s^2]'); legend('True','Estimated'); title('Accel Bias Y');
subplot(2,2,3);
plot(log.t, log.bias_gyro_true, 'k-', log.t, log.bias_gyro_est, 'r--'); 
grid on; ylabel('b_w [rad/s]'); legend('True','Estimated'); title('Gyro Bias');
subplot(2,2,4);
plot(log.t, log.bias_error(:,1), 'b', log.t, log.bias_error(:,2), 'r', log.t, log.bias_error(:,3), 'g'); 
grid on; ylabel('Bias Error'); legend('b_ax err','b_ay err','b_w err'); xlabel('t [s]'); title('Bias Estimation Errors');

% EKF internal state analysis
figure('Name','EKF Internal State Analysis - Fine-Tuning');
subplot(2,2,1);
plot(log.t, log.P_trace); grid on; ylabel('P Trace'); title('Covariance Trace (Stability)');
subplot(2,2,2);
plot(log.t, log.update_count); grid on; ylabel('Update Count'); title('EKF Update Frequency');
subplot(2,2,3);
plot(log.t, log.gps_update_step, 'g', log.t, log.mag_update_step, 'm'); 
grid on; ylabel('Update Flag'); legend('GPS','Mag'); title('Sensor Update Timing');
subplot(2,2,4);
plot(log.t, log.position_error(:,1), 'b', log.t, log.position_error(:,2), 'r'); 
grid on; ylabel('Position Error [m]'); legend('X err','Y err'); xlabel('t [s]'); title('Position Errors');

% Performance summary for fine-tuning
fprintf('\n=== FINE-TUNING PERFORMANCE SUMMARY ===\n');
fprintf('Position X RMS Error: %.3f m\n', sqrt(mean(log.position_error(:,1).^2)));
fprintf('Position Y RMS Error: %.3f m\n', sqrt(mean(log.position_error(:,2).^2)));
fprintf('Velocity X RMS Error: %.3f m/s\n', sqrt(mean(log.velocity_error(:,1).^2)));
fprintf('Velocity Y RMS Error: %.3f m/s\n', sqrt(mean(log.velocity_error(:,2).^2)));
fprintf('Yaw RMS Error: %.3f rad\n', sqrt(mean(log.yaw_error.^2)));
fprintf('Accel Bias X RMS Error: %.3f m/s^2\n', sqrt(mean(log.bias_error(:,1).^2)));
fprintf('Accel Bias Y RMS Error: %.3f m/s^2\n', sqrt(mean(log.bias_error(:,2).^2)));
fprintf('Gyro Bias RMS Error: %.3f rad/s\n', sqrt(mean(log.bias_error(:,3).^2)));
fprintf('========================================\n');

% ===== FINE-TUNING ANALYSIS =====
fprintf('\nRunning fine-tuning analysis...\n');
[tuning_report, recommendations] = fine_tuning_logger(log, params);

% ===== ANIMATION =====
fprintf('Starting animation...\n');

% Prepare sensor data for animation
sensor_data.gps_pos = log.gps_pos_raw;
sensor_data.gps_vel = log.wheel_vel_raw;  % Use wheel encoder velocity data
sensor_data.yaw = log.yaw_raw;
sensor_data.available = log.sensor_available;

% Run animation with sensor data visualization
% Duration: 60 seconds of wall-clock time
animate_rover_xy(log.x_true(:,1:3), log.x_est(:,1:3), 60.0, true, [], sensor_data);

fprintf('Animation complete!\n');

end

function ang = wrapToPi_local(ang)
ang = mod(ang + pi, 2*pi) - pi;
end

% ===== Compatibility helper =====
function [accel_meas, gyro_meas, mag_meas] = call_imu_compat(imu, orientObj, orientR3, omega_b, a_body)
% Tries both imuSensor call signatures across MATLAB versions:
%   Newer: imu(orientation, angularVelocity, acceleration)
%   Older: imu(acceleration, angularVelocity, orientation)
%
% Inputs:
%   orientObj : quaternion OR 3x3x1 rotation matrix (preferred)
%   orientR3  : 3x3x1 rotation matrix (fallback)
%   omega_b   : 1x3
%   a_body    : 1x3

    % --- 1) Try orientation-first with quaternion or R3 ---
    try
        [accel_meas, gyro_meas, mag_meas] = imu(orientObj, omega_b, a_body);
        return;
    catch
        % continue
    end

    % --- 2) Try acceleration-first with quaternion or R3 ---
    try
        [accel_meas, gyro_meas, mag_meas] = imu(a_body, omega_b, orientObj);
        return;
    catch
        % continue
    end

    % --- 3) Force 3x3x1 rotation matrix and try again (both orders) ---
    try
        [accel_meas, gyro_meas, mag_meas] = imu(orientR3, omega_b, a_body);
        return;
    catch
        % continue
    end
    [accel_meas, gyro_meas, mag_meas] = imu(a_body, omega_b, orientR3);
end
