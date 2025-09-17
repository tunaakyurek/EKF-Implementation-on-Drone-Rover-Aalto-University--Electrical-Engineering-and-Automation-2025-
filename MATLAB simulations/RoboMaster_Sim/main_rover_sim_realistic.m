function main_rover_sim_realistic()
% MAIN_ROVER_SIM_REALISTIC  2D rover simulation with 8-state EKF
% Uses realistic sensor noise models and MATLAB built-in sensor simulation

params = parameters_rm_realistic();  % New realistic parameters
dt = params.dt;
N  = round(params.T / dt);

% Ground truth state
x_true = params.x0_true;

% EKF
ekf = ekf8_init(params);

% ===== REALISTIC INITIALIZATION FOR ROVER EXPERIMENTATION =====
% For real rover experiments, we typically have good initial estimates
% from previous GPS fixes, compass calibration, and known starting conditions

% Get initial sensor measurements for proper initialization
u0 = [0;0;0];  % No motion at start
meas0 = sensor_model_wheel_encoders(x_true, u0, params);

% Initialize EKF with realistic starting conditions
% Position: Use GPS if available, otherwise start with some uncertainty
if ~isempty(meas0.gps_pos) && all(isfinite(meas0.gps_pos))
    initial_pos = meas0.gps_pos;  % Use GPS position
    pos_uncertainty = 1.0;        % 1m uncertainty for GPS position
else
    initial_pos = [0; 0];         % Assume starting at origin
    pos_uncertainty = 5.0;        % 5m uncertainty if no GPS
end

% Heading: Use magnetometer if available, otherwise start with some uncertainty
if ~isempty(meas0.yaw_mag) && isfinite(meas0.yaw_mag)
    initial_heading = meas0.yaw_mag;  % Use magnetometer heading
    heading_uncertainty = 0.1;        % 0.1 rad uncertainty for magnetometer
else
    initial_heading = 0;              % Assume starting north
    heading_uncertainty = 0.5;        % 0.5 rad uncertainty if no magnetometer
end

% Velocity: Assume starting from rest (typical for rover experiments)
initial_velocity = [0; 0];
velocity_uncertainty = 0.1;  % 0.1 m/s uncertainty

% Biases: Unknown initially, start with zero but high uncertainty
initial_bias_accel = [0; 0];
bias_accel_uncertainty = 0.1;  % 0.1 m/s² uncertainty

initial_bias_gyro = 0;
bias_gyro_uncertainty = 0.01;  % 0.01 rad/s uncertainty

% Set initial EKF state
ekf.x = [
    initial_pos(1);           % x position
    initial_pos(2);           % y position
    initial_heading;          % heading
    initial_velocity(1);      % vx
    initial_velocity(2);      % vy
    initial_bias_accel(1);    % b_ax
    initial_bias_accel(2);    % b_ay
    initial_bias_gyro         % b_w
];

% Set realistic initial covariance based on sensor quality and initial conditions
ekf.P = diag([
    pos_uncertainty^2,        % x position variance
    pos_uncertainty^2,        % y position variance
    heading_uncertainty^2,    % heading variance
    velocity_uncertainty^2,   % vx variance
    velocity_uncertainty^2,   % vy variance
    bias_accel_uncertainty^2, % b_ax variance
    bias_accel_uncertainty^2, % b_ay variance
    bias_gyro_uncertainty^2   % b_w variance
]);

fprintf('EKF initialized with realistic starting conditions:\n');
fprintf('  - Position: [%.2f, %.2f] m ± %.1f m\n', initial_pos(1), initial_pos(2), pos_uncertainty);
fprintf('  - Heading: %.2f rad ± %.2f rad\n', initial_heading, heading_uncertainty);
fprintf('  - Velocity: [%.2f, %.2f] m/s ± %.1f m/s\n', initial_velocity(1), initial_velocity(2), velocity_uncertainty);
fprintf('  - Biases: Accel ± %.1f m/s², Gyro ± %.3f rad/s\n', bias_accel_uncertainty, bias_gyro_uncertainty);

% Optional: perform initial measurement updates if sensors available
% This helps reduce initial uncertainty but doesn't override our realistic starting conditions
if ~isempty(meas0.gps_pos) && all(isfinite(meas0.gps_pos))
    fprintf('  - Applying initial GPS position update...\n');
    ekf = ekf8_update_gps_pos(ekf, meas0.gps_pos);
end

if ~isempty(meas0.yaw_mag) && isfinite(meas0.yaw_mag)
    fprintf('  - Applying initial magnetometer heading update...\n');
    ekf = ekf8_update_yaw(ekf, meas0.yaw_mag);
end

if ~isempty(meas0.wheel_vel) && all(isfinite(meas0.wheel_vel))
    fprintf('  - Applying initial wheel velocity update...\n');
    ekf = ekf8_update_wheel_vel(ekf, meas0.wheel_vel);
end

fprintf('Initial EKF state and covariance set for realistic rover experimentation.\n\n');

% ===== POST-SMOOTHING SWITCH =====
% Set to true to enable offline spike suppression + zero-phase smoothing
ENABLE_POST_SMOOTHING = true;  % <-- ONE-LINE SWITCH: true=ON, false=OFF

% Logs
log.t = (0:N-1)' * dt;
log.x_true = zeros(N,8);
log.x_est  = zeros(N,8);
log.P_trace= zeros(N,1);
log.u_true = zeros(N,3);
log.omega_true = zeros(N,1);  % Add logging for true angular velocity
log.omega_est = zeros(N,1);

% Raw sensor data logging
log.accel_raw = zeros(N,2);    % Raw accelerometer measurements
log.gyro_raw = zeros(N,1);     % Raw gyroscope measurements
log.gps_pos_raw = zeros(N,2);  % Raw GPS position measurements
log.wheel_vel_raw = zeros(N,2);  % Raw wheel encoder velocity measurements (body frame)
log.yaw_raw = zeros(N,1);      % Raw magnetometer yaw measurements
log.sensor_available = zeros(N,4); % [GPS_pos, GPS_vel, Mag, IMU] availability flags
% Additional analytics fields
log.update_count = zeros(N,1);   % Number of updates per step

% Sensor schedules
gps_step = round(1/params.rates.gps_hz / dt);
mag_step = round(1/params.rates.mag_hz / dt);
if gps_step < 1, gps_step = 1; end
if mag_step < 1, mag_step = 1; end

% Timing diagnostics for real-time performance verification
fprintf('Simulation setup:\n');
fprintf('  - Simulation dt: %.3f s\n', dt);
fprintf('  - GPS rate: %d Hz (every %d steps)\n', params.rates.gps_hz, gps_step);
fprintf('  - Mag rate: %d Hz (every %d steps)\n', params.rates.mag_hz, mag_step);
fprintf('  - Total steps: %d\n', N);
fprintf('  - Expected duration: %.1f s\n', N*dt);
fprintf('Starting simulation with REALISTIC sensor noise models...\n');
fprintf('Motion profile: Extended orbital motion (300s) with complex multi-orbit patterns\n');
fprintf('Sensor data will be logged and visualized:\n');
fprintf('  - IMU: Accelerometer (100 Hz), Gyroscope (100 Hz)\n');
fprintf('  - GPS: Position & Velocity (5 Hz)\n');
fprintf('  - Magnetometer: Yaw (10 Hz)\n');
fprintf('  - All measurements use realistic noise distributions\n');

% Control profile for extended orbital motion with varying dynamics
% Multi-orbit pattern covering large distances for better EKF performance analysis
u_body = @(k) [ 
    % Forward acceleration - dynamic forward motion for extended orbital paths
    % Creates varying orbital speeds and distances
    1.2 + 0.3*sin(0.008*k) + 0.15*sin(0.025*k);  % Dynamic forward acceleration
    
    % Steering (lateral acceleration) - creates complex orbital patterns
    % Multiple orbital cycles with varying radii and directions
    % Creates figure-8 patterns, spiral orbits, and orbital transitions
    0.6*sin(0.012*k) + 0.4*sin(0.006*k) + 0.2*sin(0.018*k) + ...
    0.3*sin(0.003*k) .* (mod(k, 1000) < 500);  % Complex orbital steering
    
    % Base angular velocity - enhanced for more dynamic motion
    % Creates natural orbital dynamics with varying angular rates
    0.05*sin(0.01*k) + 0.02*sin(0.04*k)  % Dynamic base rotation for natural motion
];

for k = 1:N
    % True control (without bias)
    u_true = u_body(k);
    log.u_true(k,:) = u_true(:)';
    
    % Extract true angular velocity for logging
    omega_true = u_true(3);

    % Advance ground truth with bias random walks inside
    x_true = rover_dynamics(x_true, u_true, dt, params);

    % Simulate measurements with wheel encoders (replace GPS velocity)
    meas = sensor_model_wheel_encoders(x_true, u_true, params);
    
    % Log raw sensor data
    log.accel_raw(k,:) = meas.accel_body(:)';
    log.gyro_raw(k) = meas.gyro_z;
    log.sensor_available(k,4) = 1;  % IMU always available
    
    % GPS and wheel encoder availability
    if mod(k, gps_step) == 0
        log.gps_pos_raw(k,:) = meas.gps_pos(:)';
        log.wheel_vel_raw(k,:) = meas.wheel_vel(:)';
        log.sensor_available(k,1:2) = 1; % [GPS_pos, Wheel_vel]
    end
    if mod(k, mag_step) == 0
        log.yaw_raw(k) = meas.yaw_mag;
        log.sensor_available(k,3) = 1;
    end

    % EKF prediction with IMU
    ekf = ekf8_predict(ekf, dt, meas.accel_body, meas.gyro_z);

    % Constraint updates each step
    ekf = ekf8_apply_constraints(ekf);

    % GPS position + Wheel velocity updates (immediate when available)
    % Guard out the first second to avoid cold-start spikes, with validation
    if mod(k, gps_step) == 0 && (k*dt > 1.0)
        if all(isfinite(meas.gps_pos)) && all(abs(meas.gps_pos) < 1e4)
            ekf = ekf8_update_gps_pos(ekf, meas.gps_pos);
        end
        if all(isfinite(meas.wheel_vel)) && all(abs(meas.wheel_vel) < 100)
            ekf = ekf8_update_wheel_vel(ekf, meas.wheel_vel);
        end
    end

    % Magnetometer (yaw) updates (immediate when available)
    if mod(k, mag_step) == 0 && (k*dt > 1.0)
        if isfinite(meas.yaw_mag) && abs(meas.yaw_mag) < 2*pi
            ekf = ekf8_update_yaw(ekf, meas.yaw_mag);
        end
    end

    % CRITICAL: Ensure we're always using the most current estimate
    % This eliminates any potential lag from using stale estimates
    current_estimate = ekf.x;
    
    % Log the current-time estimate (x(k|k))
    log.x_true(k,:) = x_true(:)';
    log.x_est(k,:)  = current_estimate(:)';
    log.P_trace(k)  = trace(ekf.P);
    % Convert cumulative update_count to per-step count
    if k == 1
        log.update_count(k) = ekf.update_count;
    else
        log.update_count(k) = ekf.update_count - sum(log.update_count(1:k-1));
    end
    
    % Log true angular velocity for proper error calculation
    log.omega_true(k) = omega_true;
    
    % Estimated omega from gyro minus estimated bias
    log.omega_est(k) = meas.gyro_z - current_estimate(8);
end

% FINAL STEP: Eliminate one-cycle display lag by doing a final prediction
% This ensures the EKF estimate is truly current-time, not one step behind
fprintf('Final EKF state update to eliminate display lag...\n');
final_meas = sensor_model_wheel_encoders(x_true, u_true, params);
ekf = ekf8_predict(ekf, dt, final_meas.accel_body, final_meas.gyro_z);
ekf = ekf8_apply_constraints(ekf);

% Update the final estimate in logs to be truly current-time
log.x_est(end,:) = ekf.x(:)';
log.omega_est(end) = final_meas.gyro_z - ekf.x(8);

% Plots - Clean and consolidated
% (Disabled by request) XY Trajectory & States - Realistic Sensors
% figure('Name','XY Trajectory & States - Realistic Sensors');
% subplot(2,2,1); 
% plot(log.x_true(:,1), log.x_true(:,2), 'k-', 'LineWidth', 1.5); hold on;
% plot(log.x_est(:,1),  log.x_est(:,2),  'r--', 'LineWidth', 1.2);
% axis equal; grid on; xlabel('X [m]'); ylabel('Y [m]');
% legend('True','EKF'); title('Rover XY Trajectory');
%
% subplot(2,2,2); 
% plot(log.t, log.x_true(:,1), 'k', log.t, log.x_est(:,1), 'r--'); grid on; 
% ylabel('x [m]'); title('Position X vs Time');
%
% subplot(2,2,3); 
% plot(log.t, log.x_true(:,2), 'k', log.t, log.x_est(:,2), 'r--'); grid on; 
% ylabel('y [m]'); title('Position Y vs Time');
%
% subplot(2,2,4); 
% plot(log.t, wrapToPi_local(log.x_true(:,3)), 'k', log.t, wrapToPi_local(log.x_est(:,3)), 'r--'); grid on; 
% ylabel('\theta [rad]'); xlabel('t [s]'); title('Yaw vs Time');

% Raw sensor data visualization - Clean and consolidated
figure('Name','Raw Sensor Data vs Time - Realistic Sensors');
subplot(2,2,1); 
plot(log.t, log.accel_raw(:,1), 'b.', 'MarkerSize', 1); grid on; ylabel('a_x [m/s²]'); title('Raw Accelerometer X');
subplot(2,2,2); 
plot(log.t, log.accel_raw(:,2), 'b.', 'MarkerSize', 1); grid on; ylabel('a_y [m/s²]'); title('Raw Accelerometer Y');
subplot(2,2,3); 
plot(log.t, log.gyro_raw, 'b.', 'MarkerSize', 1); grid on; ylabel('\omega [rad/s]'); title('Raw Gyroscope');
subplot(2,2,4); 
plot(log.t, log.gps_pos_raw(:,1), 'g.', log.t, log.gps_pos_raw(:,2), 'g.', 'MarkerSize', 1); grid on; 
ylabel('Position [m]'); xlabel('t [s]'); title('Raw GPS Position (X & Y)'); legend('GPS X', 'GPS Y');

% Sensor vs. Estimated comparison - Clean and consolidated
figure('Name','Sensor vs. Estimated Comparison - Realistic Sensors');
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

% Post-smoothing analysis (if enabled) - MUST BE BEFORE ANIMATION
if ENABLE_POST_SMOOTHING
    fprintf('Applying offline spike suppression + zero-phase smoothing...\n');
    
    % Convert logs to format expected by post_smooth_estimates
    % Note: post_smooth_estimates expects 9xN, we have 8xN, so we'll adapt
    x_est_8state = log.x_est';  % 8xN matrix
    
    % Apply smoothing to 8-state vector
    x_est_smoothed = post_smooth_estimates_8state(log.t, x_est_8state);
    
    % Convert back to Nx8 for plotting
    log.x_est_smoothed = x_est_smoothed';
    
    fprintf('Post-smoothing complete. Proceeding to visualization...\n');
else
    fprintf('Post-smoothing disabled. Proceeding to visualization...\n');
end

% Animation: run to approximately 60 seconds of wall-clock time (proportional to extended simulation)
% Prepare sensor data for animation
sensor_data.gps_pos = log.gps_pos_raw;
sensor_data.gps_vel = log.wheel_vel_raw; % reuse field for animation overlay
sensor_data.yaw = log.yaw_raw;
sensor_data.available = log.sensor_available;

if ENABLE_POST_SMOOTHING
    % Show both raw and smoothed estimates in animation with sensor data
    animate_rover_xy(log.x_true(:,1:3), log.x_est(:,1:3), 60.0, true, log.x_est_smoothed(:,1:3), sensor_data);
else
    % Show only raw estimates with sensor data
    animate_rover_xy(log.x_true(:,1:3), log.x_est(:,1:3), 60.0, true, [], sensor_data);
end

% Error plots - Clean and consolidated
ex = log.x_true(:,1) - log.x_est(:,1);
ey = log.x_true(:,2) - log.x_est(:,2);
eth = arrayfun(@(a,b) wrapToPi_local(a-b), log.x_true(:,3), log.x_est(:,3));
evx = log.x_true(:,4) - log.x_est(:,4);
evy = log.x_true(:,5) - log.x_est(:,5);
ew  = log.u_true(:,3) - log.omega_est(:);

figure('Name','State Errors vs Time - RAW EKF (Realistic Sensors)');
subplot(2,2,1); plot(log.t, ex, 'b'); grid on; ylabel('x err [m]'); title('Position X Error');
subplot(2,2,2); plot(log.t, ey, 'b'); grid on; ylabel('y err [m]'); title('Position Y Error');
subplot(2,2,3); plot(log.t, eth, 'b'); grid on; ylabel('theta err [rad]'); title('Yaw Error');
subplot(2,2,4); plot(log.t, evx, 'b', log.t, evy, 'r'); grid on; ylabel('Velocity err [m/s]'); 
xlabel('t [s]'); title('Velocity Errors'); legend('v_x err', 'v_y err');

% Sensor noise analysis - Clean and consolidated
figure('Name','Sensor Noise Analysis - Realistic Sensors');
subplot(2,2,1); 
gps_noise_x = log.gps_pos_raw(:,1) - log.x_true(:,1);
gps_noise_x(~log.sensor_available(:,1)) = NaN;  % Only show when GPS available
plot(log.t, gps_noise_x, 'g.'); grid on; ylabel('GPS X noise [m]'); title('GPS Position X Noise');
subplot(2,2,2); 
gps_noise_y = log.gps_pos_raw(:,2) - log.x_true(:,2);
gps_noise_y(~log.sensor_available(:,1)) = NaN;
plot(log.t, gps_noise_y, 'g.'); grid on; ylabel('GPS Y noise [m]'); title('GPS Position Y Noise');
subplot(2,2,3); 
mag_noise = log.yaw_raw - log.x_true(:,3);
mag_noise(~log.sensor_available(:,3)) = NaN;  % Only show when magnetometer available
mag_noise = wrapToPi_local(mag_noise);
plot(log.t, mag_noise, 'm.'); grid on; ylabel('Mag noise [rad]'); title('Magnetometer Yaw Noise');
subplot(2,2,4); 
gyro_noise = log.gyro_raw - log.omega_true;  % Compare with true angular velocity, not control input
plot(log.t, gyro_noise, 'b.'); grid on; ylabel('Gyro noise [rad/s]'); title('Gyroscope Noise'); xlabel('t [s]');

% Additional post-smoothing error plots (if enabled)
if ENABLE_POST_SMOOTHING
    % Smoothed error plots
    ex_s = log.x_true(:,1) - log.x_est_smoothed(:,1);
    ey_s = log.x_true(:,2) - log.x_est_smoothed(:,2);
    eth_s = arrayfun(@(a,b) wrapToPi_local(a-b), log.x_true(:,3), log.x_est_smoothed(:,3));
    evx_s = log.x_true(:,4) - log.x_est_smoothed(:,4);
    evy_s = log.x_true(:,5) - log.x_est_smoothed(:,5);
    ew_s  = log.u_true(:,3) - (log.x_est_smoothed(:,8) - log.x_est(:,8) + log.omega_est(:));
    
    % Comparison plot: Raw vs Smoothed - Clean and consolidated
    figure('Name','Raw vs Smoothed EKF Comparison - Realistic Sensors');
    subplot(2,2,1); plot(log.t, ex, 'b', log.t, ex_s, 'r'); grid on; ylabel('x err [m]'); legend('Raw','Smoothed'); title('Position X Error');
    subplot(2,2,2); plot(log.t, ey, 'b', log.t, ey_s, 'r'); grid on; ylabel('y err [m]'); legend('Raw','Smoothed'); title('Position Y Error');
    subplot(2,2,3); plot(log.t, eth, 'b', log.t, eth_s, 'r'); grid on; ylabel('theta err [rad]'); legend('Raw','Smoothed'); title('Yaw Error');
    subplot(2,2,4); plot(log.t, evx, 'b', log.t, evx_s, 'r'); grid on; ylabel('v_x err [m/s]'); legend('Raw','Smoothed'); title('Velocity X Error'); xlabel('t [s]');
    
    fprintf('Post-smoothing comparison complete. Check the figure for Raw vs Smoothed EKF performance.\n');
end

% Performance summary with realistic sensor characteristics
% Compute errors for tuning logger
log.position_error = [log.x_true(:,1) - log.x_est(:,1), log.x_true(:,2) - log.x_est(:,2)];
log.velocity_error = [log.x_true(:,4) - log.x_est(:,4), log.x_true(:,5) - log.x_est(:,5)];
log.yaw_error = arrayfun(@(a,b) wrapToPi_local(a-b), log.x_true(:,3), log.x_est(:,3));
log.bias_error = [log.x_true(:,6) - log.x_est(:,6), log.x_true(:,7) - log.x_est(:,7), log.x_true(:,8) - log.x_est(:,8)];

% Fine-tuning analysis report
fprintf('\nRunning fine-tuning analysis (realistic sensors)...\n');
[tuning_report, recommendations] = fine_tuning_logger(log, params); %#ok<NASGU,ASGLU>

fprintf('\n=== REALISTIC SENSOR PERFORMANCE SUMMARY ===\n');
fprintf('GPS Position RMS Error: %.3f m (Typical: 2-5 m)\n', sqrt(mean(gps_noise_x.^2, 'omitnan')));
fprintf('Wheel Velocity RMS Error (X): %.3f m/s\n', sqrt(mean((log.wheel_vel_raw(:,1) - log.x_true(:,4)).^2, 'omitnan')));
fprintf('Magnetometer RMS Error: %.3f rad (Typical: 0.01-0.05 rad)\n', sqrt(mean(mag_noise.^2, 'omitnan')));
fprintf('Gyroscope RMS Error: %.3f rad/s (Typical: 0.001-0.01 rad/s)\n', sqrt(mean((log.gyro_raw - log.omega_true).^2)));
fprintf('=============================================\n');

end

function ang = wrapToPi_local(ang)
ang = mod(ang + pi, 2*pi) - pi;
end
