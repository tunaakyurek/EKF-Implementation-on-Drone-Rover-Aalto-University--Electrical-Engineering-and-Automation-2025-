function analyze_sensor_fusion_performance(t, x_true_hist, x_est_hist, imu_hist, gps_hist, baro_hist, mag_hist)
% analyze_sensor_fusion_performance - Comprehensive sensor fusion analysis
% Based on MATLAB Simulink Autonomous Flight Design Tutorial methodology
%
% Analyzes the performance of the sensor fusion system including:
% - Individual sensor performance and characteristics
% - Sensor fusion effectiveness and accuracy
% - Cross-sensor validation and consistency
% - Temporal analysis of sensor contributions
% - Innovation analysis and filter health
%
% Inputs:
%   t - Time vector
%   x_true_hist - True state history [9 x N]
%   x_est_hist - Estimated state history [9 x N]
%   imu_hist - IMU measurement history [6 x N] [accel(3); gyro(3)]
%   gps_hist - GPS measurement history [3 x N]
%   baro_hist - Barometer measurement history [1 x N]
%   mag_hist - Magnetometer measurement history [1 x N]

fprintf('\n=== Sensor Fusion Performance Analysis ===\n');

%% COMPREHENSIVE SENSOR FUSION ANALYSIS FIGURE
figure('Name', 'Comprehensive Sensor Fusion Analysis', ...
       'Position', [100, 100, 1600, 1200]);

%% IMU ANALYSIS
subplot(4,3,1);
analyze_imu_performance(t, imu_hist, x_true_hist);

%% GPS ANALYSIS  
subplot(4,3,2);
analyze_gps_performance(t, gps_hist, x_true_hist);

%% BAROMETER ANALYSIS
subplot(4,3,3);
analyze_barometer_performance(t, baro_hist, x_true_hist);

%% MAGNETOMETER ANALYSIS
subplot(4,3,4);
analyze_magnetometer_performance(t, mag_hist, x_true_hist);

%% SENSOR AVAILABILITY
subplot(4,3,5);
analyze_sensor_availability(t, gps_hist, baro_hist, mag_hist);

%% FUSION ACCURACY
subplot(4,3,6);
analyze_fusion_accuracy(t, x_true_hist, x_est_hist);

%% INNOVATION ANALYSIS
subplot(4,3,7);
analyze_innovation_sequence(t, x_true_hist, x_est_hist, gps_hist);

%% CROSS-SENSOR VALIDATION
subplot(4,3,8);
analyze_cross_sensor_validation(t, gps_hist, baro_hist, x_true_hist);

%% FILTER CONVERGENCE
subplot(4,3,9);
analyze_filter_convergence(t, x_true_hist, x_est_hist);

%% VELOCITY ESTIMATION ERRORS
subplot(4,3,10);
analyze_velocity_estimation_errors(t, x_true_hist, x_est_hist);

%% ATTITUDE ESTIMATION ERRORS
subplot(4,3,11);
analyze_attitude_estimation_errors(t, x_true_hist, x_est_hist);

%% OVERALL PERFORMANCE SUMMARY
subplot(4,3,12);
create_performance_summary(t, x_true_hist, x_est_hist, gps_hist, baro_hist, mag_hist);

%% PRINT PERFORMANCE METRICS
print_sensor_fusion_metrics(t, x_true_hist, x_est_hist, gps_hist, baro_hist, mag_hist, imu_hist);

end

%% INDIVIDUAL SENSOR ANALYSIS FUNCTIONS

function analyze_imu_performance(t, imu_hist, x_true_hist)
% Analyze IMU sensor performance

% Plot accelerometer data
accel_data = imu_hist(1:3, :);
accel_magnitude = sqrt(sum(accel_data.^2));

plot(t, accel_magnitude, 'b-', 'LineWidth', 1.5);
hold on;

% Expected gravity magnitude
expected_gravity = 9.81;
yline(expected_gravity, 'r--', 'g = 9.81 m/s²', 'LineWidth', 2);

grid on;
xlabel('Time (s)');
ylabel('Acceleration Magnitude (m/s²)');
title('IMU Accelerometer Performance', 'FontWeight', 'bold');

% Calculate and display statistics
mean_accel = mean(accel_magnitude);
std_accel = std(accel_magnitude);
text(0.6, 0.9, sprintf('Mean: %.2f m/s²\nStd: %.2f m/s²', mean_accel, std_accel), ...
     'Units', 'normalized', 'FontSize', 10, 'BackgroundColor', 'white');

end

function analyze_gps_performance(t, gps_hist, x_true_hist)
% Analyze GPS sensor performance

% Calculate GPS errors when available
gps_available = ~isnan(gps_hist(1,:));
gps_errors = nan(size(t));

for i = 1:length(t)
    if gps_available(i)
        gps_errors(i) = norm(gps_hist(1:3,i) - x_true_hist(1:3,i));
    end
end

% Plot GPS errors
valid_indices = ~isnan(gps_errors);
plot(t(valid_indices), gps_errors(valid_indices), 'go-', 'MarkerSize', 4, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('GPS Error (m)');
title('GPS Measurement Accuracy', 'FontWeight', 'bold');

% Calculate statistics
if any(valid_indices)
    mean_gps_error = nanmean(gps_errors);
    std_gps_error = nanstd(gps_errors);
    max_gps_error = nanmax(gps_errors);
    availability = sum(gps_available) / length(gps_available) * 100;
    
    text(0.6, 0.8, sprintf('Mean Error: %.2f m\nStd: %.2f m\nMax: %.2f m\nAvailability: %.1f%%', ...
         mean_gps_error, std_gps_error, max_gps_error, availability), ...
         'Units', 'normalized', 'FontSize', 9, 'BackgroundColor', 'white');
end

end

function analyze_barometer_performance(t, baro_hist, x_true_hist)
% Analyze barometer sensor performance

% Calculate barometer errors
baro_available = ~isnan(baro_hist);
baro_errors = nan(size(t));

for i = 1:length(t)
    if baro_available(i)
        true_altitude = -x_true_hist(3,i); % NED down -> altitude up
        measured_altitude = baro_hist(i);  % Baro already stored as altitude up
        baro_errors(i) = abs(measured_altitude - true_altitude);
    end
end

% Plot barometer errors
valid_indices = ~isnan(baro_errors);
if any(valid_indices)
    plot(t(valid_indices), baro_errors(valid_indices), 'mo-', 'MarkerSize', 3, 'LineWidth', 1.5);
    
    % Calculate statistics
    mean_baro_error = nanmean(baro_errors);
    std_baro_error = nanstd(baro_errors);
    availability = sum(baro_available) / length(baro_available) * 100;
    
    text(0.6, 0.8, sprintf('Mean Error: %.2f m\nStd: %.2f m\nAvailability: %.1f%%', ...
         mean_baro_error, std_baro_error, availability), ...
         'Units', 'normalized', 'FontSize', 9, 'BackgroundColor', 'white');
end

grid on;
xlabel('Time (s)');
ylabel('Barometer Error (m)');
title('Barometer Accuracy', 'FontWeight', 'bold');

end

function analyze_magnetometer_performance(t, mag_hist, x_true_hist)
% Analyze magnetometer sensor performance

% Calculate magnetometer errors (yaw angle errors)
mag_available = ~isnan(mag_hist);
mag_errors = nan(size(t));

for i = 1:length(t)
    if mag_available(i)
        true_yaw = x_true_hist(9,i);
        measured_yaw = mag_hist(i);
        
        % Calculate angular error with proper wrapping
        yaw_error = wrapToPi(measured_yaw - true_yaw);
        mag_errors(i) = abs(rad2deg(yaw_error));
    end
end

% Plot magnetometer errors
valid_indices = ~isnan(mag_errors);
if any(valid_indices)
    plot(t(valid_indices), mag_errors(valid_indices), 'co-', 'MarkerSize', 3, 'LineWidth', 1.5);
    
    % Calculate statistics
    mean_mag_error = nanmean(mag_errors);
    std_mag_error = nanstd(mag_errors);
    availability = sum(mag_available) / length(mag_available) * 100;
    
    text(0.6, 0.8, sprintf('Mean Error: %.1f°\nStd: %.1f°\nAvailability: %.1f%%', ...
         mean_mag_error, std_mag_error, availability), ...
         'Units', 'normalized', 'FontSize', 9, 'BackgroundColor', 'white');
end

grid on;
xlabel('Time (s)');
ylabel('Yaw Error (degrees)');
title('Magnetometer Accuracy', 'FontWeight', 'bold');

end

function analyze_sensor_availability(t, gps_hist, baro_hist, mag_hist)
% Analyze sensor availability over time

% Calculate availability
gps_available = double(~isnan(gps_hist(1,:)));
baro_available = double(~isnan(baro_hist));
mag_available = double(~isnan(mag_hist));

% Plot availability
plot(t, gps_available + 2, 'g-', 'LineWidth', 2); hold on;
plot(t, baro_available + 1, 'm-', 'LineWidth', 2);
plot(t, mag_available, 'c-', 'LineWidth', 2);

ylim([-0.5, 3.5]);
yticks([0, 1, 2, 3]);
yticklabels({'Mag', 'Baro', 'GPS', ''});
grid on;
xlabel('Time (s)');
ylabel('Sensor Availability');
title('Sensor Availability Timeline', 'FontWeight', 'bold');

% Calculate uptime statistics
gps_uptime = mean(gps_available) * 100;
baro_uptime = mean(baro_available) * 100;
mag_uptime = mean(mag_available) * 100;

text(0.6, 0.2, sprintf('GPS: %.1f%%\nBaro: %.1f%%\nMag: %.1f%%', ...
     gps_uptime, baro_uptime, mag_uptime), ...
     'Units', 'normalized', 'FontSize', 10, 'BackgroundColor', 'white');

end

function analyze_fusion_accuracy(t, x_true_hist, x_est_hist)
% Analyze overall sensor fusion accuracy

% Calculate position estimation errors
pos_errors = sqrt(sum((x_true_hist(1:3,:) - x_est_hist(1:3,:)).^2));

plot(t, pos_errors, 'r-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Position Error (m)');
title('Sensor Fusion Accuracy', 'FontWeight', 'bold');

% Calculate and display statistics
mean_error = mean(pos_errors);
std_error = std(pos_errors);
max_error = max(pos_errors);
rms_error = sqrt(mean(pos_errors.^2));

% Add statistical lines
hold on;
yline(mean_error, 'b--', sprintf('Mean: %.2fm', mean_error), 'LineWidth', 1.5);
yline(rms_error, 'g--', sprintf('RMS: %.2fm', rms_error), 'LineWidth', 1.5);

text(0.6, 0.8, sprintf('Mean: %.2f m\nRMS: %.2f m\nMax: %.2f m\nStd: %.2f m', ...
     mean_error, rms_error, max_error, std_error), ...
     'Units', 'normalized', 'FontSize', 9, 'BackgroundColor', 'white');

end

function analyze_innovation_sequence(t, x_true_hist, x_est_hist, gps_hist)
% Analyze innovation sequence for filter health

% Calculate GPS innovations (when GPS is available)
gps_available = ~isnan(gps_hist(1,:));
innovations = nan(size(t));

for i = 1:length(t)
    if gps_available(i)
        predicted_pos = x_est_hist(1:3,i);
        measured_pos = gps_hist(1:3,i);
        innovations(i) = norm(measured_pos - predicted_pos);
    end
end

% Plot innovations
valid_indices = ~isnan(innovations);
if any(valid_indices)
    plot(t(valid_indices), innovations(valid_indices), 'bo-', 'MarkerSize', 3, 'LineWidth', 1.5);
    
    % Calculate statistics
    mean_innovation = nanmean(innovations);
    std_innovation = nanstd(innovations);
    
    % Add statistical lines
    hold on;
    if ~isnan(mean_innovation)
        yline(mean_innovation, 'r--', sprintf('Mean: %.2fm', mean_innovation), 'LineWidth', 1.5);
    end
    
    text(0.6, 0.8, sprintf('Mean: %.2f m\nStd: %.2f m', mean_innovation, std_innovation), ...
         'Units', 'normalized', 'FontSize', 9, 'BackgroundColor', 'white');
end

grid on;
xlabel('Time (s)');
ylabel('GPS Innovation (m)');
title('Innovation Sequence Analysis', 'FontWeight', 'bold');

end

function analyze_cross_sensor_validation(t, gps_hist, baro_hist, x_true_hist)
% Analyze cross-sensor validation and consistency

% Compare GPS and barometer altitude measurements
gps_available = ~isnan(gps_hist(1,:));
baro_available = ~isnan(baro_hist);

altitude_differences = nan(size(t));

for i = 1:length(t)
    if gps_available(i) && baro_available(i)
        gps_altitude = -gps_hist(3,i);     % NED down -> altitude up
        baro_altitude = baro_hist(i);      % Baro already altitude up
        altitude_differences(i) = abs(gps_altitude - baro_altitude);
    end
end

% Plot altitude differences
valid_indices = ~isnan(altitude_differences);
if any(valid_indices)
    plot(t(valid_indices), altitude_differences(valid_indices), 'ko-', 'MarkerSize', 3, 'LineWidth', 1.5);
    
    % Calculate statistics
    mean_diff = nanmean(altitude_differences);
    std_diff = nanstd(altitude_differences);
    
    text(0.6, 0.8, sprintf('Mean Diff: %.2f m\nStd: %.2f m', mean_diff, std_diff), ...
         'Units', 'normalized', 'FontSize', 9, 'BackgroundColor', 'white');
end

grid on;
xlabel('Time (s)');
ylabel('GPS-Baro Altitude Diff (m)');
title('Cross-Sensor Validation', 'FontWeight', 'bold');

end

function analyze_filter_convergence(t, x_true_hist, x_est_hist)
% Analyze filter convergence characteristics

% Calculate convergence metrics
pos_errors = sqrt(sum((x_true_hist(1:3,:) - x_est_hist(1:3,:)).^2));

% Moving average for convergence trend
window_size = min(50, length(pos_errors));
if window_size > 1
    moving_avg = movmean(pos_errors, window_size);
    plot(t, moving_avg, 'b-', 'LineWidth', 2);
end

hold on;
plot(t, pos_errors, 'r-', 'LineWidth', 1);

grid on;
xlabel('Time (s)');
ylabel('Position Error (m)');
title('Filter Convergence', 'FontWeight', 'bold');

% Analyze convergence time
convergence_threshold = 2.0; % meters
convergence_achieved = pos_errors < convergence_threshold;
if any(convergence_achieved)
    convergence_time = t(find(convergence_achieved, 1));
    xline(convergence_time, 'g--', sprintf('Converged: %.1fs', convergence_time), 'LineWidth', 2);
end

legend('Moving Average', 'Instantaneous Error', 'Location', 'best');

end

function analyze_sensor_contributions(t, gps_hist, baro_hist, mag_hist)
% Analyze relative contributions of different sensors

% Calculate sensor update rates
gps_updates = sum(~isnan(gps_hist(1,:)));
baro_updates = sum(~isnan(baro_hist));
mag_updates = sum(~isnan(mag_hist));
total_time = t(end) - t(1);

gps_rate = gps_updates / total_time;
baro_rate = baro_updates / total_time;
mag_rate = mag_updates / total_time;

% Plot sensor update rates
sensor_rates = [gps_rate; baro_rate; mag_rate];
sensor_names = {'GPS', 'Baro', 'Mag'};

bar(sensor_rates);
set(gca, 'XTickLabel', sensor_names);
ylabel('Update Rate (Hz)');
title('Sensor Update Rates', 'FontWeight', 'bold');
grid on;

% Add rate values as text
for i = 1:length(sensor_rates)
    text(i, sensor_rates(i) + 0.5, sprintf('%.1f Hz', sensor_rates(i)), ...
         'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end

end

function analyze_noise_characteristics(t, imu_hist, gps_hist)
% Analyze noise characteristics of sensors

% IMU accelerometer noise analysis
if size(imu_hist, 1) >= 3
    accel_data = imu_hist(1:3, :);
    accel_magnitude = sqrt(sum(accel_data.^2));
    
    % Subtract mean to analyze noise
    accel_noise = accel_magnitude - mean(accel_magnitude);
    
    plot(t, accel_noise, 'b-', 'LineWidth', 1);
    hold on;
    
    % Calculate noise statistics
    accel_noise_std = std(accel_noise);
    yline(accel_noise_std, 'r--', sprintf('σ = %.3f', accel_noise_std), 'LineWidth', 2);
    yline(-accel_noise_std, 'r--', '', 'LineWidth', 2);
end

grid on;
xlabel('Time (s)');
ylabel('Accelerometer Noise (m/s²)');
title('IMU Noise Characteristics', 'FontWeight', 'bold');

end

function analyze_velocity_estimation_errors(t, x_true_hist, x_est_hist)
% Plot linear velocity estimation errors (N, E, D) separately

vel_err = x_true_hist(4:6,:) - x_est_hist(4:6,:);
plot(t, vel_err(1,:), 'r-', 'LineWidth', 1.5); hold on;
plot(t, vel_err(2,:), 'g-', 'LineWidth', 1.5);
plot(t, vel_err(3,:), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Velocity Error (m/s)');
title('Velocity Estimation Errors (N,E,D)', 'FontWeight', 'bold');
legend('V_N error','V_E error','V_D error','Location','best');

% Stats
txt = sprintf('Mean |err|: [%.2f %.2f %.2f] m/s', ...
    mean(abs(vel_err(1,:))), mean(abs(vel_err(2,:))), mean(abs(vel_err(3,:))));
text(0.6, 0.85, txt, 'Units','normalized','BackgroundColor','white');
end

function analyze_attitude_estimation_errors(t, x_true_hist, x_est_hist)
% Plot attitude (roll, pitch, yaw) errors in degrees

att_err = x_true_hist(7:9,:) - x_est_hist(7:9,:);
att_err(3,:) = atan2(sin(att_err(3,:)), cos(att_err(3,:))); % wrap yaw
att_err_deg = rad2deg(att_err);

plot(t, att_err_deg(1,:), 'r-', 'LineWidth', 1.5); hold on;
plot(t, att_err_deg(2,:), 'g-', 'LineWidth', 1.5);
plot(t, att_err_deg(3,:), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Angle Error (deg)');
title('Attitude Estimation Errors (Roll, Pitch, Yaw)', 'FontWeight', 'bold');
legend('Roll error','Pitch error','Yaw error','Location','best');

% Stats
txt = sprintf('Mean |err|: [%.1f %.1f %.1f] deg', ...
    mean(abs(att_err_deg(1,:))), mean(abs(att_err_deg(2,:))), mean(abs(att_err_deg(3,:))));
text(0.6, 0.85, txt, 'Units','normalized','BackgroundColor','white');
end

function create_performance_summary(t, x_true_hist, x_est_hist, gps_hist, baro_hist, mag_hist)
% Create overall performance summary

% Calculate key metrics
pos_errors = sqrt(sum((x_true_hist(1:3,:) - x_est_hist(1:3,:)).^2));
mean_error = mean(pos_errors);
max_error = max(pos_errors);
rms_error = sqrt(mean(pos_errors.^2));

% Sensor availability
gps_availability = sum(~isnan(gps_hist(1,:))) / length(gps_hist) * 100;
baro_availability = sum(~isnan(baro_hist)) / length(baro_hist) * 100;
mag_availability = sum(~isnan(mag_hist)) / length(mag_hist) * 100;

% Create summary table
metrics = [mean_error; max_error; rms_error; gps_availability; baro_availability; mag_availability];
metric_names = {'Mean Error (m)', 'Max Error (m)', 'RMS Error (m)', ...
                'GPS Avail (%)', 'Baro Avail (%)', 'Mag Avail (%)'};

bar(metrics);
set(gca, 'XTickLabel', metric_names, 'XTickLabelRotation', 45);
title('Performance Summary', 'FontWeight', 'bold');
ylabel('Value');
grid on;

% Add performance assessment
if mean_error < 1.0
    performance = 'EXCELLENT';
    color = 'green';
elseif mean_error < 2.0
    performance = 'GOOD';
    color = 'blue';
elseif mean_error < 5.0
    performance = 'ACCEPTABLE';
    color = 'orange';
else
    performance = 'POOR';
    color = 'red';
end

text(0.5, 0.9, ['Overall Performance: ' performance], ...
     'Units', 'normalized', 'FontSize', 12, 'FontWeight', 'bold', ...
     'HorizontalAlignment', 'center', 'Color', color);

end

function print_sensor_fusion_metrics(t, x_true_hist, x_est_hist, gps_hist, baro_hist, mag_hist, imu_hist)
% Print comprehensive sensor fusion performance metrics

fprintf('\n=== SENSOR FUSION PERFORMANCE METRICS ===\n');

% Overall performance
pos_errors = sqrt(sum((x_true_hist(1:3,:) - x_est_hist(1:3,:)).^2));
fprintf('\nOverall Estimation Performance:\n');
fprintf('  Mean Position Error: %.3f m\n', mean(pos_errors));
fprintf('  RMS Position Error:  %.3f m\n', sqrt(mean(pos_errors.^2)));
fprintf('  Max Position Error:  %.3f m\n', max(pos_errors));
fprintf('  Min Position Error:  %.3f m\n', min(pos_errors));
fprintf('  Std Position Error:  %.3f m\n', std(pos_errors));

% Sensor availability
fprintf('\nSensor Availability:\n');
gps_avail = sum(~isnan(gps_hist(1,:))) / length(gps_hist) * 100;
baro_avail = sum(~isnan(baro_hist)) / length(baro_hist) * 100;
mag_avail = sum(~isnan(mag_hist)) / length(mag_hist) * 100;
fprintf('  GPS Availability:    %.1f%%\n', gps_avail);
fprintf('  Barometer Availability: %.1f%%\n', baro_avail);
fprintf('  Magnetometer Availability: %.1f%%\n', mag_avail);

% GPS performance
if any(~isnan(gps_hist(1,:)))
    gps_available = ~isnan(gps_hist(1,:));
    gps_errors = nan(size(t));
    for i = 1:length(t)
        if gps_available(i)
            gps_errors(i) = norm(gps_hist(1:3,i) - x_true_hist(1:3,i));
        end
    end
    
    fprintf('\nGPS Performance:\n');
    fprintf('  Mean GPS Error:      %.3f m\n', nanmean(gps_errors));
    fprintf('  RMS GPS Error:       %.3f m\n', sqrt(nanmean(gps_errors.^2)));
    fprintf('  Max GPS Error:       %.3f m\n', nanmax(gps_errors));
    fprintf('  GPS Update Rate:     %.1f Hz\n', sum(gps_available) / (t(end) - t(1)));
end

% IMU performance
if size(imu_hist, 1) >= 6
    accel_data = imu_hist(1:3, :);
    gyro_data = imu_hist(4:6, :);
    
    fprintf('\nIMU Performance:\n');
    fprintf('  Accel Noise (std):   %.4f m/s²\n', std(sqrt(sum(accel_data.^2)) - 9.81));
    fprintf('  Gyro Noise (std):    %.4f rad/s\n', std(sqrt(sum(gyro_data.^2))));
    fprintf('  IMU Update Rate:     %.1f Hz\n', length(t) / (t(end) - t(1)));
end

% Filter convergence
convergence_threshold = 2.0; % meters
convergence_achieved = pos_errors < convergence_threshold;
if any(convergence_achieved)
    convergence_time = t(find(convergence_achieved, 1));
    fprintf('\nFilter Convergence:\n');
    fprintf('  Convergence Time:    %.2f s (< %.1fm error)\n', convergence_time, convergence_threshold);
    fprintf('  Convergence Rate:    %.2f%%\n', sum(convergence_achieved) / length(convergence_achieved) * 100);
end

% Performance assessment
if mean(pos_errors) < 1.0
    assessment = 'EXCELLENT';
elseif mean(pos_errors) < 2.0
    assessment = 'GOOD';
elseif mean(pos_errors) < 5.0
    assessment = 'ACCEPTABLE';
else
    assessment = 'NEEDS IMPROVEMENT';
end

fprintf('\nOverall Assessment: %s\n', assessment);
fprintf('======================================\n');

end