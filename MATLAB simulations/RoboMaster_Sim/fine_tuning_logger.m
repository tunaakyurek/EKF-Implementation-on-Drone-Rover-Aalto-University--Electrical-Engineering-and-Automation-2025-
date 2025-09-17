function [tuning_report, recommendations] = fine_tuning_logger(log, params)
% FINE_TUNING_LOGGER  Comprehensive EKF performance analysis and tuning recommendations
% Inputs:
%   log: Simulation logging data structure
%   params: Simulation parameters
% Outputs:
%   tuning_report: Detailed performance analysis
%   recommendations: Specific tuning suggestions

fprintf('\n=== FINE-TUNING ANALYSIS REPORT ===\n');

% ===== PERFORMANCE METRICS =====
tuning_report = struct();

% Position errors
pos_x_rms = sqrt(mean(log.position_error(:,1).^2));
pos_y_rms = sqrt(mean(log.position_error(:,2).^2));
pos_x_max = max(abs(log.position_error(:,1)));
pos_y_max = max(abs(log.position_error(:,2)));

% Velocity errors
vel_x_rms = sqrt(mean(log.velocity_error(:,1).^2));
vel_y_rms = sqrt(mean(log.velocity_error(:,2).^2));
vel_x_max = max(abs(log.velocity_error(:,1)));
vel_y_max = max(abs(log.velocity_error(:,2)));

% Yaw errors
yaw_rms = sqrt(mean(log.yaw_error.^2));
yaw_max = max(abs(log.yaw_error));

% Bias errors
bias_ax_rms = sqrt(mean(log.bias_error(:,1).^2));
bias_ay_rms = sqrt(mean(log.bias_error(:,2).^2));
bias_w_rms = sqrt(mean(log.bias_error(:,3).^2));

% Store metrics
tuning_report.position = struct('x_rms', pos_x_rms, 'y_rms', pos_y_rms, ...
                               'x_max', pos_x_max, 'y_max', pos_y_max);
tuning_report.velocity = struct('x_rms', vel_x_rms, 'y_rms', vel_y_rms, ...
                               'x_max', vel_x_max, 'y_max', vel_y_max);
tuning_report.yaw = struct('rms', yaw_rms, 'max', yaw_max);
tuning_report.bias = struct('ax_rms', bias_ax_rms, 'ay_rms', bias_ay_rms, 'w_rms', bias_w_rms);

% ===== DRIFT ANALYSIS =====
% Check for systematic drift in position Y (common issue)
y_drift = polyfit(log.t, log.position_error(:,2), 1);
tuning_report.y_drift_rate = y_drift(1); % m/s drift rate

% Check for systematic bias in yaw
yaw_bias = mean(log.yaw_error);
tuning_report.yaw_bias = yaw_bias;

% ===== COVARIANCE STABILITY =====
P_trace_mean = mean(log.P_trace);
P_trace_std = std(log.P_trace);
P_trace_trend = polyfit(log.t, log.P_trace, 1);
tuning_report.covariance = struct('mean_trace', P_trace_mean, 'std_trace', P_trace_std, ...
                                 'drift_rate', P_trace_trend(1));

% ===== UPDATE FREQUENCY ANALYSIS =====
total_updates = sum(log.update_count);
avg_updates_per_step = total_updates / length(log.t);
tuning_report.updates = struct('total', total_updates, 'avg_per_step', avg_updates_per_step);

% ===== PERFORMANCE ASSESSMENT =====
fprintf('Position Performance:\n');
fprintf('  X: RMS=%.3fm, Max=%.3fm\n', pos_x_rms, pos_x_max);
fprintf('  Y: RMS=%.3fm, Max=%.3fm (Drift: %.3f m/s)\n', pos_y_rms, pos_y_max, y_drift(1));

fprintf('Velocity Performance:\n');
fprintf('  X: RMS=%.3fm/s, Max=%.3fm/s\n', vel_x_rms, vel_x_max);
fprintf('  Y: RMS=%.3fm/s, Max=%.3fm/s\n', vel_y_rms, vel_y_max);

fprintf('Yaw Performance:\n');
fprintf('  RMS=%.3frad, Max=%.3frad, Bias=%.3frad\n', yaw_rms, yaw_max, yaw_bias);

fprintf('Bias Estimation:\n');
fprintf('  Accel X: RMS=%.3fm/s²\n', bias_ax_rms);
fprintf('  Accel Y: RMS=%.3fm/s²\n', bias_ay_rms);
fprintf('  Gyro: RMS=%.3frad/s\n', bias_w_rms);

fprintf('EKF Stability:\n');
fprintf('  Covariance Trace: Mean=%.3f, Std=%.3f\n', P_trace_mean, P_trace_std);
fprintf('  Update Rate: %.2f updates/step\n', avg_updates_per_step);

% ===== TUNING RECOMMENDATIONS =====
recommendations = {};

% Position Y drift analysis
if abs(y_drift(1)) > 0.01  % More than 1cm/s drift
    if y_drift(1) > 0
        recommendations{end+1} = 'Y-position shows positive drift - consider increasing q_accel or reducing r_gps_pos';
    else
        recommendations{end+1} = 'Y-position shows negative drift - consider increasing q_accel or reducing r_gps_pos';
    end
end

% Yaw bias analysis
if abs(yaw_bias) > 0.05  % More than 0.05 rad bias
    if yaw_bias > 0
        recommendations{end+1} = 'Yaw shows positive bias - consider increasing q_gyro or reducing r_yaw';
    else
        recommendations{end+1} = 'Yaw shows negative bias - consider increasing q_gyro or reducing r_yaw';
    end
end

% Velocity oscillation analysis
if vel_x_rms > 0.5 || vel_y_rms > 0.5
    recommendations{end+1} = 'High velocity errors - consider increasing q_accel or reducing r_gps_pos';
end

% Covariance stability
if P_trace_std > P_trace_mean * 0.5
    recommendations{end+1} = 'Covariance shows high variability - consider adjusting process noise parameters';
end

if P_trace_trend(1) > 0.1
    recommendations{end+1} = 'Covariance shows upward trend - consider increasing measurement noise or reducing process noise';
end

% Specific parameter tuning suggestions
if pos_x_rms > 2.0
    recommendations{end+1} = 'High X-position error - consider increasing q_accel from current value';
end

if pos_y_rms > 2.0
    recommendations{end+1} = 'High Y-position error - consider increasing q_accel from current value';
end

if yaw_rms > 0.2
    recommendations{end+1} = 'High yaw error - consider increasing q_gyro from current value';
end

% ===== DISPLAY RECOMMENDATIONS =====
fprintf('\n=== TUNING RECOMMENDATIONS ===\n');
if isempty(recommendations)
    fprintf('No major tuning issues detected. Current parameters appear well-tuned.\n');
else
    for i = 1:length(recommendations)
        fprintf('%d. %s\n', i, recommendations{i});
    end
end

% ===== PARAMETER SUGGESTIONS =====
fprintf('\n=== PARAMETER TUNING SUGGESTIONS ===\n');

% Current parameter analysis
current_q_accel = params.q_accel;
current_q_gyro = params.q_gyro;
current_r_gps_pos = params.r_gps_pos;
current_r_yaw = params.r_yaw;

% Suggest adjustments based on performance
if pos_x_rms > 1.5 || pos_y_rms > 1.5
    suggested_q_accel = current_q_accel * 1.5;
    fprintf('Consider increasing q_accel from %.3f to %.3f\n', current_q_accel, suggested_q_accel);
end

if yaw_rms > 0.15
    suggested_q_gyro = current_q_gyro * 1.5;
    fprintf('Consider increasing q_gyro from %.3f to %.3f\n', current_q_gyro, suggested_q_gyro);
end

if pos_x_rms < 0.5 && pos_y_rms < 0.5
    suggested_r_gps_pos = current_r_gps_pos * 0.8;
    fprintf('Consider reducing r_gps_pos from %.3f to %.3f\n', current_r_gps_pos, suggested_r_gps_pos);
end

if yaw_rms < 0.1
    suggested_r_yaw = current_r_yaw * 0.8;
    fprintf('Consider reducing r_yaw from %.3f to %.3f\n', current_r_yaw, suggested_r_yaw);
end

fprintf('\n=== END OF FINE-TUNING REPORT ===\n');

end
