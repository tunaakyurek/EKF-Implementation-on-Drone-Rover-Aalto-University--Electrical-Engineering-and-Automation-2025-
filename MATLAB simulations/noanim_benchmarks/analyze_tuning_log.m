function analyze_tuning_log()
% analyze_tuning_log - Summarize tuning_log.csv and suggest gate/noise tweaks
%
% Usage:
%   From repo root or this folder:
%     analyze_tuning_log

clc;
log_path = fullfile(fileparts(mfilename('fullpath')), 'tuning_log.csv');
gate_path = fullfile(fileparts(mfilename('fullpath')), 'tuning_gates.json');

if ~isfile(log_path)
    error('Log not found: %s', log_path);
end

T = readtable(log_path);

% Coerce logicals if they came in as text
if iscell(T.tilt_used_ekf9)
    T.tilt_used_ekf9 = strcmpi(T.tilt_used_ekf9, 'true');
end
if iscell(T.tilt_used_ukf9)
    T.tilt_used_ukf9 = strcmpi(T.tilt_used_ukf9, 'true');
end

N = height(T);

% Basic summaries
frac_tilt_ekf9 = mean(T.tilt_used_ekf9, 'omitnan');
frac_tilt_ukf9 = mean(T.tilt_used_ukf9, 'omitnan');
mean_abs_a_minus_g = mean(T.abs_a_minus_g, 'omitnan');
mean_gyro = mean(T.gyro_rate, 'omitnan');

mean_pos_err = mean(T.pos_err_norm, 'omitnan');
mean_roll_err = mean(abs(T.roll_err_deg), 'omitnan');
mean_pitch_err = mean(abs(T.pitch_err_deg), 'omitnan');
mean_yaw_err = mean(abs(T.yaw_err_deg), 'omitnan');

% Conditional summaries (EKF-9)
idx_tilt = T.tilt_used_ekf9 == true;
idx_notilt = T.tilt_used_ekf9 == false;

mean_pos_err_tilt = mean(T.pos_err_norm(idx_tilt), 'omitnan');
mean_pos_err_notilt = mean(T.pos_err_norm(idx_notilt), 'omitnan');

mean_roll_err_tilt = mean(abs(T.roll_err_deg(idx_tilt)), 'omitnan');
mean_roll_err_notilt = mean(abs(T.roll_err_deg(idx_notilt)), 'omitnan');
mean_pitch_err_tilt = mean(abs(T.pitch_err_deg(idx_tilt)), 'omitnan');
mean_pitch_err_notilt = mean(abs(T.pitch_err_deg(idx_notilt)), 'omitnan');

fprintf('=== Tuning Log Summary (%d samples) ===\n', N);
fprintf('Tilt used (EKF-9): %.1f%%   Tilt used (UKF-9): %.1f%%\n', 100*frac_tilt_ekf9, 100*frac_tilt_ukf9);
fprintf('|a|-|g| mean: %.2f m/s^2   gyro rate mean: %.1f deg/s\n', mean_abs_a_minus_g, rad2deg(mean_gyro));
fprintf('EKF-9 pos err mean: %.2f m\n', mean_pos_err);
fprintf('EKF-9 roll/pitch/yaw mean abs err: [%.1f  %.1f  %.1f] deg\n', mean_roll_err, mean_pitch_err, mean_yaw_err);
fprintf('EKF-9 pos err w/ tilt: %.2f m   w/o tilt: %.2f m\n', mean_pos_err_tilt, mean_pos_err_notilt);
fprintf('EKF-9 roll err w/ tilt: %.1f deg   w/o tilt: %.1f deg\n', mean_roll_err_tilt, mean_roll_err_notilt);
fprintf('EKF-9 pitch err w/ tilt: %.1f deg   w/o tilt: %.1f deg\n', mean_pitch_err_tilt, mean_pitch_err_notilt);

% Read gates if present
gates = struct('tilt_accel_tol', 3.0, 'tilt_rate_gate_deg_s', 60.0, 'tilt_R_deg', 3.0);
try
    if isfile(gate_path)
        raw = fileread(gate_path);
        gates = jsondecode(raw);
    end
catch
    % ignore json parse errors; use defaults
end

% Recommendation logic (simple heuristics)
rec = struct('tilt_accel_tol', gates.tilt_accel_tol, 'tilt_rate_gate_deg_s', gates.tilt_rate_gate_deg_s, 'tilt_R_deg', gates.tilt_R_deg, ...
             'R_mag_scale', 1.0, 'Q_att_scale', 1.0);

high_att_err = (mean_roll_err + mean_pitch_err)/2 > 5.0; % deg

if frac_tilt_ekf9 < 0.3 && high_att_err
    rec.tilt_accel_tol = min(gates.tilt_accel_tol + 1.0, gates.tilt_accel_tol + 2.0);
    rec.tilt_rate_gate_deg_s = min(gates.tilt_rate_gate_deg_s + 20.0, gates.tilt_rate_gate_deg_s + 30.0);
elseif frac_tilt_ekf9 > 0.8 && (mean_abs_a_minus_g > 1.5 || rad2deg(mean_gyro) > gates.tilt_rate_gate_deg_s)
    rec.tilt_accel_tol = max(gates.tilt_accel_tol - 1.0, 1.0);
    rec.tilt_rate_gate_deg_s = max(gates.tilt_rate_gate_deg_s - 10.0, 30.0);
end

% If yaw error spikes, suggest downweighting mag
if mean_yaw_err > 8.0
    rec.R_mag_scale = 2.0; % 2x
end

% If attitude still noisy, increase Q_att
if high_att_err
    rec.Q_att_scale = 1.5;
end

fprintf('\n=== Recommendations ===\n');
fprintf('- tilt_accel_tol:  current %.2f  ->  recommend %.2f\n', gates.tilt_accel_tol, rec.tilt_accel_tol);
fprintf('- tilt_rate_gate: current %.1f deg/s  ->  recommend %.1f deg/s\n', gates.tilt_rate_gate_deg_s, rec.tilt_rate_gate_deg_s);
fprintf('- tilt_R_deg:     current %.1f  ->  keep or adjust to %.1f if tilt too aggressive\n', gates.tilt_R_deg, max(2.0, gates.tilt_R_deg));
fprintf('- R_mag scale:    x%.1f (apply to params.R_mag)\n', rec.R_mag_scale);
fprintf('- Q_att scale:    x%.1f (apply to params.Q(7:9,7:9))\n', rec.Q_att_scale);

% Save a small recommendations JSON next to the log
try
    rec_path = fullfile(fileparts(mfilename('fullpath')), 'tuning_recommendations.json');
    fid = fopen(rec_path, 'w');
    if fid>0
        fprintf(fid, '{"tilt_accel_tol": %.6f, "tilt_rate_gate_deg_s": %.6f, "tilt_R_deg": %.6f, "R_mag_scale": %.2f, "Q_att_scale": %.2f}', ...
            rec.tilt_accel_tol, rec.tilt_rate_gate_deg_s, gates.tilt_R_deg, rec.R_mag_scale, rec.Q_att_scale);
        fclose(fid);
        fprintf('\nWrote recommendations: %s\n', rec_path);
    end
catch ME
    fprintf('Failed to write recommendations: %s\n', ME.message);
end

end


