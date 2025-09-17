function ekf = ekf8_predict(ekf, dt, accel_body_meas, omega_meas)
% EKF8_PREDICT  Prediction using explicit 8-DOF formulary (matrices shown)

% Build Q parameters structure for formulary
q_params.q_accel      = ekf.q_accel;
q_params.q_gyro       = ekf.q_gyro;
q_params.q_accel_bias = ekf.q_accel_bias;
q_params.q_gyro_bias  = ekf.q_gyro_bias;

% Canonical propagation and matrices
[x_pred, F, Qd] = ekf8_formulary(ekf.x, dt, accel_body_meas, omega_meas, q_params);

% Commit state and covariance
ekf.x = x_pred;
ekf.P = F * ekf.P * F' + Qd;
ekf.P = (ekf.P + ekf.P')/2; % symmetrize

ekf.prediction_count = ekf.prediction_count + 1;

end


