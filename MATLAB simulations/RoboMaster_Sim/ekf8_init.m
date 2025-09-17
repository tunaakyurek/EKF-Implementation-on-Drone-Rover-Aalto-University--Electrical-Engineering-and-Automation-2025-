function ekf = ekf8_init(params)
% EKF8_INIT  Initialize 8-state EKF for 2D rover

ekf.n = 8;
ekf.x = zeros(ekf.n,1);

% Initialize covariance - use P0 if provided, otherwise use default
if isfield(params, 'P0')
    ekf.P = params.P0;
else
    % Default initial covariance if P0 not provided
    ekf.P = eye(8);
    ekf.P(1:2,1:2) = ekf.P(1:2,1:2) * 5.0;   % position uncertainty
    ekf.P(3,3)     = 1.0;                      % yaw uncertainty
    ekf.P(4:5,4:5) = ekf.P(4:5,4:5) * 1.0;   % velocity uncertainty
    ekf.P(6:7,6:7) = ekf.P(6:7,6:7) * 0.5;   % accel bias uncertainty
    ekf.P(8,8)     = 0.1;                      % gyro bias uncertainty
end

% Process noise PSD
ekf.q_accel      = params.q_accel;
ekf.q_gyro       = params.q_gyro;
ekf.q_accel_bias = params.q_accel_bias;
ekf.q_gyro_bias  = params.q_gyro_bias;

% Measurement variances
ekf.r_accel   = params.r_accel;
ekf.r_gyro    = params.r_gyro;
ekf.r_gps_pos = params.r_gps_pos;
% Optional sensors (set defaults if not provided in params)
if isfield(params, 'r_wheel_encoder')
    ekf.r_wheel_encoder = params.r_wheel_encoder;  % Wheel encoder instead of GPS velocity
else
    ekf.r_wheel_encoder = 0.1; % reasonable default [(m/s)^2]
end
if isfield(params, 'r_gps_vel')
    ekf.r_gps_vel = params.r_gps_vel; % for sims that still use GPS velocity
else
    ekf.r_gps_vel = 0.5; % reasonable default [(m/s)^2]
end
ekf.r_yaw     = params.r_yaw;
ekf.r_nhc     = params.r_nhc;
ekf.r_zupt    = params.r_zupt;
ekf.r_zaru    = params.r_zaru;

% Stats and helpers
ekf.update_count = 0;
ekf.prediction_count = 0;
ekf.last_gps_vel = [];

end


