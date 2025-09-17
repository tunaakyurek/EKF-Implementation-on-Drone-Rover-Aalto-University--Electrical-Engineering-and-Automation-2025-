function ekf = ekf8_update_wheel_vel(ekf, wheel_vel_body)
% EKF8_UPDATE_WHEEL_VEL  Update using wheel encoder velocity measurements
% Input: wheel_vel_body = [vx_body; vy_body] in body frame
% Model the measurement directly in the body frame to avoid using the
% current state estimate to rotate the measurement.

% Current state components
theta = ekf.x(3);
vx_pred = ekf.x(4);
vy_pred = ekf.x(5);

% Rotation from global to body frame
c = cos(theta); s = sin(theta);
R_global_to_body = [c s; -s c];

% Measurement vector (body-frame velocity directly from wheel encoders)
z = wheel_vel_body;

% Measurement model: h(x) = R_global_to_body(theta) * [vx; vy]
h = R_global_to_body * [vx_pred; vy_pred];

% Measurement Jacobian: H = ∂h/∂x
H = zeros(2,8);

% Partials w.r.t. vx and vy are the rotation matrix itself
H(1,4) = R_global_to_body(1,1);  % ∂h1/∂vx
H(1,5) = R_global_to_body(1,2);  % ∂h1/∂vy
H(2,4) = R_global_to_body(2,1);  % ∂h2/∂vx
H(2,5) = R_global_to_body(2,2);  % ∂h2/∂vy

% Partial w.r.t. theta uses dR/dtheta * [vx; vy]
dR_dtheta = [-s c; -c -s];
H(:,3) = dR_dtheta * [vx_pred; vy_pred];

% Measurement noise covariance (wheel encoder noise)
R = ekf.r_wheel_encoder * eye(2);

% Not an angle measurement
is_angle = false;

% Perform Kalman update
ekf = kalman_update(ekf, z, h, H, R, is_angle);

% Store last wheel velocity for potential diagnostics
ekf.last_wheel_vel = wheel_vel_body;

end
