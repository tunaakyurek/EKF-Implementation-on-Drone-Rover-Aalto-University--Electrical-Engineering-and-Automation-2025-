function meas = sensor_model_rm(x_true, u_true, params)
% SENSOR_MODEL_RM  Simulate IMU/GPS/Magnetometer measurements
% x_true: [x y theta vx vy b_ax b_ay b_w]
% u_true: [a_bx a_by omega] (true body accelerations and yaw-rate)

theta = x_true(3);

% IMU accelerometer (body frame) with bias and realistic noise
meas.accel_body = [
    u_true(1) + x_true(6) + sqrt(params.r_accel) * randn;
    u_true(2) + x_true(7) + sqrt(params.r_accel) * randn
];

% Gyro (yaw rate) with bias and realistic noise
meas.gyro_z = u_true(3) + x_true(8) + sqrt(params.r_gyro) * randn;

% GPS position (global) with realistic noise
meas.gps_pos = [
    x_true(1) + sqrt(params.r_gps_pos) * randn;
    x_true(2) + sqrt(params.r_gps_pos) * randn
];

% GPS velocity (global) with realistic noise
meas.gps_vel = [
    x_true(4) + sqrt(params.r_gps_vel) * randn;
    x_true(5) + sqrt(params.r_gps_vel) * randn
];

% Magnetometer (yaw) with realistic noise
meas.yaw_mag = wrapToPi_local(theta + sqrt(params.r_yaw) * randn);

end

function ang = wrapToPi_local(ang)
ang = mod(ang + pi, 2*pi) - pi;
end


