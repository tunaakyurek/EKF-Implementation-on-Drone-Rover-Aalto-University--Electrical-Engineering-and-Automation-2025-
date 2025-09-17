function ekf = ekf8_update_yaw(ekf, yaw_meas)
z = yaw_meas;
[h, H, R, is_ang] = ekf8_formulary_yaw(ekf.x, ekf.r_yaw);
ekf = kalman_update(ekf, z, h, H, R, is_ang);
end


