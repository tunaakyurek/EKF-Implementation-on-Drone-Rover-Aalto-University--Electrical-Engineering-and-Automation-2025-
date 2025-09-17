function ekf = ekf8_update_gps_vel(ekf, gps_vel)
z = gps_vel(:);
[h, H, R, is_ang] = ekf8_formulary_gps_vel(ekf.x, ekf.r_gps_vel);
ekf = kalman_update(ekf, z, h, H, R, is_ang);
ekf.last_gps_vel = gps_vel(:);
end


