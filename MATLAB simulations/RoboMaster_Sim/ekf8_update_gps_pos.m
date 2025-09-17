function ekf = ekf8_update_gps_pos(ekf, gps_pos)
z = gps_pos(:);
[h, H, R, is_ang] = ekf8_formulary_gps_pos(ekf.x, ekf.r_gps_pos);
ekf = kalman_update(ekf, z, h, H, R, is_ang);
end


