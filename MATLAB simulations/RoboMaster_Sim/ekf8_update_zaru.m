function ekf = ekf8_update_zaru(ekf, angular_rate_threshold)
omega_est = ekf.x(8); % using bias as proxy as in reference
if abs(omega_est) < angular_rate_threshold
    z = 0;
    [h, H, R, is_ang] = ekf8_formulary_zaru(ekf.x, ekf.r_zaru);
    ekf = kalman_update(ekf, z, h, H, R, is_ang);
end
end


