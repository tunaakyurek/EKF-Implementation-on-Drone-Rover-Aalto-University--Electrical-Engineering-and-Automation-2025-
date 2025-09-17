function ekf = ekf8_update_zupt(ekf, speed_threshold)
vx = ekf.x(4); vy = ekf.x(5);
if hypot(vx, vy) < speed_threshold
    z = [0;0];
    [h, H, R, is_ang] = ekf8_formulary_zupt(ekf.x, ekf.r_zupt);
    ekf = kalman_update(ekf, z, h, H, R, is_ang);
end
end


