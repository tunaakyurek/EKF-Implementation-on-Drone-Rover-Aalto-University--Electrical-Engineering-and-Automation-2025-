function ekf = ekf8_update_nhc(ekf, speed_threshold, yaw_rate_threshold)
vx = ekf.x(4); vy = ekf.x(5);
speed = hypot(vx, vy);

if speed > speed_threshold && abs(ekf.x(8)) < yaw_rate_threshold
    z = 0;
    [h, H, R, is_ang] = ekf8_formulary_nhc(ekf.x, ekf.r_nhc);
    % Only apply if theta observable
    if abs(H(3)) > 0.1
        ekf = kalman_update(ekf, z, h, H, R, is_ang);
    end
end

end


