function ekf = ekf8_apply_constraints(ekf)
ekf = ekf8_update_nhc(ekf, 0.5, 0.1);
ekf = ekf8_update_zupt(ekf, 0.1);
ekf = ekf8_update_zaru(ekf, 0.05);
if ~isempty(ekf.last_gps_vel)
    v = ekf.last_gps_vel;
    if norm(v) > 0.7
        yaw_course = atan2(v(2), v(1));
        ekf = ekf8_update_yaw(ekf, yaw_course);
    end
end
end


