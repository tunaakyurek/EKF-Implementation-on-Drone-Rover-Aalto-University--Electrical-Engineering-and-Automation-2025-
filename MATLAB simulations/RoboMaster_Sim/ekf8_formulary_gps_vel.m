function [h, H, R, is_angle] = ekf8_formulary_gps_vel(x, r_gps_vel)
% GPS velocity: z = [vx; vy]
h = x(4:5);
H = zeros(2,8); H(1:2,4:5) = eye(2);
R = r_gps_vel * eye(2);
is_angle = false;
end


