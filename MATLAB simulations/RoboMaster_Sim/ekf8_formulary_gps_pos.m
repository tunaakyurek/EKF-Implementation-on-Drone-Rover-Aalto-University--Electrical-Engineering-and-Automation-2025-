function [h, H, R, is_angle] = ekf8_formulary_gps_pos(x, r_gps_pos)
% GPS position: z = [x; y]
h = x(1:2);
H = zeros(2,8); H(1:2,1:2) = eye(2);
R = r_gps_pos * eye(2);
is_angle = false;
end


