function [h, H, R, is_angle] = ekf8_formulary_yaw(x, r_yaw)
% Magnetometer yaw: z = theta (wrapped)
h = x(3);
H = zeros(1,8); H(3) = 1;
R = r_yaw;
is_angle = true;
end


