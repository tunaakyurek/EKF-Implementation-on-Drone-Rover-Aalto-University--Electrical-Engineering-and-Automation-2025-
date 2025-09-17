function [h, H, R, is_angle] = ekf8_formulary_zupt(x, r_zupt)
% Zero-velocity update: [vx; vy] ~ [0; 0]
h = [x(4); x(5)];
H = zeros(2,8); H(1,4) = 1; H(2,5) = 1;
R = r_zupt * eye(2);
is_angle = false;
end


