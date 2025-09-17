function [h, H, R, is_angle] = ekf8_formulary_zaru(x, r_zaru)
% Zero angular-rate update on bias proxy (as in reference): b_w ~ 0
h = x(8);
H = zeros(1,8); H(8) = 1;
R = r_zaru;
is_angle = false;
end


