function [h, H, R, is_angle] = ekf8_formulary_nhc(x, r_nhc)
% Non-holonomic constraint: lateral body velocity ~ 0
theta = x(3); vx = x(4); vy = x(5);
h = -sin(theta)*vx + cos(theta)*vy;
H = zeros(1,8);
H(3) = -cos(theta)*vx - sin(theta)*vy; % ∂h/∂theta
H(4) = -sin(theta);                      % ∂h/∂vx
H(5) =  cos(theta);                      % ∂h/∂vy
R = r_nhc;
is_angle = false;
end


