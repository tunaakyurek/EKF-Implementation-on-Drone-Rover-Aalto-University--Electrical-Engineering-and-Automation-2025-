function ekf = kalman_update(ekf, z, h, H, R, is_angle)
% KALMAN_UPDATE  Standard Kalman update with optional angle wrapping

z = z(:); h = h(:);

% Ensure R is matrix of correct size
if isscalar(R)
    R = R;
end

% Innovation
y = z - h;
if nargin >= 6 && is_angle
    y(1) = wrapToPi_local(y(1));
end

S = H * ekf.P * H' + R;
if rcond(S) < 1e-12
    return;
end
K = ekf.P * H' / S;

ekf.x = ekf.x + K * y;
ekf.x(3) = wrapToPi_local(ekf.x(3));

I_KH = eye(ekf.n) - K*H;
ekf.P = I_KH * ekf.P * I_KH' + K*R*K';
ekf.P = (ekf.P + ekf.P')/2;
ekf.update_count = ekf.update_count + 1;

end

function ang = wrapToPi_local(ang)
ang = mod(ang + pi, 2*pi) - pi;
end


