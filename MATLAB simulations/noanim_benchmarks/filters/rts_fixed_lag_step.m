function [x_smooth, P_smooth, buf] = rts_fixed_lag_step(buf)
% rts_fixed_lag_step - One-step fixed-lag RTS smoothing over EKF-9 history
% Inputs:
%  buf: struct with fields x, P, x_pred, P_pred, F (most-recent at index 1)
% Outputs:
%  x_smooth, P_smooth: smoothed estimate at the oldest buffered frame

K = size(buf.x,2);
if K < 2 || all(buf.x(:,K) == 0)
    % Not enough history yet; output most recent as placeholder
    x_smooth = buf.x(:,1);
    P_smooth = buf.P(:,:,1);
    return;
end

% Perform a single backward RTS from index 2 -> 1 for the tail frame K
i_curr = 2; i_next = 1; i_tail = K;
F_k = buf.F(:,:,i_curr);
P_k = buf.P(:,:,i_curr);
P_k_pred = buf.P_pred(:,:,i_curr);

% RTS gain
G = P_k * F_k' / max(eps, 1); % fallback; refine below if PD
S = P_k_pred;
if rcond(S) > 1e-9
    G = P_k * F_k' / S;
else
    G = P_k * F_k' / (S + 1e-9*eye(9));
end

xk = buf.x(:,i_curr);
xp = buf.x_pred(:,i_curr);
x_next = buf.x(:,i_next);

% Angle-aware state delta for Euler angles (wrap residuals)
dx = x_next - xp;
for idx = 7:9
    dx(idx) = atan2(sin(x_next(idx) - xp(idx)), cos(x_next(idx) - xp(idx)));
end

xk_s = xk + G * dx;
Pk_s = P_k + G * (buf.P(:,:,i_next) - P_k_pred) * G';

% Emit smoothed oldest frame and roll buffer tail forward
x_smooth = buf.x(:,i_tail);
P_smooth = buf.P(:,:,i_tail);

% Move smoothed result to tail to approximate fixed-lag output
buf.x(:,i_tail) = xk_s;
buf.P(:,:,i_tail) = Pk_s;
end


