function [x_pred, F, Qd] = ekf8_formulary(x, dt, accel_body_meas, omega_meas, q_params)
% EKF8_FORMULARY  Canonical 8-DOF EKF prediction equations and matrices
% State: x = [ x; y; theta; vx; vy; b_ax; b_ay; b_w ]
% Inputs: accel_body_meas = [a_bx; a_by], omega_meas = w_z
% Discrete-time propagation with first-order hold (Euler) and INS-style Qd

% Unpack state
theta = x(3);
vx    = x(4); vy = x(5);
bax   = x(6); bay = x(7); bw = x(8);

% Rotation terms
c = cos(theta); s = sin(theta);

% True specific force in body (bias removed)
ax_b = accel_body_meas(1) - bax;
ay_b = accel_body_meas(2) - bay;

% Body→global acceleration (uses theta at start of step)
ax_g =  ax_b*c - ay_b*s;
ay_g =  ax_b*s + ay_b*c;

% Bias-compensated yaw-rate
omega_true = omega_meas - bw;

% ------------------------------
% Vector propagation x(k+1|k) = f(x,u)
% ------------------------------
% Position
x_pred = x;
x_pred(1) = x(1) + vx*dt;                 % x
x_pred(2) = x(2) + vy*dt;                 % y

% Heading
x_pred(3) = wrapToPi_local(theta + omega_true*dt);

% Velocity
x_pred(4) = vx + ax_g*dt;                 % vx
x_pred(5) = vy + ay_g*dt;                 % vy

% Bias random walks (mean remain the same; driven by Qd below)
% b_ax, b_ay, b_w remain unchanged in the mean model

% ------------------------------
% Discrete state-transition Jacobian F = ∂f/∂x
% ------------------------------
F = eye(8);

% dx/dvx, dy/dvy
F(1,4) = dt;                        % ∂x/∂vx
F(2,5) = dt;                        % ∂y/∂vy

% dtheta/dbw (theta = theta + (omega_meas - b_w) dt)
F(3,8) = -dt;                       % ∂theta/∂b_w

% Velocity dependence on theta and biases
% ax_g = (a_bx-b_ax)cos - (a_by-b_ay)sin
% ay_g = (a_bx-b_ax)sin + (a_by-b_ay)cos

% ∂vx/∂theta, ∂vy/∂theta
F(4,3) = dt * (-ax_b*s - ay_b*c);
F(5,3) = dt * ( ax_b*c - ay_b*s);

% ∂vx/∂b_ax, ∂vx/∂b_ay, ∂vy/∂b_ax, ∂vy/∂b_ay
F(4,6) = -dt * c;                   % ∂vx/∂b_ax
F(4,7) =  dt * s;                   % ∂vx/∂b_ay
F(5,6) = -dt * s;                   % ∂vy/∂b_ax
F(5,7) = -dt * c;                   % ∂vy/∂b_ay

% ------------------------------
% Discrete process noise Qd (INS-style with pos-vel coupling)
% ------------------------------
qa         = q_params.q_accel;       % accel white noise PSD
qg         = q_params.q_gyro;        % gyro white noise PSD driving yaw
qa_bias    = q_params.q_accel_bias;  % accel bias RW PSD
qg_bias    = q_params.q_gyro_bias;   % gyro bias RW PSD

dt2 = dt*dt; dt3 = dt2*dt;

Qd = zeros(8);

% x-axis (position-velocity driven by accel white noise)
Qd(1,1) = (dt3/3) * qa;             % pos x
Qd(1,4) = (dt2/2) * qa; Qd(4,1) = Qd(1,4);
Qd(4,4) = dt * qa;                  % vel x

% y-axis
Qd(2,2) = (dt3/3) * qa;             % pos y
Qd(2,5) = (dt2/2) * qa; Qd(5,2) = Qd(2,5);
Qd(5,5) = dt * qa;                  % vel y

% yaw (driven by gyro white noise)
Qd(3,3) = dt * qg;

% bias random walks
Qd(6,6) = dt * qa_bias;             % b_ax
Qd(7,7) = dt * qa_bias;             % b_ay
Qd(8,8) = dt * qg_bias;             % b_w

end

function ang = wrapToPi_local(ang)
ang = mod(ang + pi, 2*pi) - pi;
end


