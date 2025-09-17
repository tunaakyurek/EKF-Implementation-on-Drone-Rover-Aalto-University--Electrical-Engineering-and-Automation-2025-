function buf = push_filter_frame(buf, x, P, x_pred, P_pred, F)
% push_filter_frame - Push one EKF frame into a fixed-size circular buffer
% Fields:
%  buf.x (9 x K), buf.P (9 x 9 x K), buf.x_pred (9 x K), buf.P_pred (9 x 9 x K), buf.F (9 x 9 x K)

buf.x      = circshift(buf.x,      [0, 1]); buf.x(:,1) = x;
buf.P      = circshift(buf.P,      [0, 0, 1]); buf.P(:,:,1) = P;
buf.x_pred = circshift(buf.x_pred, [0, 1]); buf.x_pred(:,1) = x_pred;
buf.P_pred = circshift(buf.P_pred, [0, 0, 1]); buf.P_pred(:,:,1) = P_pred;
buf.F      = circshift(buf.F,      [0, 0, 1]); buf.F(:,:,1) = F;

end


