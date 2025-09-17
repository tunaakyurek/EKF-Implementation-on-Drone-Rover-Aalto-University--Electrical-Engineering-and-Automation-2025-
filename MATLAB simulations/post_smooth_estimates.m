function x_s = post_smooth_estimates(t, x)
% post_smooth_estimates - Offline spike suppression and zero-phase smoothing
% Inputs:
%   t : 1xN time vector
%   x : 9xN state history (EKF output)
% Output:
%   x_s : 9xN smoothed state history

% Parameters
win_sec_med = 0.12;      % median filter window (s) to kill isolated spikes
win_sec_sg  = 0.24;      % Savitzky-Golay window (s) for zero-phase smoothing
poly_order  = 3;         % SG polynomial

dt = mean(diff(t(:)));
if ~isfinite(dt) || dt <= 0
    dt = 0.01;
end
win_med = max(3, 2*floor(round(win_sec_med/dt)/2)+1); % odd
win_sg  = max(5, 2*floor(round(win_sec_sg/dt)/2)+1);  % odd

x_s = x;

% 1) Median filter per state to suppress spikes/outliers
for i = 1:size(x,1)
    xi = x(i,:);
    try
        x_med = medfilt1(xi, win_med, 'omitnan', 'truncate');
    catch
        x_med = xi; % fallback if Signal Processing Toolbox not available
    end
    x_s(i,:) = x_med;
end

% 2) Zero-phase smoothing (filtfilt) via SG or lowpass
try
    % Use Savitzky-Golay as a differentiator-preserving smoother
    for i = 1:size(x,1)
        xi = x_s(i,:);
        x_s(i,:) = sgolayfilt(xi, poly_order, win_sg);
    end
catch
    % Fallback: lowpass with zero-phase if available
    try
        fc = 4.0; % Hz
        [b,a] = butter(3, min(0.99, fc/(0.5/dt)));
        for i = 1:size(x,1)
            xi = x_s(i,:);
            x_s(i,:) = filtfilt(b,a,xi);
        end
    catch
        % Last resort: moving average
        w = win_sg;
        for i = 1:size(x,1)
            xi = x_s(i,:);
            x_s(i,:) = movmean(xi, w);
        end
    end
end

% 3) Wrap yaw back to [-pi,pi]
x_s(9,:) = atan2(sin(x_s(9,:)), cos(x_s(9,:)));

end


