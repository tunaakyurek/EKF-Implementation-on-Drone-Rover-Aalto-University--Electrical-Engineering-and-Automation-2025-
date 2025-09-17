function R = rotation_matrix(phi, theta, psi)
% PURPOSE
% Compute ZYX (yaw-pitch-roll) body-to-world rotation matrix.
% Inputs are Euler angles: phi(roll), theta(pitch), psi(yaw) in radians.
%
% MAJOR STEPS
% 1) Guard against near-singularity when cos(theta) ~ 0
% 2) Build ZYX rotation and re-orthogonalize via SVD for numerical stability

    % 1) Safety check for numerical stability near pitch singularity
    if abs(cos(theta)) < 1e-6
        warning('Near-singularity detected in rotation matrix (cos(theta) near zero)');
        theta = sign(theta) * 1e-6; % small-angle replacement
    end
    
    % Precompute sines and cosines
    cph = cos(phi); sph = sin(phi);
    cth = cos(theta); sth = sin(theta);
    cps = cos(psi); sps = sin(psi);

    % 2) Construct R_z(psi) * R_y(theta) * R_x(phi)
    R = [
        cps*cth, cps*sth*sph - sps*cph, cps*sth*cph + sps*sph;
        sps*cth, sps*sth*sph + cps*cph, sps*sth*cph - cps*sph;
        -sth,    cth*sph,               cth*cph
    ];
    
    % Re-orthogonalize for numerical stability
    [U, ~, V] = svd(R);
    R = U * V';
end