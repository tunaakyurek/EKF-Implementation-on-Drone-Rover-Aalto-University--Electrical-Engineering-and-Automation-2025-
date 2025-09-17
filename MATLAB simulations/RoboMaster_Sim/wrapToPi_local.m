function ang = wrapToPi_local(ang)
% WRAPTOPI_LOCAL  Normalize angle to [-pi, pi]
ang = mod(ang + pi, 2*pi) - pi;
end


