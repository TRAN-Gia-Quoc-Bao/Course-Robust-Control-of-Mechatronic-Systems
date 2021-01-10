% Reference generator (equation 6 in the question file)
function [phi, phi_dot, time] = reference_generator(vx, rho, ts)
time = [0 : ts : (length(rho) - 1)*ts];
% Yaw rate
phi_dot = rho*vx;
% Yaw angle
phi = cumtrapz(ts, phi_dot);
end 