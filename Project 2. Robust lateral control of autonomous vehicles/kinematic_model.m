% Kinematic model (equation 1 in the question file)
function [x_dot, y_dot, x, y] = kinematic_model(phi, vx, ts)
x_dot = vx*cos(phi);
y_dot = vx*sin(phi);
x = cumtrapz(ts, x_dot);
y = cumtrapz(ts, y_dot);
end