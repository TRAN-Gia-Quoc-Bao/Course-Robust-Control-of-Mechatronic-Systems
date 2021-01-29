%% Lab: Robust control of an anti-roll system

%% Default commands
close all; clear all; clc;

%% System and parameters (Megane car)
meganeParametersHalf;
actuator = 1000;
qhalf = halfLinearModel(ms,mus_fl,mus_fr,k_fl,k_fr,c_fl,c_fr,kt_fl,kt_fr,kb,t_f,Ix,actuator);

%% Templates
% Wn1  = .001;
% Wn2  = Wn1;
Wzr1 = 0.01;        % chosen from the bump height of 10mm
Wzr2 = Wzr1;
kzs =  4;               % chosen from
wzs = 5;                % chosen from
Wzs = kzs*tf(1, [1/(2*pi*wzs) 1]);
krol = 4;               % chosen from
wrol = 20;             % chosen from
Wrol = krol*tf(1, [1/(2*pi*wrol) 1]);
Wu1 = 0.14;         % chosen from
Wu2 = Wu1;
% Begin with the freq 
% We work with the inverse of the template

%% LTI H-inf control
systemnames = 'qhalf Wzr1 Wzr2 Wu1 Wu2 Wzs Wrol';
inputvar         = '[zr1; zr2; Fdz; Mdx; u1; u2]';
outputvar       = '[Wzs; Wrol; Wu1; Wu2; qhalf(12); qhalf(13)]';
input_to_qhalf = '[Wzr1; Wzr2; Fdz; Mdx; u1; u2]';
input_to_Wzr1 = '[zr1]';
input_to_Wzr2 = '[zr2]';
input_to_Wu1  = '[u1]';
input_to_Wu2  = '[u2]';
input_to_Wzs   = '[qhalf(3)]';
input_to_Wrol  = '[qhalf(5)]';
cleanupsysic = 'yes';
Pnom           = sysic;

% LTI H-inf controller
nmeas = 2;
ncon  = 2;
[hinfK, hinfCL, hinfGinf, info] = hinfsyn(Pnom, nmeas, ncon, 'DISPLAY', 'ON')
CLhinf = lft(qhalf, hinfK); % closed-loop system

%% Frequency response linear
w = logspace(-1, 2, 1000);

figure(1);
bodemag(qhalf(3, 1), 'y-', qhalf(3, 2), 'g--', hinfGinf/Wzs/Wzr1, 'k-.', CLhinf(3, 1), 'b', CLhinf(3, 2), 'r--', w);
z = title(['Frequency response to $z_s/z_r$ with $\gamma$ = ' num2str(hinfGinf)], 'Interpreter', 'latex');
set(z, 'FontSize', 24);
legend('$z_{rfl}$ to $z_s$ open loop', '$z_{rfr}$ to $z_s$ open loop', '$1/Wzs$', '$z_{rfl}$ to $z_s$ closed loop', '$z_{rfr}$ to $z_s$ closed loop', 'Interpreter', 'latex', 'Location', 'southwest');
set(findall(gcf, 'type', 'line'), 'linewidth', 3);
set(gca, 'FontSize', 16);
grid on;

figure(2);
bodemag(qhalf(5, 1), 'y-', qhalf(5, 2), 'g--', hinfGinf/Wrol/Wzr1, 'k-.', CLhinf(5, 1), 'b', CLhinf(5, 2),'r--', w);
z = title(['Frequency response to $\theta/z_r$ with $\gamma$ = ' num2str(hinfGinf)], 'Interpreter', 'latex');
set(z,'FontSize', 24);
legend('$z_{rfl}$ to $\theta$ open loop', '$z_{rfr}$ to $\theta$ open loop', '$1/Wroll$', '$z_{rfl}$ to $\theta$ closed loop', '$z_{rfr}$ to $\theta$ closed loop', 'Interpreter', 'latex', 'Location', 'southwest');
set(findall(gcf, 'type', 'line'), 'linewidth', 3);
set(gca, 'FontSize', 16);
grid on;

%% Simulation
out = sim('halfVehicle2020', 10);
%
figure(3); plot(output1.time, output1.signals(1).values, 'r', 'LineWidth', 2);
xlabel('Time (s)', 'Interpreter', 'latex'); ylabel('$z_s$ (m)', 'Interpreter', 'latex'); grid on; set(gca, 'FontSize', 28);

figure(4); plot(output2.time, output2.signals(1).values, 'r', 'LineWidth', 2);
xlabel('Time (s)', 'Interpreter', 'latex'); ylabel('$\theta$ (rad)', 'Interpreter', 'latex'); grid on; set(gca, 'FontSize', 28);

figure(5); plot(control.time, control.signals.values(:, 1), 'b', control.time, control.signals.values(:, 2), 'r', 'LineWidth', 2);
legend('$u_{fl}$', '$u_{fr}$', 'Interpreter', 'latex', 'Location', 'southeast');
xlabel('Time (s)', 'Interpreter', 'latex'); ylabel('Control (N)', 'Interpreter', 'latex'); grid on; set(gca, 'FontSize', 28);

figure(6);
plot(zdef.time, zdef.signals(1).values, 'b', zdef.time, zdef.signals(2).values, 'r', 'LineWidth', 2);
legend('$z_{def fl}$', '$z_{def fr}$', 'Interpreter', 'latex', 'Location', 'southeast');
xlabel('Time (s)', 'Interpreter', 'latex'); ylabel('Suspension deflection (m)', 'Interpreter', 'latex'); grid on; set(gca, 'FontSize', 28);