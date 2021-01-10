%% Lab: Robust lateral control of autonomous vehicles

%% Default commands
close all; clear all; clc;

%% Parameters
Cf = 57117;     % Nrad^-1 
Cr = 81396;     % Nrad^-1
Iz = 1975;      % kg*m^2
m = 1621;       % kg
g = 9.8;        % m/s^2
Lf = 1.15;      % m
Lr = 1.38;      % m
% r = 0.20;       % m wheel radius
% h = 1.5;        % m distance between wheels (right-left)
ts = 2;

%% Inputs
vx = 10;        % vehicle longitudinal speed
% Road curvature (we design)
%rho = [0 0 0 0 0 0 0 0.001 0.002 0.003 0.004 0.005 0.006 0.007 0.008 0.009 0.010 0.011 0.012 0.013 0.014 0.015 0.016 0.017 0.018 0.019  0.02 0.02 0.02 0.02 0.02];
rho = [0 0 0 0 0 0 0 0.001 0.002 0.003 0.004 0.005 0.006 0.007 0.008 0.009 0.010 0.009 0.008 0.007 0.006 0.005 0.004 0.003 0.002 0.001 0 0 0 0 0 0 0];
% Trajectory
[phi, phi_dot, time] = reference_generator(vx, rho, ts);
% Centrifugal force
f_centrifugal = m*vx^2*rho;

% figure('Name', 'Trajectory');
% subplot(211); plot(time, phi_dot, 'LineWidth', 3); xlim([0 time(end)]); title('Yaw velocity'); xlabel('Time (s)'); ylabel('Yaw velocity (rad/s)'); grid on; set(gca,'FontSize', 14);
% subplot(212); plot(time, phi, 'LineWidth', 3); xlim([0 time(end)]); title('Yaw angle'); xlabel('Time (s)'); ylabel('Yaw angle (rad)'); grid on; set(gca,'FontSize', 14);

%% System modeling
% Kinematic model
[x_dot, y_dot, x, y] = kinematic_model(phi, vx, ts);
% Dynamic model
% Input = steering wheel angle, output = yaw rate
A = [-(Cf + Cr)/(m*vx)        -vx + (Cr*Lr - Cf*Lf)/(m*vx)
    (-Lf*Cf + Lr*Cr)/(Iz*vx)  -(Lf^2*Cf + Lr^2*Cr)/(Iz*vx)];
B = [Cf/m; Lf*Cf/Iz];
E = [0; 1/Iz];
C = [0 1];
D = [0];
sysDynamic = ss(A, B, C, D);

%% System kinematic response
% figure('Name', 'Speeds');
% subplot(211); plot(time, x_dot, 'LineWidth', 3); xlim([0 time(end)]); title('Longitudinal velocity'); xlabel('Time (s)'); ylabel('Longitudinal velocity (m/s)'); grid on; set(gca,'FontSize', 14);
% subplot(212); plot(time, y_dot, 'LineWidth', 3); xlim([0 time(end)]); title('Lateral velocity'); xlabel('Time (s)'); ylabel('Lateral velocity (m/s)'); grid on; set(gca,'FontSize', 14);
% 
% figure('Name', 'Position');
% plot(x, y, 'LineWidth', 3); title('Position'); xlabel('Longitudinal position (m)'); ylabel('Lateral position (m)'); grid on; set(gca,'FontSize', 14);
% 
% figure('Name', 'Centrifugal force');
% plot(time, f_centrifugal, 'LineWidth', 3); xlim([0 time(end)]); title('Centrifugal force'); xlabel('Time (s)'); ylabel('Centrifugal force (N)'); grid on; set(gca,'FontSize', 14);

%% System dynamics control
%% LQR + integral
Ae = [A zeros(2, 1); -C 0];
Be = [B; 0];
Ce = [C 0];
De = D;
sysE = ss(Ae, Be, Ce, De);
Qe = [1 0 0; 0 1 0; 0 0 10];
Re = 0.01;
Fe = lqr(sysE.a, sysE.b, Qe, Re);
sysLQR = ss(sysE.a - sysE.b*Fe, [0; 0; 1], sysE.c, sysE.d);

% Simulate LQR
% phi_dotLQR = lsim(sysLQR, phi_dot', time')';
% figure('Name', 'Yaw velocity control using LQR');
% plot(time, phi_dot, 'b', time, phi_dotLQR, '--r', 'LineWidth', 3); xlim([0 time(end)]); title('Yaw velocity control'); xlabel('Time (s)'); ylabel('Yaw velocity (rad/s)'); legend('Reference', 'Response'); grid on; set(gca,'FontSize', 14);
% % Integrate the kinematic model
% phiLQR = cumtrapz(ts, phi_dotLQR);
% [x_dotLQR, y_dotLQR, xLQR, yLQR] = kinematic_model(phiLQR, vx, ts);

% figure('Name', 'Position');
% plot(x, y, 'b', xLQR, yLQR, '--r', 'LineWidth', 3); title('Position'); xlabel('Longitudinal position (m)'); ylabel('Lateral position (m)'); legend('Reference', 'Response'); grid on; set(gca,'FontSize', 14);

% Observer
W = [1 0; 0 1];
V = 0.01;
L = lqr(A', C', W, V)';

%% Output-feedback H-inf control
% Template for error
Ms = 2; wb = 5; epsi = 0.001; We=tf([1/Ms wb], [1 wb*epsi]);
% Template for control
Mu = 2; wbc = 50; epsi1 = 0.1; Wu=tf([1 wbc/Mu], [epsi1 wbc]);
% New weights to enforce KS to satisfy the required template
Mu2 = 2; wbc2 = 10; epsi2 = 0.001; W2 = tf([1 wbc2/Mu2], [epsi2 wbc2]);
% Template for disturbance
Wd = 1;

% Generalized plant P is found with function sysic
systemnames = 'sysDynamic We Wu Wd';
inputvar = '[r(1); d; n; u(1)]';
outputvar = '[We; Wu; r - sysDynamic - n]';
input_to_sysDynamic = '[u + Wd]';
input_to_We = '[r - sysDynamic - n]';
input_to_Wu = '[u]';
input_to_Wd = '[d]';
sysoutname = 'sysP';
cleanupsysic = 'yes';
sysic;

% Find H-infinity optimal controller
nmeas = 1; ncon = 1;
[khinf, CL, gam, info] = hinfsyn(sysP, nmeas, ncon, 'DISPLAY', 'ON');

% Check freq-domain performance
S = inv(1 + sysDynamic*khinf);
poleS = pole(S);
T = 1 - S;
poleT = pole(T);
SG = S*sysDynamic;
poleSG = pole(SG);
KS = khinf*S;
poleKS = pole(KS);
%%%%
figure();
w = logspace(-2, 3, 500);
subplot(221); sigma(S, 1/We, w); title('Sensitivity function S'); grid on;
subplot(222); sigma(T, w);  title('Complementary sensitivity function T'); grid on;
subplot(223); sigma(SG, w); title('Sensitivity*Plant SG'); grid on;
subplot(224); sigma(KS, 'b', 1/Wu, 'r', 1/W2, 'g--', w); title('Controller*Sensitivity KS'); grid on;

%% State-feedback H-inf control
sysD = c2d(sysDynamic, 0.01);
Khinf = H_inf_control_design(sysD.a, sysD.b, 0.01*E);
Ghinf = inv(sysD.c*inv(eye(2)-sysD.a + sysD.b*Khinf)*sysD.b);

% Khinf = stateFeedbackHinf(A, B, [0.01; 0.01], C, 0, 0);
% Ghinf = inv(C*inv(-A + B*Khinf)*B);

% Real reference for simulation
ref = [time' phi_dot'];

%% LPV
% We have 2 varying parameters
% rho1 = vx, rho2 = 1/vx
vxmin = 5; vxmax = 15;
rho1Min = vxmin; rho1Max = vxmax;
rho2Min = 1/vxmax; rho2Max = 1/vxmin;
% Sys min min
A11 = [-(Cf + Cr)/(m*vxmax)        -vxmin + (Cr*Lr - Cf*Lf)/(m*vxmax)
    (-Lf*Cf + Lr*Cr)/(Iz*vxmax)     -(Lf^2*Cf + Lr^2*Cr)/(Iz*vxmax)];
sys11 = c2d(ss(A11, B, C, D), 0.01);
% Sys min max
A12 = [-(Cf + Cr)/(m*vxmin)        -vxmin + (Cr*Lr - Cf*Lf)/(m*vxmin)
    (-Lf*Cf + Lr*Cr)/(Iz*vxmin)     -(Lf^2*Cf + Lr^2*Cr)/(Iz*vxmin)];
sys12 = c2d(ss(A12, B, C, D), 0.01);
% Sys max min
A21 = [-(Cf + Cr)/(m*vxmax)        -vxmax + (Cr*Lr - Cf*Lf)/(m*vxmax)
    (-Lf*Cf + Lr*Cr)/(Iz*vxmax)     -(Lf^2*Cf + Lr^2*Cr)/(Iz*vxmax)];
sys21 = c2d(ss(A21, B, C, D), 0.01);
% Sys max max
A22 = [-(Cf + Cr)/(m*vxmin)        -vxmax + (Cr*Lr - Cf*Lf)/(m*vxmin)
    (-Lf*Cf + Lr*Cr)/(Iz*vxmin)     -(Lf^2*Cf + Lr^2*Cr)/(Iz*vxmin)];
sys22 = c2d(ss(A22, B, C, D), 0.01);

listSys = {sys11, sys12, sys21, sys22};
% Solve LMI for all the vertices
[listK, listG, gamma2] = H_inf_control_designLPV(listSys, 0.01*E);

%% Plot
% State-feedback H-inf + LQR comparison
out = sim('RobustLateral.slx');
figure('Name', 'H-inf');
plot(out.results.time, out.results.signals.values(:, 1), 'g', out.results.time, out.results.signals.values(:, 2), 'b', out.results.time, out.results.signals.values(:, 3), 'r', 'LineWidth', 2); xlim([0 out.results.time(end)]); title('State-feedback H_{\infty} vs. LQR integral control'); xlabel('Time (s)'); ylabel('\Psi dot (rad/s)'); legend('Reference', 'LQR integral control', 'State-feedback H-inf control'); grid on; set(gca, 'FontSize', 18);
figure('Name', 'control');
plot(out.control.time, out.control.signals(1).values, 'b', out.control.time, out.control.signals(2).values, 'r', 'LineWidth', 2); xlim([0 out.control.time(end)]); title('State-feedback H_{\infty} vs. LQR integral control'); xlabel('Time (s)'); ylabel('Control (rad)'); legend('LQR integral control', 'State-feedback H-inf control'); grid on; set(gca, 'FontSize', 18);

%% LPV
out = sim('RobustLateralLPV.slx');
figure('Name', 'LPV: ouput');
plot(out.lpvOutput.time, out.lpvOutput.signals.values(:, 1), 'b', out.lpvOutput.time, out.lpvOutput.signals.values(:, 2), 'r', 'LineWidth', 2); xlim([0 out.lpvOutput.time(end)]); title('LPV control: Output'); xlabel('Time (s)'); ylabel('\Psi dot (rad/s)'); legend('Reference', 'Response'); grid on; set(gca, 'FontSize', 18);
figure('Name', 'LPV: position');
subplot(211); plot(out.lpvPosition.time, out.lpvPosition.signals.values(:, 1), 'b', out.lpvPosition.time, out.lpvPosition.signals.values(:, 2), 'r', 'LineWidth', 2); xlim([0 out.lpvPosition.time(end)]); ylabel('x (m)'); legend('Reference', 'Response', 'Location', 'Southeast'); grid on; title('LPV control: Positions'); set(gca,'FontSize', 18);
subplot(212); plot(out.lpvPosition.time, out.lpvPosition.signals.values(:, 3), 'b', out.lpvPosition.time, out.lpvPosition.signals.values(:, 4), 'r', 'LineWidth', 2); xlim([0 out.lpvPosition.time(end)]); xlabel('Time (s)'); ylabel('y (m)'); legend('Reference', 'Response', 'Location', 'Southeast'); grid on; set(gca, 'FontSize', 18);
figure('Name', 'LPV: force');
plot(out.lpvForce.time, out.lpvForce.signals.values(:, 1), 'b', out.lpvForce.time, out.lpvForce.signals.values(:, 2), 'r', 'LineWidth', 2); xlim([0 out.lpvForce.time(end)]); title('LPV control: Centrifugal force'); xlabel('Time (s)'); ylabel('Force (N)'); legend('Reference', 'Response'); grid on; set(gca, 'FontSize', 18);
figure('Name', 'LPV: gains');
subplot(411); plot(out.lpvGains.time, out.lpvGains.signals.values(:, 1), 'b', 'LineWidth', 2); xlim([0 out.lpvGains.time(end)]); ylabel('\alpha_1'); grid on; set(gca, 'FontSize', 14);
subplot(412); plot(out.lpvGains.time, out.lpvGains.signals.values(:, 2), 'b', 'LineWidth', 2); xlim([0 out.lpvGains.time(end)]); ylabel('\alpha_2'); grid on; set(gca, 'FontSize', 14);
subplot(413); plot(out.lpvGains.time, out.lpvGains.signals.values(:, 3), 'b', 'LineWidth', 2); xlim([0 out.lpvGains.time(end)]); ylabel('\alpha_3'); grid on; set(gca, 'FontSize', 14);
subplot(414); plot(out.lpvGains.time, out.lpvGains.signals.values(:, 4), 'b', 'LineWidth', 2); xlim([0 out.lpvGains.time(end)]); ylabel('\alpha_4'); grid on; set(gca, 'FontSize', 14);
figure('Name', 'LPV: control');
plot(out.lpvControl.time, out.lpvControl.signals.values, 'b', 'LineWidth', 2); xlim([0 out.lpvControl.time(end)]); title('LPV control: Control input'); xlabel('Time (s)'); ylabel('Control (rad)'); grid on; set(gca, 'FontSize', 18);