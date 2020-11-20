%% Embedded control of an electric steering wheel

%% Default commands
close all;
clear all;
clc;

%% Parameters
J = 6.8*10^(-3);
Ke = 0.47;
R = 3.33;
c = 10^(-9);
Ts = 0.01;

%% System
A = [0 1 0; 0 -Ke^2/(R*J) 1/J; 0 0 -c];
B = [0; Ke/(R*J); 0];
C = [1 0 0];
D = 0;
sysC = ss(A, B, C, D);
sysD = c2d(sysC, Ts);

% With integral action
Ae = [A zeros(3, 1); -C 0];
Be = [B; 0];
Ce = [C 0];
De = 0;
sysE = ss(Ae, Be, Ce, De);
sysED = c2d(sysE, Ts);

%% Observer
W = sysD.b*sysD.b';
V = 0.01;
Ld = dlqr(sysD.a', sysD.c', W, V)';
Ao = sysD.a - Ld*sysD.c;

%% Control 1 - state-feedback only
% Q = sysD.c'*sysD.c;
Q = [1 0 0; 0 0 0; 0 0 10];
R = 0.01;
F = dlqr(sysD.a, sysD.b, Q, R);

% Simulate with referenceAngle = 1 & plot
time = [0 : Ts : 500*Ts]';
r = [ones(length(time), 1) zeros(length(time), 1) zeros(length(time), 1)];
%subplot(224);
lsim(ss(sysD.a - sysD.b*F, sysD.b*F, sysD.c, sysD.d, Ts), r, time);
legend('Q2 and R2', 'location', 'southeast');
set(gca, 'FontSize', 11);
grid on;

%% Control 2 - integral action
% Qe = sysED.c'*sysED.c; 
Qe = [1 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 10];
Re = 0.01;
Fe = dlqr(sysED.a, sysED.b, Qe, Re);

% Simulate with referenceAngle = 1 & plot
time = [0 : Ts : 500*Ts]';
r = [ones(length(time), 1) zeros(length(time), 1) zeros(length(time), 1) zeros(length(time), 1)];
%subplot(224);
lsim(ss(sysED.a - sysED.b*Fe, sysED.b*Fe + Ts*[zeros(3, 4); ones(1, 4)], sysED.c, sysED.d, Ts), r, time);
legend('Q2 and R2', 'location', 'southeast');
set(gca, 'FontSize', 11);
grid on;

%% Control 3 - electrical assistance
alpha = 5;
beta = 0.1;
FA = [0 beta*Ke alpha*R/Ke];
% Extended system
sysA = ss([sysD.a sysD.b*FA; Ld*sysD.c sysD.a-Ld*sysD.c+sysD.b*FA], zeros(6, 1), [eye(3) -eye(3); 0 0 alpha 0 -beta*Ke^2/R -alpha], zeros(4, 1), Ts, 'Statename', {'x1', 'x2', 'x3', 'xEst1', 'xEst2', 'xEst3'}, 'Inputname', {'none'}, 'Outputname', {'error1', 'error2', 'error3', 'errorAssistance'});
initial(sysA, [0; 0; 0; 1; 0; 0], 50*Ts);
grid on;