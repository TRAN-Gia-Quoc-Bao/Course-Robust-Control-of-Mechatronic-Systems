%% Lab: Robust control of a clutch flexible system

%% Default commands
close all; clear all; clc;

%% Parameters
k = 0.8; Jm = 4e-4; Jl = 4e-4; Bm = 0.015; Bl = 0; 
Kp = 0.3; Kd = 0.0033;

%% System
% G = tf(k, [Jm*Jl      (Jm*Bl+Jl*Bm)     (Jm*k+Jl*k+Bm*Bl)      (Bm*k+Bl*k)        0]);
A = [0 1 0 0; -k/Jl -Bl/Jl k/Jl 0; 0 0 0 1; k/Jm 0 -k/Jm -Bm/Jm];
B = [0; 0; 0; 1/Jm];
C = [1 0 0 0];
D = 0;
G = ss(A, B, C, D);
K = tf([Kd Kp], 1);

%% Performance analysis and specifications
L_PD = series(G, K);                   % Loop transfer function L = GK
S_PD = inv(1 + L_PD);                         % S = 1/(1+L)
poleS_PD = pole(S_PD);                        % result: stable
T_PD = feedback(L_PD, 1);                      % also T = 1 - S
poleT_PD = pole(T_PD);                        % result: stable
SG_PD = S_PD*G;
poleSG_PD = pole(SG_PD);                      % result: stable
KS_PD = K*S_PD;
poleKS_PD = pole(KS_PD);                      % result: stable (1 pole = 0)

% Stability analysis
if (sum(poleS_PD > 0) & sum(poleT_PD > 0) & sum(poleSG_PD > 0) & sum(poleKS_PD > 0))
    display('The closed-loop system is not stable')
else
    display('The closed-loop system is internally stable')
end
 
% Plot sensitivity functions
w = logspace(-3, 3, 500);               % to be adjusted
% subplot(221); sigma(S_PD, w); title('Sensitivity function'); grid on;
% subplot(222); sigma(T_PD, w); title('Complementary sensitivity function'); grid on;
% subplot(223); sigma(SG_PD, w); title('Sensitivity*Plant'); grid on;
% subplot(224); sigma(KS_PD, w); title('Controller*Sensitivity'); grid on;

% Analysis
% The closed-loop system is internally stable
% For Ms = max (S) = 5.6dB < 6dB so deltaM > 0.5, robustness good, for phase
% margin it's 0.7 < 3.5dB, good
% omegaT = 31.3 rad/s to tr = 2.3/omegaT = 0.0735s
% When omega = 0, we have from S a gain < -80dB so tracking steady state error lower than 1/1000
% For attenuation of input disturbance step, look at SG at freq 0 and it's
% < -90 dB so good attenuation
% Sensitivity of the control input to a measurement noise of sinusoıdal
% type at 800 rad/s: look at KS and it's 8.5dB, so it's NOT good
% Maximal controller gain: max of KS is almost 11dB > 3dB, NOT good

%% Design of templates
% Weights
% Ms = 2; wb = 5; epsi = 0.001; We = tf([1/Ms wb], [1 wb*epsi]);
Ms = 1.4; wb = 2.2; epsi = 0.001; We = tf([1/Ms wb], [1 wb*epsi]);
% Desired template for KS
% Mu = 1.4; eps = 0.01; wbc = 80; Wu = tf([1 wbc/Mu], [eps wbc]);
Mu = 1.2; eps = 0.01; wbc = 45; Wu = tf([1 wbc/Mu], [eps wbc]);
% sigma(1/Wu, w); title('Template on KS'); grid on;
Wn = tf([3 150], [1 2000]);

%% H-inf control
%% Test 1
systemnames = 'G We';
inputvar = '[r(1); d; u(1)]';
outputvar = '[We; r - G]';
input_to_G = '[u + d]';
input_to_We = '[r - G]';
sysoutname = 'P';
cleanupsysic = 'yes';
sysic;

nmeas = 1; ncon = 1;
[khinf, CL, gam, info] = hinfsyn(P, nmeas, ncon, 'DISPLAY', 'ON');

% Check again
S = inv(1 + G*khinf);
poleS = pole(S);
T = 1 - S;
poleT = pole(T);
SG = S*G;
poleSG = pole(SG);
KS = khinf*S;
poleKS=pole(KS);

figure(1);
subplot(221); sigma(S, 1/We, w); title('Sensitivity function S'); grid on;
subplot(222); sigma(T, w);  title('Complementary sensitivity function T'); grid on;
subplot(223); sigma(SG, w); title('Sensitivity*Plant SG'); grid on;
subplot(224); sigma(KS, w); title('Controller*Sensitivity KS'); grid on;

% Check settling time
figure(2); step(T); grid on;

%% Test 2
systemnames = 'G We Wu';
inputvar = '[ r(1); d; u(1)]';
outputvar = '[We; Wu; r-G]';
input_to_G = '[u+d]';
input_to_We = '[r-G]';
input_to_Wu = '[u]';
sysoutname = 'P';
cleanupsysic = 'yes';
sysic;

nmeas = 1; ncon = 1;
[khinf, CL, gam, info] = hinfsyn(P, nmeas, ncon, 'DISPLAY', 'ON');

% Check again
S = inv(1 + G*khinf);
poleS = pole(S);
T = 1 - S;
poleT = pole(T);
SG = S*G;
poleSG = pole(SG);
KS = khinf*S;
poleKS=pole(KS);

figure(3);
subplot(221); sigma(S, 1/We, w); title('Sensitivity function S'); grid on;
subplot(222); sigma(T, w);  title('Complementary sensitivity function T'); grid on;
subplot(223); sigma(SG, w); title('Sensitivity*Plant SG'); grid on;
subplot(224); sigma(KS, 'b', 1/Wu, 'r', w); title('Controller*Sensitivity KS'); grid on;

% Check settling time
figure(4); step(T); grid on;

%% Test 3
systemnames = 'G We Wu Wn';
inputvar = '[r(1); d; n; u(1)]';
outputvar = '[We; Wu; r-G-Wn]';
input_to_G = '[u+d]';
input_to_We = '[r-G-Wn]';
input_to_Wu = '[u]';
input_to_Wn= '[n]';
sysoutname = 'P';
cleanupsysic = 'yes';
sysic;

nmeas = 1; ncon = 1;
[khinf, CL, gam, info] = hinfsyn(P, nmeas, ncon, 'DISPLAY', 'ON');

% Check again
S = inv(1 + G*khinf);
poleS = pole(S);
T = 1 - S;
poleT = pole(T);
SG = S*G;
poleSG = pole(SG);
KS = khinf*S;
poleKS=pole(KS);

figure(5);
subplot(221); sigma(S, 1/We, w); title('Sensitivity function S'); grid on;
subplot(222); sigma(T, w);  title('Complementary sensitivity function T'); grid on;
subplot(223); sigma(SG, w); title('Sensitivity*Plant SG'); grid on;
subplot(224); sigma(KS, 'b', 1/Wu, 'r', w); title('Controller*Sensitivity KS'); grid on;

% Check settling time
figure(6); step(T); grid on;

%% Simulation
out = sim('ClutchControl', 5);
%
figure();
plot(out.output.time, out.output.signals(1).values, 'k', out.output.time, out.output.signals(2).values, 'b', out.output.time, out.output.signals(3).values, 'r', 'LineWidth', 2);
legend('Reference', 'PD', '$\mathcal{H}_{\infty}$', 'Interpreter', 'latex', 'Location', 'southeast');
xlabel('Time (s)', 'Interpreter', 'latex');
ylabel('$\theta_l$', 'Interpreter', 'latex');
grid on;
set(gca, 'FontSize', 24);

figure();
plot(out.control.time, out.control.signals(1).values, 'b', out.control.time, out.control.signals(2).values, 'r', 'LineWidth', 2);
legend('PD', '$\mathcal{H}_{\infty}$', 'Interpreter', 'latex', 'Location', 'southeast');
xlabel('Time (s)', 'Interpreter', 'latex');
ylabel('u', 'Interpreter', 'latex');
grid on;
set(gca, 'FontSize', 24);

%% Robustness study
% The closed-loop system is internally stable
% For Ms = max (S) = 5.95dB < 6dB so deltaM > 0.5, robustness good, for phase
% margin it's 1.96 < 3.5dB, good

ku       = ureal('ku', k, 'Percentage', 10);
Jmu    = ureal('Jmu', Jm, 'Percentage', 10);
Jlu      = ureal('Jlu', Jl, 'Percentage', 10);
Bmu   = ureal('Bmu', Bm, 'Percentage', 10);
% Blu     = ureal('Blu', Bl, 'Percentage', 10); % Percentage variations are not meaningful when the nominal value is zero

Au = [0 1 0 0; -ku/Jlu -Bl/Jlu ku/Jlu 0; 0 0 0 1; ku/Jmu 0 -ku/Jmu -Bmu/Jmu];
Bu = [0; 0; 0; 1/Jmu];
Cu = [1 0 0 0];
Du = 0;
Gu = ss(Au, Bu, Cu, Du);

% figure(); bodemag(Gu.NominalValue, 'r-+', usample(G, 25), 'b'); legend('Nominal', 'Samples'); grid on;
figure(); bodemag(Gu.NominalValue, 'r-+', usample(Gu, 3), 'b'); legend('Nominal', 'Samples'); grid on;

%% Frequency-domain representation of multiplicative unstructured uncertainties
% Choice of uncertainty weighting ORDER 1
[P1, Info1] = ucover(usample(Gu, 10), Gu.NominalValue, 1, 'OutputMult');
figure; sigma((Gu.NominalValue - usample(Gu, 10))/Gu.NominalValue, 'b--', Info1.W1, 'r', w); grid on;
% Choice of uncertainty weighting ORDER 2
[P2, Info2] = ucover(usample(Gu, 10), Gu.NominalValue, 2, 'OutputMult');
figure; sigma((Gu.NominalValue - usample(Gu, 10))/Gu.NominalValue, 'b--', Info2.W1, 'r', w); grid on;
% Choice of uncertainty weighting ORDER 4
[P4, Info4] = ucover(usample(Gu, 10), Gu.NominalValue, 4 ,'OutputMult');
figure; sigma((Gu.NominalValue - usample(Gu, 10))/Gu.NominalValue, 'b--', Info4.W1, 'r', w); grid on;

% We see that order 4 fits the best so we choose it

%% RS analysis
figure; sigma(T_PD, 'r--', T, 'r', 1/Info4.W1, 'b', w); title('Small gain theorem'); legend('T function of PD control', 'T function of H-inf control', '1/W order 4', 'Location', 'southwest'); grid on;