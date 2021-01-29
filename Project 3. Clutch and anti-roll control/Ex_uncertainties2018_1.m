%% Example on Hinf design and robust stability and performance analysis.
% Olivier Sename
% Nov 2018
clear all
close all
clc

%% System definition
m=0.2;k=20;k1=1;k2=0.1;r=0.15;b=0.25;R=2;Km=2;J=0.01;
A=[0 -1 r; 2*k/m 0 0;-2*k*r/J -Km*k1*k2/J/R -b/J];
B=[0;0;-1/J];
C=[0 1 0];
D=0;

G=ss(A,B,C,D)
pole(G)
zero(G)

%% Performance specification
Ms=2;wb=3;epsi=0.01;

W1=tf([1/Ms wb],[1 wb*epsi]);

Mu=3;wbc=20;epsi1=0.01;

W2=tf([1 wbc/Mu],[epsi1 wbc]);

Md=0.01;wd=0.1;epsi1=0.1;
Wd=tf([1 wd/Md],[epsi1 wd]);
%
systemnames = 'G W1 W2 Wd';
inputvar = '[ r(1); d;n;u(1)]';
outputvar = '[W1;W2; r-G-n]';
input_to_G = '[u+Wd]';
input_to_W1 = '[r-G-n]';
input_to_W2 = '[u]';
input_to_Wd = '[d]';
sysoutname = 'P';
cleanupsysic = 'yes';
sysic;

%%
% Find H-infinity optimal controller
nmeas=1; ncon=1;
[khinf,CL,gam,info] = hinfsyn(P,nmeas,ncon,'DISPLAY','ON')

%%
% Performance analysis
L=series(khinf,G)
S=inv(1+L);
poleS=pole(S);
T=feedback(L,1);
poleT=pole(T);
SG=S*G;
poleSG=pole(SG);
KS=khinf*S;
poleKS=pole(KS);
%%%%
w=logspace(-2,2,500);
figure
subplot(2,2,1), sigma(S,1/W1,w), title('Sensitivity function')
subplot(2,2,2), sigma(T,w),  title('Complementary sensitivity function')
subplot(2,2,3), sigma(SG,1/Wd,w), title('Sensitivity*Plant')
subplot(2,2,4), sigma(KS,1/W2,w), title('Controller*Sensitivity')


%% Uncertainty modelling
mu=ureal('m',m,'Percentage',10);
ku=ureal('k',k,'Percentage',10);
k1u=ureal('k1',k1,'Percentage',10);
k2u=ureal('k2',k2,'Percentage',10);
ru=ureal('r',r,'Percentage',10);
bu=ureal('b',b,'Percentage',10);
Ru=ureal('R',R,'Percentage',10);
Kmu=ureal('Km',Km,'Percentage',10);
Ju=ureal('J',J,'Percentage',10);


Au=[0 -1 ru; 2*ku/mu 0 0;-2*ku*ru/Ju -Kmu*k1u*k2u/Ju/Ru -bu/Ju];
Bu=[0;0;-1/Ju];
Cu=[0 1 0];
Du=0;
Gu=ss(Au,Bu,Cu,Du);

%%

figure
bodemag(Gu.NominalValue,'r-+',usample(G,25),'b');
legend('Nominal','Samples')

%% ANALYSIS OF ROBUST STABILITY FOR UNSTRUCTURED UNCERTAINTIES (Class formulation)

%%
% Compute the uncertainty weight
% Choice of uncertainty weighting ORDER 1
[P1,Info1] = ucover(usample(Gu,10),Gu.NominalValue,1,'OutputMult')
figure
sigma((Gu.NominalValue-usample(Gu,10))/Gu.NominalValue,'b--',Info1.W1,'r'); grid

%%
% Choice of uncertainty weighting ORDER 2
[P2,Info2] = ucover(usample(Gu,10),Gu.NominalValue,2,'OutputMult')
figure
sigma((Gu.NominalValue-usample(Gu,10))/Gu.NominalValue,'b--',Info2.W1,'r'); grid


%%
% Choice of uncertainty weighting ORDER 4
[P4,Info4] = ucover(usample(Gu,10),Gu.NominalValue,4,'OutputMult')
figure
sigma((Gu.NominalValue-usample(Gu,10))/Gu.NominalValue,'b--',Info4.W1,'r'); grid
%%

figure
sigma((Gu.NominalValue-usample(Gu,25))/Gu.NominalValue,'b--',Info1.W1,'b',Info2.W1,'r',Info4.W1,'g'); grid

%% 
% RS analysis
figure
sigma(T,'r',1/Info1.W1,1/Info2.W1,1/Info4.W1,w), title('Small Gain theorem')

%%  ANALYSIS OF ROBUST STABILITY FOR STRUCTURED UNCERTAINTIES 
% Robust Stability analysis using ROBSTAB tool in MATLAB
K=khinf;
systemnames = 'Gu K W1 W2 Wd';
inputvar = '[ r(1); d;n]';
outputvar = '[W1;W2]';
input_to_Gu = '[K+Wd]';
input_to_K = '[r-Gu-n]';
input_to_W1 = '[r-Gu-n]';
input_to_W2 = '[K]';
input_to_Wd = '[d]';
sysoutname = 'CL';
cleanupsysic = 'yes';
sysic;

%%
% check the nominal closed-loop system
pole(CL.NominalValue)
norm(CL.NominalValue,inf)

opts = robOptions('Display','on','Sensitivity','on','VaryFrequency','on');
[stabmarg,wcu,infors] = robstab(CL,opts)

infors.Sensitivity

stabmarg

[maxgain,wcu] = wcgain(CL);
maxgain

%pole(usubs(CL,destabunc))

%%
% Plot of the MU (Structured Singular Value) with robusstab
figure
semilogx(infors.Frequency,1./infors.Bounds)
xlabel('Frequency (rad/sec)');
ylabel('Mu upper/lower bounds');
title('Mu plot of robust stability margins ');


%% % ROBUST PERFORMANCE test with unstructured uncertianties.
w=logspace(-3,3,1000);
[sv1]=sigma(W1*S,w);
[sv2]=sigma(Info4.W1*T,w);
figure
semilogx(w,sv1+sv2)


%% ROBUST PERFORMANCE with STRCUTURED uncertainties
% First case using robustperf Matlab tool
opts = robOptions('Display','on','Sensitivity','on','VaryFrequency','on');
[perfmarg,wcu,inforp] = robgain(CL,1.2*gam,opts)

perfmarg
inforp.Sensitivity
% Plot of the MU (Structured Singular Value) with robusstab
figure
semilogx(inforp.Frequency,1./inforp.Bounds)
xlabel('Frequency (rad/sec)');
ylabel('Mu upper/lower bounds');
title('Mu plot of robust peerformance margins');

Smax = usubs(CL,wcu);
getPeakGain(Smax,1e-6)

%% Robust Performance analysis of a simpler case
% The choice of the important weighting functions to be considered for
% Robust Performance is crucial to avoid bad results: not to required too much for RP !!

%% 
% The new Hinf control scheme with a small number of weighting functions
% Here only the output performances are considered (tracking)
K=khinf;
systemnames = 'Gu K W1';
inputvar = '[ r]';
outputvar = '[W1]';
input_to_Gu = '[K]';
input_to_K = '[r-Gu]';
input_to_W1 = '[r-Gu]';
sysoutname = 'CLsimple';
cleanupsysic = 'yes';
sysic;

% plot of the uncertain and nominal sensitivity functions
figure,bodemag(CLsimple/W1,S,1/W1,'r')
figure,step(CLsimple/W1,S,'r')

norm(CLsimple.NominalValue,inf)

%% robust perf
perflevel=2;
opts = robOptions('Display','on','Sensitivity','on','VaryFrequency','on');
[perfmarg2,wcu,inforp2] = robgain(CLsimple,perflevel,opts)

perfmarg
inforp2.Sensitivity
% Plot of the MU (Structured Singular Value) with robusstab
figure
semilogx(inforp2.Frequency,1./inforp2.Bounds)
xlabel('Frequency (rad/sec)');
ylabel('Mu upper/lower bounds');
title('Mu plot of robust stability margins');

%%
%  Robust control desgin using DK iteration

systemnames = 'Gu W1';
inputvar = '[ r(1);d;u(1)]';
outputvar = '[W1;r-Gu]';
input_to_Gu = '[u+d]';
input_to_W1 = '[r-Gu]';
sysoutname = 'Pu';
cleanupsysic = 'yes';
sysic;

[k,clp,bnd] = dksyn(Pu,nmeas,ncon)
