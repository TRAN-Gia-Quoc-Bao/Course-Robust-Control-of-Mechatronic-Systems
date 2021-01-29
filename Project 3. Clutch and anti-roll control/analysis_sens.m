%% Plot of the sensitivity functions for a SISO system
% G: plant model (ss or tf) 
% K: controller (ss or tf)
% O.Sename 2020

%% Determination of the sensitivity fucntions
L=series(G,K) % Loop transfer function L=GK
S=inv(1+L);   % S= 1/(1+L)
T= feedback(L,1)
SG=S*G;
KS=K*S;
%% stability analysis
poleS=pole(S);
poleT=pole(T);
poleSG=pole(SG);
poleKS=pole(KS);
if (sum(poleS>0) & sum(poleT>0) & sum(poleSG>0) & sum(poleKS>0))
    display('the closed-loop system is not stable')
else
    display('the closed-loop system is internally stable')
 end

%% Plot
figure
w=logspace(-5,5,1000); %% to be adjusted
subplot(2,2,1), bodemag(S,w), title('Sensitivity function')
subplot(2,2,2), bodemag(T,w),  title('Complementary sensitivity function')
subplot(2,2,3), bodemag(SG,w), title('Sensitivity*Plant')
subplot(2,2,4), bodemag(KS,w), title('Controller*Sensitivity')