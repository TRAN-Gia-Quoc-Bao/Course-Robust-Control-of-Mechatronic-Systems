%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Specification of system parameters (half Megane)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Linear front parameters
ms     = 315*2;  % [kg]
mus_fl = 37.5;   % [kg]
mus_fr = 37.5;   % [kg]
k_fl   = 29500;  % [N/m]
k_fr   = 29500;  % [N/m]
c_fl   = 1500;   % [N/m/s]
c_fr   = 1500;   % [N/m/s]
kt_fl  = 208000; % [N/m]
kt_fr  = 208000; % [N/m]
kb     = 150000*0; % [N/m]
Ix     = 250;    % [kg*m^2]

%%% Geometry
l_r = 1.4;    % [m]
l_f = 1;      % [m]
t_f = 1.4/2;    % [m]  important (divided by 2)
t_r = 1.4/2;    % [m]
h   = 0.4;    % [m]
r   = 0.3;    % [m]

%%% Others
g      = 9.81; % gravity
Timage = 1/24; % for displaying

%%% Suspensions front characteristic
carkf = [-86.5 ,-3705;
         -74   ,1500-3705;
         0     ,0;
         17    ,4230-3705;
         33.5  ,4750-3705;
         46    ,5250-3705;
         51    ,5500-3705;
         58    ,7000-3705];

carcf = [-950 ,-627;
         -750 ,-525;
         -550 ,-415;
         -400 ,-337;
         -300 ,-282;
         -200 ,-239;
         -100 ,-177;
         -50  ,-134;
         0    ,0;
         50   ,137;
         100  ,374;
         200  ,715;
         300  ,842;
         400  ,925;
         550  ,1057;
         750  ,1251;
         950  ,1455];
  
Xk = carkf(:,1)./1000; %[m]
Xc = carcf(:,1)./1000; %[m/s]
Fk = carkf(:,2);       %[N]
Fc = carcf(:,2);       %[N]

kdefMax = max(Xk); % [m]
kdefMin = min(Xk); % [m]
cdefMax = max(Xc); % [m]
cdefMin = min(Xc); % [m]

cNonLinear = [];
kNonLinear = [];
for i = 1:size(Xc,1)
    if Xc(i) == 0
        temp = c_fl;
    else
        temp = Fc(i)/Xc(i);
    end
    cNonLinear = [cNonLinear temp/c_fl];
end

for i = 1:size(Xk,1)
    if Xk(i) == 0
        temp = k_fl;
    else
        temp = Fk(i)/Xk(i);
    end
    kNonLinear = [kNonLinear temp/k_fl];
end
