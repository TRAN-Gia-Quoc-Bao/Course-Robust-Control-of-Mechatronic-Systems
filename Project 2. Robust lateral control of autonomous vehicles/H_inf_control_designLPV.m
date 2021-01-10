function [listK, listG, gamma2] = H_inf_control_designLPV(listSys, E)
%
% The function:
%
%           K = H_inf_control_designLPV(A,B,E)
% 
% uses the "Bounded-Real Lemma" to compute an LPV state feedback 
% H-infinity control:
%
%   u(k) = -K*x(k) 
%
% for discrete-time linear systems: 
%
%   x(k+1) = A*x(k) + B*u(k) + E*w(k) 
%
% with w(k) a bounded input disturbance. This function 
% requires CVX optimisation toolbox
%%
[n,m]=size(E); % m disturbances
Q=eye(n);

%%
% listK = {};
% listG = {};
% for i = 1 : size(listSys, 2)
%     A = listSys{i}.a;
%     B = listSys{i}.b;
% end
A1 = listSys{1}.a; B1 = listSys{1}.b; 
A2 = listSys{2}.a; B2 = listSys{2}.b; 
A3 = listSys{3}.a; B3 = listSys{3}.b; 
A4 = listSys{4}.a; B4 = listSys{4}.b; 
[n,p]=size(B1); % n states and p controls

%%

cvx_begin sdp
    variable U(n,n) symmetric
    variable Y1(n,p) 
    variable Y2(n,p) 
    variable Y3(n,p) 
    variable Y4(n,p) 
    variable gamma2  % find: gamma^2
    minimize( gamma2 ) 
    
    subject to
       gamma2 >= 0;
       U >= 0;
                                 [-U            zeros(n,m)        (U*A1'-Y1*B1')     U;
                   zeros(m,n)   -gamma2*eye(m)     E'              zeros(m,n); 
                   (U*A1'-Y1*B1')'  E                -U               zeros(n,n);
                   U             zeros(n,m)        zeros(n,n)      -inv(Q) ] <= 0;  
               
                                 [-U            zeros(n,m)        (U*A2'-Y2*B2')     U;
                   zeros(m,n)   -gamma2*eye(m)     E'              zeros(m,n); 
                   (U*A2'-Y2*B2')'  E                -U               zeros(n,n);
                   U             zeros(n,m)        zeros(n,n)      -inv(Q) ] <= 0; 
               
                                 [-U            zeros(n,m)        (U*A3'-Y3*B3')     U;
                   zeros(m,n)   -gamma2*eye(m)     E'              zeros(m,n); 
                   (U*A3'-Y3*B3')'  E                -U               zeros(n,n);
                   U             zeros(n,m)        zeros(n,n)      -inv(Q) ] <= 0; 
               
                                 [-U            zeros(n,m)        (U*A4'-Y4*B4')     U;
                   zeros(m,n)   -gamma2*eye(m)     E'              zeros(m,n); 
                   (U*A4'-Y4*B4')'  E                -U               zeros(n,n);
                   U             zeros(n,m)        zeros(n,n)      -inv(Q) ] <= 0; 
cvx_end

%H-inf state feedback gain:
K1=(U\Y1)'; G1 = inv(listSys{1}.c*inv(eye(2) - listSys{1}.a + listSys{1}.b*K1)*listSys{1}.b);
K2=(U\Y2)'; G2 = inv(listSys{2}.c*inv(eye(2) - listSys{2}.a + listSys{2}.b*K2)*listSys{2}.b);
K3=(U\Y3)'; G3 = inv(listSys{3}.c*inv(eye(2) - listSys{3}.a + listSys{3}.b*K3)*listSys{3}.b);
K4=(U\Y4)'; G4 = inv(listSys{4}.c*inv(eye(2) - listSys{4}.a + listSys{4}.b*K4)*listSys{4}.b);

listK = {K1, K2, K3, K4};
listG = {G1, G2, G3, G4};
end
