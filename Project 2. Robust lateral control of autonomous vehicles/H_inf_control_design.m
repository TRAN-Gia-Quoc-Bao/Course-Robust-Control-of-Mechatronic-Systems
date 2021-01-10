function K = H_inf_control_design(A,B,E)
%
% The function:
%
%           K = H_inf_control_design(A,B,E)
% 
% uses the "Bounded-Real Lemma" to compute a state feedback 
% H-infinity control:
%
%   u(k) = -K*x(k) 
%
% for discrete-time linear systems: 
%
%   x(k+1) = A*x(k) + B*u(k) + E*w(k) 
%
% with w(k) a bounded input disturbance. This function 
% requieres CVX optimisation toolbox
%
% By John J. Martinez, Grenoble-INP, October 2019. 
%%

[n,p]=size(B); % n states and p controls
[n,m]=size(E); % m disturbances
Q=eye(n);

%%

cvx_begin sdp
    variable U(n,n) symmetric
    variable Y(n,p) 
    variable gamma2  % find: gamma^2
    minimize( gamma2 ) 
    
    subject to
       gamma2 >= 0;
       U >= 0;
                  [-U            zeros(n,m)        (U*A'-Y*B')     U;
                   zeros(m,n)   -gamma2*eye(m)     E'              zeros(m,n); 
                   (U*A'-Y*B')'  E                -U               zeros(n,n);
                   U             zeros(n,m)        zeros(n,n)      -inv(Q) ] <= 0;                                       
cvx_end

%H-inf state feedback gain:
K=(U\Y)';

end
