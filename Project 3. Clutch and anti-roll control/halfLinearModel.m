% Author: Charles Poussot-Vassal
% 
% Description
% Function that compute the half vehicle linear model (front or rear)
% 
% Input
%  ms       : suspensded mass
%  mus_rl   : unsprung mass (left)
%  mus_rr   : unsprung mass (right)
%  k_rl     : stiffness of the suspension (left)
%  k_rr     : stiffness of the suspension (right)
%  c_rl     : damping of the suspension (left)
%  c_rr     : damping of the suspension (right)
%  kt_rl    : stiffness of the tire (left)
%  kt_rr    : stiffness of the tire (right)
%  kb       : antiroll bar
%  t_r      : horizontal distance center of gravity - left tire
%  Ix       : roll inertia
%  actuator : gain on the control signal
% 
% Output
%  P : system state space
% 
%  x = [zus_fl zus_fl_p zus_fr zus_fr_p zs zs_p roll roll_p] 
%  u = [zr_fl zr_fr Fdz Mdx u_fl u_fr]
%  y = [zs_pp zs_p zs roll_pp roll zus_fl_p zus_fr_p zus_fl zus_fr zdef_fl_p zdef_fr_p zdef_fl zdef_fr]
%
% [P] = halfLinearModel(ms,mus_rl,mus_rr,k_rl,k_rr,c_rl,c_rr,kt_rl,kt_rr,kb,t_r,Ix,actuator)

function [P] = halfLinearModel(ms,mus_fl,mus_fr,k_fl,k_fr,c_fl,c_fr,kt_fl,kt_fr,kb,t_f,Ix,actuator)
%  x = [zus_fl zus_fl_p zus_fr zus_fr_p zs zs_p roll roll_p] 
%  u = [zr_fl zr_fr Fdz Mdx u_fl u_fr]
%  y = [zs_pp zs_p zs roll_pp roll zus_fl_p zus_fr_p zus_fl zus_fr zdef_fl_p zdef_fr_p zdef_fl zdef_fr]
A = [0 1 0 0 0 0 0 0;
    [-k_fl-kt_fl -c_fl 0 0 k_fl c_fl -k_fl*t_f-kb/2/t_f -c_fl*t_f]/mus_fl;
    0 0 0 1 0 0 0 0;
    [0 0 -k_fr-kt_fr -c_fr k_fr c_fr  k_fr*t_f+kb/2/t_f  c_fr*t_f]/mus_fr;
    0 0 0 0 0 1 0 0;
    [k_fl c_fl k_fr c_fr -k_fl-k_fr -c_fl-c_fr (k_fl*t_f+kb/2/t_f-k_fr*t_f-kb/2/t_f) c_fl*t_f-c_fr*t_f]/ms;
    0 0 0 0 0 0 0 1;
    [-k_fl*t_f -c_fl*t_f k_fr*t_f c_fr*t_f k_fl*t_f-k_fr*t_f c_fl*t_f-c_fr*t_f (-k_fl*t_f*t_f-kb/2-k_fr*t_f*t_f-kb/2) -c_fl*t_f*t_f-c_fr*t_f*t_f]/Ix];
B = [0 0 0 0 0 0;
    [kt_fl 0 0 0 actuator 0]/mus_fl;
    0 0 0 0 0 0;
    [0 kt_fr 0 0 0 actuator]/mus_fr;
    0 0 0 0 0 0;
    [0 0 -1 0 -actuator -actuator]/ms;
    0 0 0 0 0 0;
    [0 0 0 1 actuator*t_f -actuator*t_f]/Ix];
C = [A(6,:);
    0 0 0 0 0 1 0 0;
    0 0 0 0 1 0 0 0;
    A(8,:);
    0 0 0 0 0 0 1 0;
    0 1 0 0 0 0 0 0;
    0 0 0 1 0 0 0 0;
    1 0 0 0 0 0 0 0;
    0 0 1 0 0 0 0 0;
    0 -1 0 0 0 1 -t_f 0;
    0 0 0 -1 0 1 t_f 0;
    -1 0 0 0 1 0 0 -t_f;
    0 0 -1 0 1 0 0 t_f];
D = [B(6,:); 
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    B(8,:);
    0 0 0 0 0 0;
    zeros(8,6)];
P = ss(A,B,C,D);