%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle 6DoF 4WS 4ID Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Version: v1.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date: 2022/11/01 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Model Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set Steps
syms Ts;
% Set Vehicle Parameters
syms Ms Iz Lf Lr Wf Wr Cf Cr Re;
% Set States & Inputs & Disturbances
syms x1 x2 x3 x4 x5 x6;
syms u1 u2 u3 u4 u5 u6;
% Set Dimensions
nX = 6;
nU = 6;
% Set Intermediate Variables
alpha_FL = (x5 + Lf*x6)/(x4 - 0.5*Wf*x6) - u5;
alpha_FR = (x5 + Lf*x6)/(x4 + 0.5*Wf*x6) - u5;
alpha_RL = (x5 - Lr*x6)/(x4 - 0.5*Wf*x6) - u6;
alpha_RR = (x5 - Lr*x6)/(x4 + 0.5*Wf*x6) - u6;
Fx_FL = (1/Re)*u1;  Fy_FL = Cf*alpha_FL;
Fx_FR = (1/Re)*u2;  Fy_FR = Cf*alpha_FR;
Fx_RL = (1/Re)*u3;  Fy_RL = Cr*alpha_RL;
Fx_RR = (1/Re)*u4;  Fy_RR = Cr*alpha_RR;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% get Vehicle Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% System Function
f1 = x4*cos(x3) - x5*sin(x3);
f2 = x4*sin(x3) + x5*cos(x3);
f3 = x6;
f4 = (1/Ms)*(Fx_FL*cos(u5) - Fy_FL*sin(u5) + Fx_FR*cos(u5) - Fy_FR*sin(u5) +...
             Fx_RL*cos(u6) - Fy_RL*sin(u6) + Fx_RR*cos(u6) - Fy_RR*sin(u6));
f5 = (1/Ms)*(Fx_FL*sin(u5) + Fy_FL*cos(u5) + Fx_FR*sin(u5) + Fy_FR*cos(u5) +...
             Fx_RL*sin(u6) + Fy_RL*cos(u6) + Fx_RR*sin(u6) + Fy_RR*cos(u6));
f6 = (1/Iz)*(-0.5*Wf*(Fx_FL*cos(u5) - Fy_FL*sin(u5)) + 0.5*Wf*(Fx_FR*cos(u5) - Fy_FR*sin(u5))...
             -0.5*Wr*(Fx_RL*cos(u6) - Fy_RL*sin(u6)) + 0.5*Wr*(Fx_RR*cos(u6) - Fy_RR*sin(u6))...
             +    Lf*(Fx_FL*sin(u5) + Fy_FL*cos(u5)) +     Lf*(Fx_FR*sin(u5) + Fy_FR*cos(u5))...
             -    Lr*(Fx_RL*sin(u6) + Fy_RL*cos(u6)) -     Lr*(Fx_RR*sin(u6) + Fy_RR*cos(u6)));
% System Linearization
Matrix_Ac = jacobian([f1,f2,f3,f4,f5,f6], [x1,x2,x3,x4,x5,x6]);
Matrix_Bc = jacobian([f1,f2,f3,f4,f5,f6], [u1,u2,u3,u4,u5,u6]);
% Matrix Discretization
Matrix_Ad = eye(nX) + Ts*Matrix_Ac;
Matrix_Bd =           Ts*Matrix_Bc;