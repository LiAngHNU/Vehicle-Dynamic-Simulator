%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle 2DoF Error Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Version: v1.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date: 2022/10/31 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Model Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set Steps
syms Ts
% Set Vehicle Parameters
syms Ms Iz Lf Lr Cf Cr Vx
% Set States & Inputs & Disturbances
syms x1 x2 x3 x4;
syms u1 u2;
syms v1
% Set Dimensions
nX = 4;
nU = 2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% get Vehicle Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get System Dynamics
f1 = x2;
f2 = 2*(Cf+Cr)/(Ms*Vx)*x2 - 2*(Cf+Cr)/(Ms)*x3 + 2*(Cf*Lf-Cr*Lr)/(Ms*Vx)*x4...
    -2*(Cf/Ms)*u1 - 2*(Cf/Ms)*u2 + 2*((Cf*Lf-Cr*Lr)/Ms - Vx*Vx)*v1;
f3 = x4;
f4 = 2*(Cf*Lf-Cr*Lr)/(Iz*Vx)*x2 - -2*(Cf*Lf-Cr*Lr)/(Iz)*x3 + 2*(Cf*Lf*Lf+Cr*Lr*Lr)/(Iz*Vx)*x4...
    -2*(Cf*Lf/Iz)*u1 + 2*(Cr*Lr/Iz)*u2 + 2*((Cf*Lf*Lf+Cr*Lr*Lr)/Iz)*v1;
% Linearization
Matrix_Ac_FWS = jacobian([f1,f2,f3,f4],[x1,x2,x3,x4]);
Matrix_Ac_RWS = jacobian([f1,f2,f3,f4],[x1,x2,x3,x4]);
Matrix_Ac_4WS = jacobian([f1,f2,f3,f4],[x1,x2,x3,x4]);
Matrix_Bc_FWS = jacobian([f1,f2,f3,f4],[u1]);
Matrix_Bc_RWS = jacobian([f1,f2,f3,f4],[u2]);
Matrix_Bc_4WS = jacobian([f1,f2,f3,f4],[u1,u2]);
Matrix_Gc_FWS = jacobian([f1,f2,f3,f4], [v1]);
Matrix_Gc_RWS = jacobian([f1,f2,f3,f4], [v1]);
Matrix_Gc_4WS = jacobian([f1,f2,f3,f4], [v1]);
% Discretization
Matrix_Ad_FWS = eye(nX) + Ts*Matrix_Ac_FWS;
Matrix_Ad_RWS = eye(nX) + Ts*Matrix_Ac_RWS;
Matrix_Ad_4WS = eye(nX) + Ts*Matrix_Ac_4WS;
Matrix_Bd_FWS =           Ts*Matrix_Bc_FWS;
Matrix_Bd_RWS =           Ts*Matrix_Bc_RWS;
Matrix_Bd_4WS =           Ts*Matrix_Bc_4WS;
Matrix_Gd_FWS =           Ts*Matrix_Gc_FWS;
Matrix_Gd_RWS =           Ts*Matrix_Gc_RWS;
Matrix_Gd_4WS =           Ts*Matrix_Gc_4WS;