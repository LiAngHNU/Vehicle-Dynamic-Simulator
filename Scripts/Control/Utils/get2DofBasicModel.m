%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle 2DoF Basic Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
syms x1 x2;
syms u1 u2;
% Set Dimensions
nX = 2;
nU = 2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% get Vehicle Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get System Dynamics
f1 = (2*(Cf+Cr)/(Ms*Vx))*x1 + (2*(Cf*Lf-Cr*Lr)/(Ms*Vx) - Vx)*x2 - (2*Cf/Ms)*u1 - (2*Cr/Ms)*u2;
f2 = (2*(Cf*Lf-Cr*Lr)/(Iz*Vx))*x1 + (2*(Cf*Lf*Lf+Cr*Lr*Lr)/(Iz*Vx))*x2- (2*Cf*Lf/Ms)*u1 + (2*Cr*Lr/Ms)*u2;
% Linearization
Matrix_Ac_FWS = jacobian([f1,f2],[x1,x2]);
Matrix_Ac_RWS = jacobian([f1,f2],[x1,x2]);
Matrix_Ac_4WS = jacobian([f1,f2],[x1,x2]);
Matrix_Bc_FWS = jacobian([f1,f2],[u1]);
Matrix_Bc_RWS = jacobian([f1,f2],[u2]);
Matrix_Bc_4WS = jacobian([f1,f2],[u1,u2]);
% Discretization
Matrix_Ad_FWS = eye(nX) + Ts*Matrix_Ac_FWS;
Matrix_Ad_RWS = eye(nX) + Ts*Matrix_Ac_RWS;
Matrix_Ad_4WS = eye(nX) + Ts*Matrix_Ac_4WS;
Matrix_Bd_FWS =           Ts*Matrix_Bc_FWS;
Matrix_Bd_RWS =           Ts*Matrix_Bc_RWS;
Matrix_Bd_4WS =           Ts*Matrix_Bc_4WS;