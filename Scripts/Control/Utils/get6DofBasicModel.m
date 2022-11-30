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
syms Ts;
% Set Vehicle Parameters
syms Ms Iz Lf Lr Cf Cr Vx Re;
% Set States & Inputs & Disturbances
syms x1 x2 x3 x4 x5 x6;
syms u1 u2;
syms v1;
% Set Dimensions
nX = 6;
nU = 2;
% Set Intermediate Variables
Af = (x5 + Lf*x6)/(x4) - u2;
Ar = (x5 - Lr*x6)/(x4);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% get Vehicle Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% System Function
f1 =  x4*cos(x3) - x5*sin(x3);
f2 =  x4*sin(x3) + x5*cos(x3);
f3 =  x6;
f4 =  (1/Ms)*((1/Re)*u1 - Cf*Af*sin(u2) + Ms*x5*x6);
f5 =  (Cf+Cr)/(Ms*x4)*x5 + ((Cf*Lf-Cr*Lr)/(Ms*x4))*x6 - (Cf/Ms)*u2;
f6 =  (1/Iz)*(Cf*Af*Lf*cos(u2) - Cr*Ar*Lr);
% System Linearization
Matrix_Ac = jacobian([f1,f2,f3,f4,f5,f6], [x1,x2,x3,x4,x5,x6]);
Matrix_Bc = jacobian([f1,f2,f3,f4,f5,f6], [u1,u2]);
Matrix_Cc = eye(nX);
Matrix_Dc = zeros(nX,nU);
% Matrix Discretization
Matrix_Ad = eye(size(Matrix_Ac)) + Ts*Matrix_Ac;
Matrix_Bd =                        Ts*Matrix_Bc;
