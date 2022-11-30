%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle 2DoF MPCC Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
syms Ms Iz Lf Lr Cf Cr Re;
% Set States & Inputs & Disturbances
syms x1 x2 x3 x4 x5 x6 x7;
syms u1 u2 u3 du1 du2 du3;
% Set Dimensions
nX =  7;
nU =  3;
nD =  3;
% Set Intermediate Variables
Af = (x5 + Lf*x6)/(x4) - u2;
Ar = (x5 - Lr*x6)/(x4);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% get Vehicle Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% System Function
f1 = x4*cos(x3) - x5*sin(x3);
f2 = x4*sin(x3) + x5*cos(x3);
f3 = x6;
f4 = (1/Ms)*((1/Re)*u1 - Cf*Af*sin(u2) + Ms*x5*x6);
f5 = (1/Ms)*((Cf+Cr)*x5/x4 + (Cf*Lf-Cr*Lr)*x6/x4 - Cf*u2);
f6 = (1/Iz)*(Cf*Af*Lf*cos(u2) - Cr*Ar*Lr);
f7 = u3;
f8 = du1;
f9 = du2;
f10=du3;
% System Linearization
Matrix_Ac = jacobian([f1,f2,f3,f4,f5,f6,f7], [x1,x2,x3,x4,x5,x6,x7]);
Matrix_Bc = jacobian([f1,f2,f3,f4,f5,f6,f7], [u1,u2,u3]);
% Matrix Discretization
Matrix_Ad = eye(nX) + Ts*Matrix_Ac;
Matrix_Bd =           Ts*Matrix_Bc;