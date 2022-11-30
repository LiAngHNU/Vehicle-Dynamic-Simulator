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
syms Ms Iz Lf Lr Cf Cr Vx Re;
% Set States & Inputs & Disturbances
syms x1 x2 x3 x4 x5 x6;
syms u1 u2;
syms v1;
% Set Dimensions
nX = 5;
nU = 2;
% Set Intermediate Variables
Af = (x5 + Lf*x6)/(x4) - u2;
Ar = (x5 - Lr*x6)/(x4);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% get Vehicle Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% System Function
f1 = x2;
f2 = 2*x2*(Cf+Cr)/(Ms*u1) - 2*x3*(Cf+Cr)/(Ms) + 2*x4*(Cf*Lf-Cr*Lr)/(Ms*u1) - 2*u2*Cf/Ms + (2*(Cf*Lf-Cr*Lr)/(Ms)-u1*u1)*v1;
f3 = x4;
f4 = 2*x2*(Cf*Lf-Cr*Lr)/(Iz*u1) - 2*x3*(Cf*Lf-Cr*Lr)/(Iz) + 2*x4*(Cf*Lf*Lf+Cr*Lr*Lr)/(Iz*x1) - 2*Cf*Lf*u2/Iz + 2*v1*(Cf*Lf*Lf+Cr*Lr*Lr)/Iz;
f5 = (u1*cos(x3)-(x2-u1*x3)*sin(x3))/(1-x1*v1);
% System Linearization
Matrix_Ac = jacobian([f1,f2,f3,f4,f5], [x1,x2,x3,x4,x5]);
Matrix_Bc = jacobian([f1,f2,f3,f4,f5], [u1,u2]);
Matrix_Gc = jacobian([f1,f2,f3,f4,f5], [v1]);
% Matrix Discretization
Matrix_Ad = eye(size(Matrix_Ac)) + Ts*Matrix_Ac;
Matrix_Bd =                        Ts*Matrix_Bc;
Matrix_Gd =                        Ts*Matrix_Gc;
