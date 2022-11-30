%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Published by: Li.Ang  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This Model is Verified by Simulation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Controller Configuration %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
syms Ts;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Update States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Symbolic Variables Definition
syms Ms Iz Lf Lr Cf Cr Re;
syms Kx Cd Lx Ly;
syms x1 x2 x3 x4 x5 x6 x7;
syms x1_ref x2_ref x3_ref;
syms u1 u2 u3;
syms du1 du2 du3;
% Intermediate Variables Definition
Af = (x5 + Lf*x6)/(x4) - u2;
Ar = (x5 - Lr*x6)/(x4);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Matrix Initialization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% System Function
f1 =  x4*cos(x3) - x5*sin(x3);
f2 =  x4*sin(x3) + x5*cos(x3);
f3 =  x6;
f4 =  (1/Ms)*((1/Re)*u1 - Cf*Af*sin(u2) + Ms*x5*x6);
f5 =  (1/Ms)*(Cr*Ar - Cf*Af*cos(u2) - Ms*x4*x6);
f6 =  (1/Iz)*(Cf*Af*Lf*cos(u2) - Cr*Ar*Lr);
f7 =  x4;
f8 = du1;
f9 = du2;
% System Linearization
Matrix_Ac = jacobian([f1,f2,f3,f4,f5,f6,f7,f8,f9], [x1,x2,x3,x4,x5,x6,x7,u1,u2]);
Matrix_Bc = jacobian([f1,f2,f3,f4,f5,f6,f7,f8,f9], [du1,du2]);
% Matrix Discretization
Matrix_Ad = eye(size(Matrix_Ac)) + Ts*Matrix_Ac;
Matrix_Bd =                        Ts*Matrix_Bc;
% 
Ec = + (x1 - x1_ref)*sin(x3_ref) - (x2 - x2_ref)*cos(x3_ref);
El = - (x1 - x1_ref)*cos(x3_ref) - (x2 - x2_ref)*sin(x3_ref);
PD_EcX = diff(Ec, x1, 1);
PD_EcY = diff(Ec, x2, 1);
PD_ElX = diff(El, x1, 1);
PD_ElY = diff(El, x2, 1);
%
Alpha_F = (x5 + Lf*x6)/(x4) - u2;
Alpha_R = (x5 - Lr*x6)/(x4);
Grad_Alpha_F = jacobian(Alpha_F, [x1,x2,x3,x4,x5,x6,x7,u1,u2]);