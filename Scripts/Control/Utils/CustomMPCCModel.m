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
syms Ms Iz Lf Lr Cf Cr;
syms x1 x2 x3 x4 x5;
syms u1 u2;
syms v1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Matrix Initialization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% System Function
f1 = x2;
f2 = 2*x2*(Cf+Cr)/(Ms*u1) - 2*x3*(Cf+Cr)/(Ms) + 2*x4*(Cf*Lf-Cr*Lr)/(Ms*u1) - 2*u2*Cf/Ms + (2*(Cf*Lf-Cr*Lr)/(Ms)-u1*u1)*v1;
f3 = x4;
f4 = 2*x2*(Cf*Lf-Cr*Lr)/(Iz*u1) - 2*x3*(Cf*Lf-Cr*Lr)/(Iz) + 2*x4*(Cf*Lf*Lf+Cr*Lr*Lr)/(Iz*x1) - 2*Cf*Lf*u2/Iz + 2*v1*(Cf*Lf*Lf+Cr*Lr*Lr)/Iz;
f5 = (u1 + u1*x3*x3 - x2*x3)/(1 - v1*x1);
% System Linearization
Matrix_Ac = jacobian([f1,f2,f3,f4,f5], [x1,x2,x3,x4,x5]);
Matrix_Bc = jacobian([f1,f2,f3,f4,f5], [u1,u2]);
Matrix_Gc = jacobian([f1,f2,f3,f4,f5], [v1]);
% Matrix Discretization
Matrix_Ad = eye(size(Matrix_Ac)) + Ts*Matrix_Ac;
Matrix_Bd =                        Ts*Matrix_Bc;
Matrix_Gd =                        Ts*Matrix_Gc;

