%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle 3DoF Kinematic Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Version: v1.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date: 2022/11/21 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Model Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set Steps
syms Ts Tl;
% Set Vehicle Parameters
syms Lf Lr;
% Set States & Inputs & Disturbances
syms x1 x2 x3;
syms u1 u2;
% Set Dimensions
nX = 3;
nU = 2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% get Vehicle Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% System Function (at Front Axle)
f1 =  u1*cos(u2 + x3);
f2 =  u1*sin(u2 + x3);
f3 =  u1*sin(u2)/(Lf+Lr);
% System Function (at CoG)
beta = atan(Lr/(Lf+Lr)*tan(u2));
f1 =  u1*cos(beta + x3);
f2 =  u1*sin(beta + x3);
f3 =  u1*tan(u2)*cos(beta)/(Lf+Lr);
% System Function (at Rear Axle)
f1 =  u1*cos(u2 + x3);
f2 =  u1*sin(u2 + x3);
f3 =  u1*sin(u2)/(Lf+Lr);
% System Linearization
Matrix_Ac = jacobian([f1,f2,f3], [x1,x2,x3]);
Matrix_Bc = jacobian([f1,f2,f3], [u1,u2]);
% Matrix Discretization
Matrix_Ad = eye(size(Matrix_Ac)) + Ts*Matrix_Ac;
Matrix_Bd =                        Ts*Matrix_Bc;
