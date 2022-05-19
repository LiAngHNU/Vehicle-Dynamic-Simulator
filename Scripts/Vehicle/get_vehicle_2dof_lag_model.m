%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hunan University
% Published by: Ang Li
% Email: ang1997@hnu.edu.cn
% Copyright (c) 2022 . All rights reserved.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Numerical 2DOF Model
% Lag and Delay Settings
Ts = 0.01;
Tl = 0.5;
Td = 30;

% Load Vehicle Infomation
load('Models\Vehicles\C_3A_Cab_Over_6x4\C_3A_Cab_Over_6x4_Calib.mat');
M = Vehicle_Calib.Mass;
Iz = Vehicle_Calib.Inertia;
Lf = Vehicle_Calib.Lf;
Lr = Vehicle_Calib.Lr;
% Cyf = Vehicle_Calib.Cyf;    
% Cyr = Vehicle_Calib.Cyr;    
Cyf = -50000;
Cyr = -100000;
% Calculate Continuous Vehicle Model
a22 =  2*(Cyf+Cyr)/(M*Vx);          b51 = 1/Tl;  
a23 = -2*(Cyf+Cyr)/(M);             g21 = 2*(Cyf*Lf-Cyr*Lr)/(M) - Vx*Vx;
a24 =  2*(Cyf*Lf-Cyr*Lr)/(M*Vx);    g41 = 2*(Cyf*Lf*Lf+Cyr*Lr*Lr)/(Iz);
a25 = -2*Cyf/M;                     

a42 =  2*(Cyf*Lf-Cyr*Lr)/(Iz*Vx); 
a43 = -2*(Cyf*Lf-Cyr*Lr)/(Iz);    
a44 =  2*(Cyf*Lf*Lf+Cyr*Lr*Lr)/(Iz*Vx);
a45 = -2*Cyf*Lf/Iz;

a55 = -1/Tl;

Matrix_A = [0   1   0   0   0;...
            0 a22 a23 a24 a25;...
            0   0   0   1   0;...
            0 a42 a43 a44 a45;...
            0   0   0   0 a55;];
Matrix_B = [  0;...
              0;...
              0;...
              0;...
            b51;];
Matrix_G = [  0;...
            g21;...
              0;...
            g41;...
              0;];

% Calculate Discrete Vehicle Model
Matrix_Ad = eye(5) + Ts*Matrix_A;
Matrix_Bd =          Ts*Matrix_B;
Matrix_Gd =          Ts*Matrix_G;

%% Symbolic 2DOF Model
syms s;
syms a22 a23 a24 a25 a42 a43 a44 a45 a55;
syms b51 g21 g41;
syms k1 k2 k3 k4 k5;
syms kappa;

Matrix_A = [0   1   0   0   0;...
            0 a22 a23 a24 a25;...
            0   0   0   1   0;...
            0 a42 a43 a44 a45;...
            0   0   0   0 a55;];
Matrix_B = [  0;...
              0;...
              0;...
              0;...
            b51;];
Matrix_G = [  0;...
            g21;...
              0;...
            g41;...
              0;];
Matrix_K = [k1, k2, k3, k4, k5];
Matrix_I = eye(5);

Ess = inv(s*Matrix_I - Matrix_A + Matrix_B*Matrix_K)*Matrix_G;
Ess = limit(Ess, s, 0);