%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hunan University
% Published by: Ang Li
% Email: ang1997@hnu.edu.cn
% Copyright (c) 2022 . All rights reserved.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load Vehicle Infomation
load('Models\Vehicles\C_3A_Cab_Over_6x4\C_3A_Cab_Over_6x4_Calib.mat');
M = Vehicle_Calib.Mass;
Iz = Vehicle_Calib.Inertia;
Lf = Vehicle_Calib.Lf;
Lr = Vehicle_Calib.Lr;
Cyf = Vehicle_Calib.Cyf;
Cyr = Vehicle_Calib.Cyr;

% Calculate Continuous Vehicle Model
a22 =  2*(Cyf+Cyr)/(M*Vx);        b21 = -2*Cyf/M;          b22 = -2*Cyr/M;
a23 = -2*(Cyf+Cyr)/(M);           b41 = -2*Cyf*Lf/Iz;      b42 =  2*Cyr*Lr/Iz;
a24 =  2*(Cyf*Lf-Cyr*Lr)/(M*Vx);     

a42 =  2*(Cyf*Lf-Cyr*Lr)/(Iz*Vx); g21 = 2*(Cyf*Lf-Cyr*Lr)/(M) - Vx*Vx;
a43 = -2*(Cyf*Lf-Cyr*Lr)/(Iz);    g41 = 2*(Cyf*Lf*Lf+Cyr*Lr*Lr)/(Iz);
a44 =  2*(Cyf*Lf*Lf+Cyr*Lr*Lr)/(Iz*Vx);

Matrix_A = [0   1   0   0;...
            0 a22 a23 a24;...
            0   0   0   1;...
            0 a42 a43 a44;];
Matrix_B = [  0   0;...
            b21 b22;...
              0   0;...
            b41 b42;];
Matrix_G = [  0;...
            g21;...
              0;...
            g41;];

% Calculate Discrete Vehicle Model