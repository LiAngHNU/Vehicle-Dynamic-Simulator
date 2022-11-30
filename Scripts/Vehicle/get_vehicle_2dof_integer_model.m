%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hunan University
% Published by: Ang Li
% Email: ang1997@hnu.edu.cn
% Copyright (c) 2022 . All rights reserved.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load Vehicle Infomation
load('Models\Vehicles\F_Class_Sedan_4WS\F_Class_Sedan_4WS.mat');
M = Vehicle.Mass;
Iz = Vehicle.Inertia;
Lf = Vehicle.Lf;
Lr = Vehicle.Lr;
Cf = Vehicle.Cf;
Cr = Vehicle.Cr;

% Calculate Continuous Vehicle Model
a22 =  2*(Cf+Cr)/(M*Vx);        b21 = -2*Cf/M;          b22 = -2*Cr/M;
a23 = -2*(Cf+Cr)/(M);           b41 = -2*Cf*Lf/Iz;      b42 =  2*Cr*Lr/Iz;
a24 =  2*(Cf*Lf-Cr*Lr)/(M*Vx);     

a42 =  2*(Cf*Lf-Cr*Lr)/(Iz*Vx); g21 = 2*(Cf*Lf-Cr*Lr)/(M) - Vx*Vx;
a43 = -2*(Cf*Lf-Cr*Lr)/(Iz);    g41 = 2*(Cf*Lf*Lf+Cr*Lr*Lr)/(Iz);
a44 =  2*(Cf*Lf*Lf+Cr*Lr*Lr)/(Iz*Vx);

Matrix_A = [0   0   1   0   0;...
            0   0 a22 a23 a24;...
            0   0   0   0   1;...
            0   0 a42 a43 a44;...
            0   1   0   0   0;];
Matrix_B = [  0   0;...
            b21 b22;...
              0   0;...
            b41 b42;...
              0   0];
Matrix_G = [  0;...
            g21;...
              0;...
            g41;...
              0];

% Calculate Discrete Vehicle Model