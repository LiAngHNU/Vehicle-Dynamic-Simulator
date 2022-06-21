%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control System Analysis                                                 %
% Hunan University                                                        %
% Author: Ang Li                                                          %
% Email: ang1997@hnu.edu.cn                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% System Parameters Definition                                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
syms s;
% System Cofficients Definition
Ts = 0.01;
Td = 0.20;
Tl = 0.20;
Tp = 2.00;
% Lead-Lag Cofficients Definition
Kgc = 1.00;
Ka  = 0.00;
Kb  = 0.00;
Kc  = 0.00;
Kd  = 0.00;
% Filter Coefficients Definition
% Control Coefficients Definition
Pd = Td/Ts;
Pp = Tp/Ts;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% System Initial Weight Settings                                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Vx_table = [1.25, 2.50, 3.75, 5.00, 6.25, 7.50, 8.75,10.00,11.25,12.50]';

q1_table = [   1,    1,    1,    1,    1,    1,    1,    1,    1,    1];
q2_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0];
q3_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0];
q4_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0];
q5_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0];
q6_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0];
r1_table = [   1,    1,    1,    1,    1,    1,    1,    1,    1,    1];

Coeff_k1 = 1.00;
Coeff_k2 = 1.00;
Coeff_k3 = 1.00;
Coeff_k4 = 1.00;
Coeff_ki = 0.00;
Coeff_Kl = 0.00;
Coeff_kd = 0.00;
Coeff_Kp = 0.00;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% System Model Generation                                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('Models\Vehicles\CiDi\Foton_Auman\Foton_Auman_Calib.mat');
M = Vehicle_Calib.Mass;
Iz = Vehicle_Calib.Inertia;
Lf = Vehicle_Calib.Lf;
Lr = Vehicle_Calib.Lr;
Cyf = Vehicle_Calib.Cyf;    
Cyr = Vehicle_Calib.Cyr;
Vx = Vx_table(1,1);

q1 = q1_table(1,1);
q2 = q2_table(1,1);
q3 = q3_table(1,1);
q4 = q4_table(1,1);
r  = r1_table(1,1);

a22 =  2*(Cyf+Cyr)/(M*Vx);          a42 =  2*(Cyf*Lf-Cyr*Lr)/(Iz*Vx);
a23 = -2*(Cyf+Cyr)/M;               a43 = -2*(Cyf*Lf-Cyr*Lr)/Iz;
a24 =  2*(Cyf*Lf-Cyr*Lr)/(M*Vx);    a44 =  2*(Cyf*Lf*Lf+Cyr*Lr*Lr)/(Iz*Vx);
a26 = -2*Cyf/M;                     a46 = -2*Cyf*Lf/Iz;
a66 = -1/Tl;                        b61 =  1/Tl;

g21 = 2*(Cyf*Lf-Cyr*Lr)/M - Vx^2;   g41 = 2*(Cyf*Lf*Lf+Cyr*Lr*Lr)/Iz;

Matrix_A = [0,  1,  0,  0,  0,  0;...
            0,a22,a23,a24,  0,a26;...
            0,  0,  0,  1,  0,  0;...
            0,a42,a43,a44,  0,a46;...
            1,  0,  0,  0,  0,  0;...
            0,  0,  0,  0,  0,a66;];
Matrix_B = [  0;...
              0;...
              0;...
              0;...
              0;...
            b61;];
Matrix_G = [  0;...
            g21;...
              0;...
            g41;...
              0;...
              0;];

Matrix_A = eye(6) + Ts*Matrix_A;
Matirx_B =          Ts*Matrix_B;
Matrix_G =          Ts*Matrix_G;

Matrix_Delay = [zeros(Pd - 1, 1), eye(Pd - 1);...
                0,                          zeros(1, Pd - 1);];
Matrix_Prev  = [zeros(Pp - 1, 1),   eye(Pp - 1);...
                0,                           zeros(1, Pp - 1);];

Matrix_A_Aug = zeros(6 + Pd + Pp, 6 + Pd + Pp);
Matrix_B_Aug = zeros(6 + Pd + Pp, 1);
Matrix_G_Aug = zeros(6 + Pd + Pp, 1);

Matrix_A_Aug(1:6, 1:6) = Matrix_A;
Matrix_A_Aug(1:6, 7)   = Matrix_B;
Matrix_A_Aug(1:6, 7 + Pd)= Matrix_G;
Matrix_A_Aug(7:6+Pd, 7:6+Pd) = Matrix_Delay;
Matrix_A_Aug(7+Pd:6+Pd+Pp, 7+Pd:6+Pd+Pp) = Matrix_Prev;

Matrix_B_Aug(6+Pd, 1) = 1;

Matrix_G_Aug(6+Pd+Pp, 1) = 1;

Matrix_Q_Aug = diag([q1 q2 q3 q4 zeros(1, 2+Pd+Pp)]);

Matrix_R_Aug = diag(r);

Matrix_K_Aug = dlqr(Matrix_A_Aug, Matrix_B_Aug, Matrix_Q_Aug, Matrix_R_Aug);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% System Performance Analysis                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Complex-Frequency-Domain Performance Analysis
% Complex-Frequency-Domain Performance: Tranfer Function Matrix
% TF_leadlag  = Kgc*(a*s+1)*(c*s+1)/(b*s+1)/(d*s+1);
% TF_delay    = exp(-Td*s);
% TF_lag      = 1/(Tl*s+1);
% TF_model    = inv(s*eye(Matirx_Size) - Matrix_Acl)*Matrix_Bcl;
% TF_Total    = TF_leadlag*TF_delay*TF_lag*TF_model;
% 
% TF_Total = subs();
% TF_Total = subs();
% TF_Total = subs();

% Time-Domain Performance Analysis
% Time-Domain Performance: Overshoot
% Time-Domain Performance: Settling Time
% Time-Domain Performance: Steady-State Error

% Frequency-Domain Performance Analysis
% Frequency-Domain Performance: Margin
% Frequency-Domain Performance: Cutoff Frequency
