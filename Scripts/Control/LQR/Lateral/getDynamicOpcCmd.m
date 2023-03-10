%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Online Lateral Dynamic Optimal Preview Controller %%%%%%%%%%%%%%%%%%%%%%%
% Version: v1.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date: 2023/03/09 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc; tic;                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Controller Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%**** Set Vehicle ********************************************************%
FLAG_VEHICLE_ = "Audi_R8_Etron";                                          %
FLAG_STEER_TYPE_ = "4WS";                                                 %
%**** Load Vehicle *******************************************************%
load(strcat("Configs\",FLAG_VEHICLE_,"\Vehicle\Vehicle_Calib.mat"));      %
Ms = Vehicle_Calib.Ms;          Iz = Vehicle_Calib.Iz;                    %
Lf = Vehicle_Calib.Lf;          Lr = Vehicle_Calib.Lr;                    %
Cf = Vehicle_Calib.Cyf;         Cr = Vehicle_Calib.Cyr;                   %
%**** Set Steps **********************************************************%
tS =  0.01;     % Time Gap of Discretization                              %
tL =  0.20;     % First-Order Lag Coefficient of Front Steer Actuator     % 
tP =    50;     % Horinzon Length of Preview                              % 
%**** Set Weights ********************************************************%
q1 = 1;                                                                   %
q2 = 1;                                                                   %
q3 = 1;                                                                   %
q4 = 1;                                                                   %
r1 = 1;                                                                   %
r2 = 1;                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Matrices & Vectors %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%**** Initialize Continuous Matrices *************************************%
Matrix_Ac_4ws = zeros(4,4);             Matrix_Ad_4ws = zeros(4,4);
Matrix_Bc_4ws = zeros(4,2);             Matrix_Bd_4ws = zeros(4,2);
Matrix_Gc_4ws = zeros(4,1);             Matrix_Gd_4ws = zeros(4,1);
%**** Initialize Preview Matrices ****************************************%
Matrix_Ap_4ws = zeros(4+tP,4+tP);
Matrix_Bp_4ws = zeros(4+tP,   2);
%**** Initialize Weight Matrices *****************************************%
Matrix_Qp_4ws = zeros(4+tP,4+tP);
Matrix_Rp_4ws = zeros(2,2);
%**** Initialize Gain Matrices *******************************************%
Matrix_Gain_4ws = zeros(2,4+tP);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Update States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Vx = +5.00;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate LQR Matrices & Vectors %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%**** Calculate Matrix Ac, Bc & Gc ***************************************%
Matrix_Ac_4ws(1,2) = +1;
Matrix_Ac_4ws(2,2) = +2*(Cf+Cr)/(Ms*Vx);
Matrix_Ac_4ws(2,3) = -2*(Cf+Cr)/(Ms);
Matrix_Ac_4ws(2,4) = +2*(Cf*Lf-Cr*Lr)/(Ms*Vx);
Matrix_Ac_4ws(3,4) = +1;
Matrix_Ac_4ws(4,2) = +2*(Cf*Lf-Cr*Lr)/(Iz*Vx);
Matrix_Ac_4ws(4,3) = -2*(Cf*Lf-Cr*Lr)/(Iz);
Matrix_Ac_4ws(4,4) = +2*(Cf*Lf^2+Cr*Lr^2)/(Iz*Vx);
Matrix_Bc_4ws(2,1) = -2*(Cf)/(Ms);
Matrix_Bc_4ws(2,2) = -2*(Cr)/(Ms);
Matrix_Bc_4ws(4,1) = -2*(Cf*Lf)/(Iz);
Matrix_Bc_4ws(4,2) = +2*(Cr*Lr)/(Iz);
Matrix_Gc_4ws(2,1) = +2*(Cf*Lf-Cr*Lr)/(Ms)-Vx^2;
Matrix_Gc_4ws(4,1) = +2*(Cf*Lf^2+Cr*Lr^2)/(Iz);
%**** Calculate Matrix Ad, Bd & Gd ***************************************%
Matrix_Ad_4ws = eye(4) + tS*Matrix_Ac_4ws;
Matrix_Bd_4ws =          tS*Matrix_Bc_4ws;
Matrix_Gd_4ws =          tS*Matrix_Gc_4ws;
%**** Calculate Matrix Ap, Bp & Gp ***************************************%
Matrix_Ap_4ws(1:4,1:4) = Matrix_Ad_4ws;
Matrix_Ap_4ws(1:4,  5) = Matrix_Gd_4ws;
Matrix_Ap_4ws(5:end-1,6:end) = eye(tP-1);
Matrix_Bp_4ws(1:4,1:2) = Matrix_Bd_4ws;
%**** Calculate Matrix Ap, Bp & Gp ***************************************%
Matrix_Qp_4ws(1,1) = q1;
Matrix_Qp_4ws(2,2) = q2;
Matrix_Qp_4ws(3,3) = q3;
Matrix_Qp_4ws(4,4) = q4;
Matrix_Rp_4ws(1,1) = r1;
Matrix_Rp_4ws(2,2) = r2;
Matrix_Gain_4ws = dlqr(Matrix_Ap_4ws,Matrix_Bp_4ws,Matrix_Qp_4ws,Matrix_Rp_4ws);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,1,1);hold on;
plot(Matrix_Gain_4ws(1,1:4));
plot(Matrix_Gain_4ws(2,1:4));
subplot(2,1,2);hold on;
plot(Matrix_Gain_4ws(1,5:end));
plot(Matrix_Gain_4ws(2,5:end));