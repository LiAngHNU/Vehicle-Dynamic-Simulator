%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Model Predictive Controller %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Version: v1.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date: 2022/11/17 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Controller Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set Steps
Ts = 0.05;
Tp = 40;
% Set Dimensions
nX = 4;
nU = 1;
nV = 1;
% Set Weights
Matrix_Qp = diag([1000 1000 1000 1000]);
Matrix_Qt = diag([1000 1000 1000 1000]);
Matrix_Pe = diag([ 1e3  1e3  1e3  1e3  1e3  1e3  1e3  1e3]);
% Set Constraints
Umax = +0.625;      Dmax = +0.200;
Umin = -0.625;      Dmin = -0.200;
% Set Ref Line
load('Scenarios\2004_mcity_line_04.mat');
% Set Vehicle
load('Models\Vehicles\VehicleSim\RacingVehicles\HUR_A18\HUR_A18_Calib.mat');
% Set Start Position
StartID = 1000;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Update States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Update Vehicle Parameters
Ms = Vehicle_Calib.Mass;        Iz = Vehicle_Calib.Inertia;
Lf = Vehicle_Calib.Lf;          Lr = Vehicle_Calib.Lr;
Cf = Vehicle_Calib.Cyf;         Cr = Vehicle_Calib.Cyr;
Vx = 15;
% Update States
x1 = +0.00;
x2 = +0.00;
x3 = +0.00;
x4 = +0.00;
% Update Inputs
% Update States
Vector_Xo = [x1;x2;x3;x4];
% Update Disturbances
Vector_Vo = zeros(Tp, 1);
for i = 1:1:Tp
Vector_Vo(i,1) = LocalizationInfo(36+i);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Vectors & Matrices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize Matrices
Matrix_Ac = zeros(nX,nX);
Matrix_Bc = zeros(nX,nU);
Matrix_Gc = zeros(nX,nV);
% Update Matrices
Matrix_Ac(1,2) = +1;
Matrix_Ac(2,2) = +2*(Cf + Cr)/(Ms*Vx);
Matrix_Ac(2,3) = -2*(Cf + Cr)/Ms;
Matrix_Ac(2,4) = +2*(Cf*Lf - Cr*Lr)/(Ms*Vx);
Matrix_Ac(3,4) = +1;
Matrix_Ac(4,2) = +2*(Cf*Lf - Cr*Lr)/(Iz*Vx);
Matrix_Ac(4,3) = -2*(Cf*Lf - Cr*Lr)/Iz;
Matrix_Ac(4,4) = +2*(Cf*Lf^2 + Cr*Lr^2)/(Iz*Vx);
Matrix_Bc(2,1) = -(2*Cf)/Ms;
Matrix_Bc(4,1) = -(2*Cf*Lf)/Iz;
Matrix_Gc(2,1) = -Vx^2 + (2*Cf*Lf - 2*Cr*Lr)/Ms;
Matrix_Gc(4,1) = +(2*(Cf*Lf^2 + Cr*Lr^2))/Iz;
% Discretization
Matrix_Ad = eye(nX) + Ts*Matrix_Ac;
Matrix_Bd =           Ts*Matrix_Bc;
Matrix_Gd =           Ts*Matrix_Gc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Prediction Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% QP Construction %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cost Function
% Constraints