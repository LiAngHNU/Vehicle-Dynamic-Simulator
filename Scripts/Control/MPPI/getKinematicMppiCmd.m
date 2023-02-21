%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Model Predictive Path Integral Controller %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Version: v2.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date: 2022/11/16 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Controller Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set Steps
Ts = 0.05;
% Set Dimensions
nP = 50;
nS =  1;
% Set MPPI Parameters
Exp_u1 = 3.00; Exp_u2 = 0.00; Exp_u3 = 0.00;
Var_u1 = 0.10; Var_u2 = 0.20; Var_u3 = 0.10;
Lam_u1 = 1.00; Lam_u2 = 1.00; Lam_u3 = 1.00;
% Set Weights
Qp = diag([   1    0    5    0]);    % Progress State Weights
Qt = diag([  10    0    5    0]);    % Terminal State Weights
Ru = diag([  10]);
% Set Ref Line
load('Scenarios\2004_mcity_line_04.mat');
StartID = 500; s0 = RefLineInfo.S(StartID);
% Set Vehicle
load('Configs\Liuqi_H5\Liuqi_H5_Calib.mat');
Lf = Vehicle_Calib.Lf; Lr = Vehicle_Calib.Lr;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Vectors & Matrices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize Initial Optimal Control Sequence
Matrix_u1_opt_last = zeros(nP,1); Matrix_u1_norm = zeros(nP,1);
Matrix_u2_opt_last = zeros(nP,1); Matrix_u2_norm = zeros(nP,1);
Matrix_u3_opt_last = zeros(nP,1); Matrix_u3_norm = zeros(nP,1);
Matrix_u1 = zeros(nP+1,1);        Matrix_u1_opt = zeros(nP+1,1);
Matrix_u2 = zeros(nP+1,1);        Matrix_u2_opt = zeros(nP+1,1);
Matrix_u3 = zeros(nP+1,1);        Matrix_u3_opt = zeros(nP+1,1);
% Initialize States Matrices
Matrix_x1_prev = zeros(nP+1,nS); Matrix_dx1_prev = zeros(nP+1,nS);
Matrix_x2_prev = zeros(nP+1,nS); Matrix_dx2_prev = zeros(nP+1,nS);
Matrix_x3_prev = zeros(nP+1,nS); Matrix_dx3_prev = zeros(nP+1,nS);
Matrix_x4_prev = zeros(nP+1,nS); Matrix_dx4_prev = zeros(nP+1,nS);
Matrix_x5_prev = zeros(nP+1,nS); Matrix_dx5_prev = zeros(nP+1,nS);
Matrix_x6_prev = zeros(nP+1,nS); Matrix_dx6_prev = zeros(nP+1,nS);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Update States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1 = RefLineInfo.X(StartID);        x4 = +3.00;
x2 = RefLineInfo.Y(StartID);        x5 = +0.00;
x3 = RefLineInfo.Tht(StartID);      x6 = +0.00;
for i = 1:1:nS
    Matrix_x1_prev(1,i) = x1;
    Matrix_x2_prev(1,i) = x2;
    Matrix_x3_prev(1,i) = x3;
    Matrix_x4_prev(1,i) = x4;
    Matrix_x5_prev(1,i) = x5;
    Matrix_x6_prev(1,i) = x6;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Generate Input Sequences %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Matrix_u1_norm = normrnd(Exp_u1,Var_u1,[nP,nS]);
Matrix_u2_norm = normrnd(Exp_u2,Var_u2,[nP,nS]);
Matrix_u3_norm = normrnd(Exp_u3,Var_u3,[nP,nS]);
for i = 1:1:nS
    Matrix_u1(2:end,i) = Matrix_u1_opt_last + Matrix_u1_norm(:,i);
    Matrix_u2(2:end,i) = Matrix_u2_opt_last + Matrix_u2_norm(:,i);
    Matrix_u3(2:end,i) = Matrix_u3_opt_last + Matrix_u3_norm(:,i);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate State Sequences %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:nS
    for j = 1:1:nP
        Ltmp = (Lr*tan(Matrix_u2(j+1,i)) + Lf*tan(Matrix_u3(j+1,i)))/(tan(Matrix_u2(j+1,i)) - tan(Matrix_u3(j+1,i))); 
        
        Matrix_dx1_prev(j,i) = Matrix_x4_prev(j,i)*cos(Matrix_x3_prev(j,i)) - Matrix_x5_prev(j,i)*sin(Matrix_x3_prev(j,i));
        Matrix_x1_prev(j+1,i) = Matrix_x1_prev(j,i) + Matrix_dx1_prev(j,i)*Ts;

        Matrix_dx2_prev(j,i) = Matrix_x4_prev(j,i)*sin(Matrix_x3_prev(j,i)) + Matrix_x5_prev(j,i)*cos(Matrix_x3_prev(j,i));
        Matrix_x2_prev(j+1,i) = Matrix_x2_prev(j,i) + Matrix_dx2_prev(j,i)*Ts;

        Matrix_dx3_prev(j,i) = Matrix_x6_prev(j,i);
        Matrix_x3_prev(j+1,i) = Matrix_x3_prev(j,i) + Matrix_dx3_prev(j,i)*Ts;

        Matrix_dx4_prev(j,i) = Matrix_u3_norm(j,i);
        Matrix_x4_prev(j+1,i) = Matrix_x4_prev(j,i) + Matrix_dx4_prev(j,i)*Ts;

        Matrix_dx5_prev(j,i) = Matrix_x4_prev(j,i)*tan(Matrix_u2(j+1,i))*((Lr+Ltmp)/(Lf+Lr+Ltmp)) + Matrix_x4_prev(j,i)*tan(Matrix_u3(j+1,i))*((Lr+Ltmp)/(Ltmp));
        Matrix_x5_prev(j+1,i) = Matrix_x5_prev(j,i) + Matrix_dx5_prev(j,i)*Ts;

        Matrix_dx6_prev(j,i) = Matrix_x4_prev(j,i)*tan(Matrix_u2(j+1,i))/(Lf+Lr+Ltmp) + Matrix_x4_prev(j,i)*tan(Matrix_u3(j+1,i))/(Ltmp);
        Matrix_x6_prev(j+1,i) = Matrix_x6_prev(j,i) + Matrix_dx6_prev(j,i)*Ts;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate Cost %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hold on; axis equal;
plot(RefLineInfo.X,RefLineInfo.Y,'k');
plot(RefLineInfo.Xl,RefLineInfo.Yl,'r');
plot(RefLineInfo.Xr,RefLineInfo.Yr,'r');
for i = 1:1:nS
    plot(Matrix_x1_prev(:,i),Matrix_x2_prev(:,i),'k:');
    pause(0.05);
end