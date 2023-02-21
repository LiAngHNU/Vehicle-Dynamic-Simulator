%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Model Predictive Path Integral Controller %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Version: v2.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date: 2023/02/13 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
nS = 100;
% Set MPPI Parameters
Exp = 0.00;
Var = 0.50;
Lam = 10.0;
% Set Weights
Qp = diag([   1    0    5    0]);    % Progress State Weights
Qt = diag([  10    0    5    0]);    % Terminal State Weights
Ru = diag([  10]);
% Set Ref Line
load('Scenarios\2004_mcity_line_04.mat');
StartID = 500; s0 = RefLineInfo.S(StartID);
% Set Vehicle
load('Configs\Liuqi_H5\Liuqi_H5_Calib.mat');
Ms = Vehicle_Calib.Ms;      Iz = Vehicle_Calib.Iz;
Lf = Vehicle_Calib.Lf;      Lr = Vehicle_Calib.Lr;
Cf = Vehicle_Calib.Cyf;     Cr = Vehicle_Calib.Cyr;
Vx = +15.00;
a22 =  2*(Cf+Cr)/(Ms*Vx);          a42 =  2*(Cf*Lf-Cr*Lr)/(Iz*Vx);
a23 = -2*(Cf+Cr)/Ms;               a43 = -2*(Cf*Lf-Cr*Lr)/Iz;    
a24 =  2*(Cf*Lf-Cr*Lr)/(Ms*Vx);    a44 =  2*(Cf*Lf*Lf+Cr*Lr*Lr)/(Iz*Vx);   
b21 = -2*Cf/Ms;                    b41 = -2*Cf*Lf/Iz;
g21 =  2*(Cf*Lf-Cr*Lr)/Ms - Vx^2;  g41 =  2*(Cf*Lf*Lf+Cr*Lr*Lr)/Iz;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Vectors & Matrices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize Initial Optimal Control Sequence
Matrix_uopt_last = zeros(nP, 1);
% Initialize 
Matrix_x1 = zeros(nP+1,nS);   Matrix_dx1 = zeros(nP,nS);
Matrix_x2 = zeros(nP+1,nS);   Matrix_dx2 = zeros(nP,nS);
Matrix_x3 = zeros(nP+1,nS);   Matrix_dx3 = zeros(nP,nS);
Matrix_x4 = zeros(nP+1,nS);   Matrix_dx4 = zeros(nP,nS);
Matrix_u1 = zeros(nP+1,nS);
Matrix_d1 = zeros(nP+1,nS);
Matrix_v1 = zeros(nP+1, 1);
Matrix_sp = zeros(nP,nS);
Matrix_jp = zeros(1, nS);
Matrix_wt = zeros(1, nS);
Matrix_u1_opt = zeros(nP+1,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Update States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Update States X(k)
x1 = +0.10;     % Linear Coordinate along X-axis
x2 = +0.02;     % Linear Coordinate along Y-axis
x3 = +0.10;     % Angular Coordinate around Z-axis
x4 = +0.05;     % Linear Velocity along X-axis
for i = 1:1:nS
    Matrix_x1(1,i) = x1;
    Matrix_x2(1,i) = x2;
    Matrix_x3(1,i) = x3;
    Matrix_x4(1,i) = x4;
end
for i= 1:1:nP+1
    Matrix_v1(i,1) = interp1(RefLineInfo.S,RefLineInfo.Kpa,s0+(i-1)*Vx*Ts);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate State Sequences %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Matrix_sp = normrnd(Exp,Var,[nP,nS]);
for i = 1:1:nS
    Matrix_u1(2:end,i) = Matrix_uopt_last + Matrix_sp(:,i);
    Matrix_d1(2:end,i) = diff(Matrix_u1(:,i));
end
for i = 1:1:nS
    for j = 1:1:nP
        Matrix_dx1(j,i) = Matrix_x2(j,i);
        Matrix_x1(j+1,i) = Matrix_x1(j,i) + Matrix_dx1(j,i)*Ts;
        
        Matrix_dx2(j,i) = a22*Matrix_x2(j,i) + a23*Matrix_x3(j,i) + a24*Matrix_x4(j,i) + ...
                          b21*Matrix_u1(j,i) + ...
                          g21*Matrix_v1(j,1);
        Matrix_x2(j+1,i) = Matrix_x2(j,i) + Matrix_dx2(j,i)*Ts;
        
        Matrix_dx3(j,i) = Matrix_x4(j,i);
        Matrix_x3(j+1,i) = Matrix_x3(j,i) + Matrix_dx3(j,i)*Ts;
        
        Matrix_dx4(j,i) = a42*Matrix_x2(j,i) + a43*Matrix_x3(j,i) + a44*Matrix_x4(j,i) + ...
                          b41*Matrix_u1(j,i) + ...
                          g41*Matrix_v1(j,1);
        Matrix_x4(j+1,i) = Matrix_x4(j,i) + Matrix_dx4(j,i)*Ts;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate Cost %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:nS
    for j = 2:1:nP
        Matrix_jp(1,i) = Matrix_jp(1,i) + Qp(1,1)*Matrix_x1(j,i)^2 + ...
                                          Qp(2,2)*Matrix_x2(j,i)^2 + ...
                                          Qp(3,3)*Matrix_x3(j,i)^2 + ...
                                          Qp(4,4)*Matrix_x4(j,i)^2 + ...
                                          Lam*Matrix_uopt_last(j-1,1)*Exp*Matrix_sp(j,i);
    end
    Matrix_jp(1,i) = Matrix_jp(1,i) + Qt(1,1)*Matrix_x1(nP+1,i)^2 + ...
                                      Qt(2,2)*Matrix_x2(nP+1,i)^2 + ...
                                      Qt(3,3)*Matrix_x3(nP+1,i)^2 + ...
                                      Qt(4,4)*Matrix_x4(nP+1,i)^2 + ...
                                      Lam*Matrix_uopt_last(nP,1)*Exp*Matrix_sp(nP,i);
end
minID = find(Matrix_jp==min(Matrix_jp));
minJP = Matrix_jp(minID);
Ita = 0;
for i = 1:1:nS
    Ita = Ita + exp((minJP-Matrix_jp(1,i))/Lam);
end
for i = 1:1:nS
    Matrix_wt(1,i) = (1/Ita)*exp((minJP-Matrix_jp(1,i))/Lam);
end
for i = 1:1:nS
    Matrix_u1_opt(:,1) = Matrix_u1_opt(:,1) + Matrix_wt(1,i)*Matrix_u1(:,i);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,3,1); hold on; title('Lateral Error');
subplot(2,3,4); hold on; title('Lateral Error Rate');
subplot(2,3,2); hold on; title('Heading Error');
subplot(2,3,5); hold on; title('Heading Error Rate');
subplot(2,3,3); hold on; title('Steer Angle');
subplot(2,3,6); hold on; title('Steer Rate');
for i = 1:1:nS
    if i == minID
    else
        subplot(2,3,1); plot(Matrix_x1(:,i),'k:');
        subplot(2,3,4); plot(Matrix_x2(:,i),'k:');
        subplot(2,3,2); plot(Matrix_x3(:,i),'k:');
        subplot(2,3,5); plot(Matrix_x4(:,i),'k:');
        subplot(2,3,3); plot(Matrix_u1(:,i),'k:');
        subplot(2,3,6); plot(Matrix_d1(:,i),'k:');
    end
    pause(0.05);
end
subplot(2,3,1); plot(Matrix_x1(:,i),'r-','LineWidth',2);
subplot(2,3,4); plot(Matrix_x2(:,i),'r-','LineWidth',2);
subplot(2,3,2); plot(Matrix_x3(:,i),'r-','LineWidth',2);
subplot(2,3,5); plot(Matrix_x4(:,i),'r-','LineWidth',2);
subplot(2,3,3); plot(Matrix_u1(:,i),'r-','LineWidth',2);
subplot(2,3,6); plot(Matrix_d1(:,i),'r-','LineWidth',2);