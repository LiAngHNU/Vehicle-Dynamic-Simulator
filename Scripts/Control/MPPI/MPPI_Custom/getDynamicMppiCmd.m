%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Model Predictive Path Integral Controller %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Version: v1.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date: 2022/11/16 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Controller Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic;
% Set Steps
Ts = 0.05;
Tp = 40;
% Set Dimensions
nX = 4;
nU = 1;
nV = 1;
nS = 100;
% Set Weights
Qp = diag([   1    0    0    0]);    % Progress State Weights
Qt = diag([  10    0    0    0]);    % Terminal State Weights
Ru = diag([  10]);
Rd = diag([  10]);
% Set Constraints
Vector_Umax = [+0.60];    Vector_Dmax = [+0.20];
Vector_Umin = [-0.60];    Vector_Dmin = [-0.20];
% Set Ref Line
load('Scenarios\2004_mcity_line_04.mat');
% Set Vehicle
load('Models\Vehicles\CiDi\Shanqi_E9\Shanqi_E9_Calib.mat');
% Set Start Position
StartID = 1;
disp('Loading Time:');
disp(toc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Vectors & Matrices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic;
% Initialize Vectors
Vector_Sta = zeros(Tp,1);
Vector_Kpa = zeros(Tp,1);
Vector_Uop = zeros(Tp+1,nU);
% Initialize Matrices
Matrix_Xp = zeros(nX*(Tp+1),nS);        % Predicted States
Matrix_Up = zeros(nU*(Tp+1),nS);        % Predicted Inputs
Matrix_Vp = zeros(nV*(Tp+1),nS);
Matrix_Dp = zeros(nU*(Tp+1),nS);        % Predicted Deltas
Matrix_Jp = zeros(1,        nS);        % Predicted Costs
Matrix_Qp = zeros(nX*(Tp-1),nX*(Tp-1)); % Progress Weights
Matrix_Qt = zeros(   nX,    nX);        % Terminal Weights
Matrix_Ru = zeros(nU*Tp,nU*Tp);         % Input Weights
Matrix_Rd = zeros(nU*Tp,nU*Tp);         % Delta Weights
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Update States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Update Vehicle Parameters
Ms = Vehicle_Calib.Mass;    Iz = Vehicle_Calib.Inertia;
Lf = Vehicle_Calib.Lf;      Lr = Vehicle_Calib.Lr;
Cf = Vehicle_Calib.Cyf;     Cr = Vehicle_Calib.Cyr;
% Update States X(k)
x1 = +0.00;     % Linear Coordinate along X-axis
x2 = +0.00;     % Linear Coordinate along Y-axis
x3 = +0.00;     % Angular Coordinate around Z-axis
x4 = +0.00;     % Linear Velocity along X-axis
Vx = +3.00;
for i = 1:1:nS
    Matrix_Xp(1,i) = x1;
    Matrix_Xp(2,i) = x2;
    Matrix_Xp(3,i) = x3;
    Matrix_Xp(4,i) = x4;
end
% Update Inputs
u1 = +0.00;     % u1(k-1)
for i = 1:1:nS
    Matrix_Up(1,i) = u1;
end
% Update Disturbances
for i = 1:1:nS
    for j = 1:1:Tp+1
        S_prev = RefLineInfo.S(StartID) + (j-1)*Vx*Ts;
        Matrix_Vp(j,i) = interp1(RefLineInfo.S, RefLineInfo.Kpa, S_prev);
    end
end
% Update Weights
for i = 1:1:Tp
    if i ~= Tp
        Matrix_Qe(nX*(i-1)+1:nX*i,nX*(i-1)+1:nX*i) = Qp;
        Matrix_Ru(nU*(i-1)+1:nU*i,nU*(i-1)+1:nU*i) = Ru;
        Matrix_Rd(nU*(i-1)+1:nU*i,nU*(i-1)+1:nU*i) = Rd;
    else
        Matrix_Qe(nX*(i-1)+1:nX*i,nX*(i-1)+1:nX*i) = Qt;
        Matrix_Ru(nU*(i-1)+1:nU*i,nU*(i-1)+1:nU*i) = Ru;
        Matrix_Rd(nU*(i-1)+1:nU*i,nU*(i-1)+1:nU*i) = Rd;
    end
end
disp('Initiailzation Time:');
disp(toc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Generate Random Input Sequences %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic;
for i = 1:1:nS
    for j = 2:1:Tp+1
        for k = 1:1:nU
            Matrix_Up(nU*(j-1)+k,i) = ...
                Matrix_Up(nU*(j-2)+k,i) + normrnd(0,0.33*Vector_Dmax(k));
            Matrix_Dp(nU*(j-1)+k,i) = ...
                Matrix_Up(nU*(j-1)+k,i) - Matrix_Up(nU*(j-1)+k-1,i);
            % Saturation Check
            if Matrix_Up(nU*(j-1)+k,i) > Vector_Umax(k)
                Matrix_Up(nU*(j-1)+k,i) = Vector_Umax(k);
            end
            if Matrix_Up(nU*(j-1)+k,i) < Vector_Umin(k)
                Matrix_Up(nU*(j-1)+k,i) = Vector_Umin(k);
            end
            % Saturation Check Finished
        end
    end
end
disp('Input Sequences Generation Time:');
disp(toc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate State Sequences %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic;
for i = 1:1:nS
    for j = 2:1:Tp+1
        x1_last = Matrix_Xp(nX*(j-2)+1,i);
        x2_last = Matrix_Xp(nX*(j-2)+2,i);
        x3_last = Matrix_Xp(nX*(j-2)+3,i);
        x4_last = Matrix_Xp(nX*(j-2)+4,i);
        u1_last = Matrix_Up(nU*(j-2)+1,i);
        v1_last = Matrix_Vp(nV*(j-2)+1,i);
        x1_dot = x2_last;
        x2_dot = 2*x2_last*(Cf+Cr)/(Ms*Vx) - 2*x3_last*(Cf+Cr)/Ms + 2*x4_last*(Cf*Lf-Cr*Lr)/(Ms*Vx) - 2*u1_last*Cf/Ms + (2*(Cf*Lf-Cr*Lr)/Ms-Vx*Vx)*v1_last;
        x3_dot = x4_last;
        x4_dot = 2*x2_last*(Cf*Lf-Cr*Lr)/Iz*Vx - 2*x3_last*(Cf*Lf-Cr*Lr)/Iz + 2*x4_last*(Cf*Lf*Lf+Cr*Lr*Lr)/(Iz*Vx) - 2*u1_last*Cf*Lf/Iz + 2*v1_last*(Cf*Lf*Lf+Cr*Lr*Lr)/Iz;
        Matrix_Xp(nX*(j-1)+1,i) = x1_last + x1_dot*Ts;
        Matrix_Xp(nX*(j-1)+2,i) = x2_last + x2_dot*Ts;
        Matrix_Xp(nX*(j-1)+3,i) = x3_last + x3_dot*Ts;
        Matrix_Xp(nX*(j-1)+4,i) = x4_last + x4_dot*Ts;
    end
end
disp('State Prediction Time:');
disp(toc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate Cost %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic;
for i = 1:1:nS
    Matrix_Jp(1,i) = transpose(Matrix_Xp(nX+1:nX*(Tp+1),i))*Matrix_Qe*Matrix_Xp(nX+1:nX*(Tp+1),i) + ...
                     transpose(Matrix_Up(nU+1:nU*(Tp+1),i))*Matrix_Ru*Matrix_Up(nU+1:nU*(Tp+1),i) + ...
                     transpose(Matrix_Dp(nU+1:nU*(Tp+1),i))*Matrix_Rd*Matrix_Dp(nU+1:nU*(Tp+1),i);
end
minCostID = find(Matrix_Jp == min(min(Matrix_Jp)));
disp('Cost Calculation Time:')
disp(toc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Matrix_x1_Predict = zeros(Tp+1,nS);
Matrix_x2_Predict = zeros(Tp+1,nS);
Matrix_x3_Predict = zeros(Tp+1,nS);
Matrix_x4_Predict = zeros(Tp+1,nS);
Matrix_u1_Predict = zeros(Tp+1,nS);
Matrix_d1_Predict = zeros(Tp+1,nS);
subplot(2,3,1); hold on; title('Lateral Error');
subplot(2,3,4); hold on; title('Lateral Error Rate');
subplot(2,3,2); hold on; title('Heading Error');
subplot(2,3,5); hold on; title('Heading Error Rate');
subplot(2,3,3); hold on; title('Steer Angle');
subplot(2,3,6); hold on; title('Steer Rate');
for i = 1:1:nS
    for j = 1:1:Tp+1
        Matrix_x1_Predict(j,i) = Matrix_Xp(nX*(j-1)+1,i);
        Matrix_x2_Predict(j,i) = Matrix_Xp(nX*(j-1)+2,i);
        Matrix_x3_Predict(j,i) = Matrix_Xp(nX*(j-1)+3,i);
        Matrix_x4_Predict(j,i) = Matrix_Xp(nX*(j-1)+4,i);
        Matrix_u1_Predict(j,i) = Matrix_Up(nU*(j-1)+1,i);
    end
    for j = 2:1:Tp+1
        Matrix_d1_Predict(j,i) = Matrix_Up(nU*(j-1)+1,i) - Matrix_Up(nU*(j-1),i);
    end
    if i == minCostID
        subplot(2,3,1); plot(Matrix_x1_Predict(:,i),'r-','LineWidth',2);
        subplot(2,3,4); plot(Matrix_x2_Predict(:,i),'r-','LineWidth',2);
        subplot(2,3,2); plot(Matrix_x3_Predict(:,i),'r-','LineWidth',2);
        subplot(2,3,5); plot(Matrix_x4_Predict(:,i),'r-','LineWidth',2);
        subplot(2,3,3); plot(Matrix_u1_Predict(:,i),'r-','LineWidth',2);
        subplot(2,3,6); plot(Matrix_d1_Predict(:,i),'r-','LineWidth',2);
    else
        subplot(2,3,1); plot(Matrix_x1_Predict(:,i),'k:');
        subplot(2,3,4); plot(Matrix_x2_Predict(:,i),'k:');
        subplot(2,3,2); plot(Matrix_x3_Predict(:,i),'k:');
        subplot(2,3,5); plot(Matrix_x4_Predict(:,i),'k:');
        subplot(2,3,3); plot(Matrix_u1_Predict(:,i),'k:');
        subplot(2,3,6); plot(Matrix_d1_Predict(:,i),'k:');
    end
    pause(0.05);
end