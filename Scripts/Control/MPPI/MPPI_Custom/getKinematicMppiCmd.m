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
% Set Steps
Ts = 0.05;
Tp = 100;
% Set Dimensions
nX = 3;
nU = 2;
nV = 1;
nS = 100;
% Set Weights
Qe =    1;
Qp =   10;
Qt = 1000;
Qs = -100;
% Set Constraints
Vector_Umax = [+ 5;+0.60];    Vector_Dmax = [+   5;+0.20];
Vector_Umin = [+ 1;-0.60];    Vector_Dmin = [-   5;-0.20];
% Set Ref Line
load('Scenarios\2004_mcity_line_04.mat');
% Set Vehicle
load('Models\Vehicles\CiDi\Shanqi_E9\Shanqi_E9_Calib.mat');
% Set Start Position
StartID = 550;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Vectors & Matrices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize Vectors
Vector_Sta = zeros(Tp,1);
Vector_Kpa = zeros(Tp,1);
Vector_Uop = zeros(Tp+1,nU);
% Initialize Matrices
Matrix_Xp = zeros(nX*(Tp+1),nS);        % Predicted States
Matrix_Up = zeros(nU*(Tp+1),nS);        % Predicted Inputs
Matrix_Dp = zeros(nU*(Tp+1),nS);        % Predicted Deltas
Matrix_Jp = zeros(1,        nS);        % Predicted Costs
Matrix_Qp = zeros(nX*(Tp-1),nX*(Tp-1)); % Progress Weights
Matrix_Qt = zeros(   nX,    nX);        % Terminal Weights
Matrix_Ru = zeros(nU*Tp,nU*Tp);         % Input Weights
Matrix_Rd = zeros(nU*Tp,nU*Tp);         % Delta Weights
Matrix_Lateral_Error = zeros(Tp+1,nS);
Matrix_Heading_Error = zeros(Tp+1,nS);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Update States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Update Vehicle Parameters
Lf = Vehicle_Calib.Lf;      Lr = Vehicle_Calib.Lr;
% Update States X(k)
x1 = RefLineInfo.X(StartID);    % Linear Coordinate along X-axis
x2 = RefLineInfo.Y(StartID);    % Linear Coordinate along Y-axis
x3 = RefLineInfo.Tht(StartID);  % Angular Coordinate around Z-axis
for i = 1:1:nS
    Matrix_Xp(1,i) = x1;
    Matrix_Xp(2,i) = x2;
    Matrix_Xp(3,i) = x3;
end
% Update Inputs
u1 = +2.00;     % u1(k-1)
u2 = -0.20;
for i = 1:1:nS
    Matrix_Up(1,i) = u1;
    Matrix_Up(2,i) = u2;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Generate Random Input Sequences %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:nS
    for j = 2:1:Tp+1
        for k = 1:1:nU
            Matrix_Up(nU*(j-1)+k,i) = ...
                Matrix_Up(nU*(j-2)+k,i) + normrnd(0,0.33*Vector_Dmax(k));
            % Saturation Check
            if Matrix_Up(nU*(j-1)+k,i) > Vector_Umax(k)
                Matrix_Up(nU*(j-1)+k,i) = Vector_Umax(k);
            end
            if Matrix_Up(nU*(j-1)+k,i) < Vector_Umin(k)
                Matrix_Up(nU*(j-1)+k,i) = Vector_Umin(k);
            end
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate State Sequences %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:nS
    for j = 2:1:Tp+1
        x1_last = Matrix_Xp(nX*(j-2)+1,i);
        x2_last = Matrix_Xp(nX*(j-2)+2,i);
        x3_last = Matrix_Xp(nX*(j-2)+3,i);
        u1_last = Matrix_Up(nU*(j-2)+1,i);
        u2_last = Matrix_Up(nU*(j-2)+2,i);
        x1_dot = u1_last*cos(u2_last + x3_last);
        x2_dot = u1_last*sin(u2_last + x3_last);
        x3_dot = u1_last*sin(u2_last)/(Lf+Lr);
        Matrix_Xp(nX*(j-1)+1,i) = x1_last + x1_dot*Ts;
        Matrix_Xp(nX*(j-1)+2,i) = x2_last + x2_dot*Ts;
        Matrix_Xp(nX*(j-1)+3,i) = x3_last + x3_dot*Ts;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate Cost %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Matrix_x1_Predict = zeros(Tp+1,nS);
Matrix_x2_Predict = zeros(Tp+1,nS);
Matrix_x3_Predict = zeros(Tp+1,nS);
Matrix_u1_Predict = zeros(Tp+1,nS);
Matrix_u2_Predict = zeros(Tp+1,nS);
Matrix_d1_Predict = zeros(Tp+1,nS);
Matrix_d2_Predict = zeros(Tp+1,nS);
subplot(2,2,1);hold on; title('Coordinates');
plot(RefLineInfo.X,RefLineInfo.Y,'r--');
plot(RefLineInfo.Xl,RefLineInfo.Yl,'k');
plot(RefLineInfo.Xr,RefLineInfo.Yr,'k');
subplot(2,2,2); hold on; title('Vx');
subplot(2,2,4); hold on; title('Steer Angle');
for i = 1:1:nS
    for j = 1:1:Tp+1
        Matrix_x1_Predict(j,i) = Matrix_Xp(nX*(j-1)+1,i);
        Matrix_x2_Predict(j,i) = Matrix_Xp(nX*(j-1)+2,i);
        Matrix_x3_Predict(j,i) = Matrix_Xp(nX*(j-1)+3,i);
        Matrix_u1_Predict(j,i) = Matrix_Up(nU*(j-1)+1,i);
        Matrix_u2_Predict(j,i) = Matrix_Up(nU*(j-1)+2,i);
    end
    for j = 2:1:Tp+1
        Matrix_d1_Predict(j,i) = Matrix_Up(nU*(j-1)+1,i) - Matrix_Up(nU*(j-1),i);
        Matrix_d2_Predict(j,i) = Matrix_Up(nU*(j-1)+2,i) - Matrix_Up(nU*(j-1),i);
    end
        subplot(2,2,1); plot(Matrix_x1_Predict(:,i),Matrix_x2_Predict(:,i),'k:');
        subplot(2,2,2); plot(Matrix_u1_Predict(:,i),'k:');
        subplot(2,2,4); plot(Matrix_u2_Predict(:,i),'k:');
    pause(0.02);
end