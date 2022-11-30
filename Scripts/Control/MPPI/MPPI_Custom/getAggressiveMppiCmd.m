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
Ts = 0.03;
Tp = 60;
% Set Dimensions
nX = 7;
nU = 2;
nV = 1;
nS = 500;
% Set Weights
Qe =    1;
Qp =   10;
Qt = 1000;
Qs = -100;
% Set Constraints
Vector_Umax = [+10000;+0.60];    Vector_Dmax = [+10000;+0.20];
Vector_Umin = [-10000;-0.60];    Vector_Dmin = [-10000;-0.20];
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
Ms = Vehicle_Calib.Mass;    Iz = Vehicle_Calib.Inertia;
Lf = Vehicle_Calib.Lf;      Lr = Vehicle_Calib.Lr;
Cf = Vehicle_Calib.Cyf;     Cr = Vehicle_Calib.Cyr;
Re = 0.51;
% Update States X(k)
x1 = RefLineInfo.X(StartID);    % Linear Coordinate along X-axis
x2 = RefLineInfo.Y(StartID);    % Linear Coordinate along Y-axis
x3 = RefLineInfo.Tht(StartID);  % Angular Coordinate around Z-axis
x4 = +5.00;                     % Linear Velocity along X-axis
x5 = +0.00;                     % Linear Velocity along Y-axis
x6 = +0.00;                     % Angular Velocity around Z-axis
x7 = RefLineInfo.S(StartID);    % Linear Coordinate along S-axis
for i = 1:1:nS
    Matrix_Xp(1,i) = x1;
    Matrix_Xp(2,i) = x2;
    Matrix_Xp(3,i) = x3;
    Matrix_Xp(4,i) = x4;
    Matrix_Xp(5,i) = x5;
    Matrix_Xp(6,i) = x6;
    Matrix_Xp(7,i) = x7;
end
% Update Inputs
u1 = +0.00;     % u1(k-1)
u2 = -0.20;
for i = 1:1:nS
    Matrix_Up(1,i) = u1;
    Matrix_Up(2,i) = u2;
end
disp('Initialization Time:');
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
        x5_last = Matrix_Xp(nX*(j-2)+5,i);
        x6_last = Matrix_Xp(nX*(j-2)+6,i);
        x7_last = Matrix_Xp(nX*(j-2)+7,i);
        u1_last = Matrix_Up(nU*(j-2)+1,i);
        u2_last = Matrix_Up(nU*(j-2)+2,i);
        alpha_f = (x5_last + Lf*x6_last)/x4_last - u2_last;
        alpha_r = (x5_last - Lr*x6_last)/x4_last;
        x1_ref = interp1(RefLineInfo.S,RefLineInfo.X,x7_last,'linear','extrap');
        x2_ref = interp1(RefLineInfo.S,RefLineInfo.Y,x7_last,'linear','extrap');
        x3_ref = interp1(RefLineInfo.S,RefLineInfo.Tht,x7_last,'linear','extrap');
        kr_ref = interp1(RefLineInfo.S,RefLineInfo.Kpa,x7_last,'linear','extrap');
        lateral_error = sqrt((x1_last-x1_ref)^2 + (x2_last-x2_ref)^2);
        heading_error = x3_last - x3_ref;
        x1_dot = x4_last*cos(x3_last) - x5_last*sin(x3_last);
        x2_dot = x4_last*sin(x3_last) + x5_last*cos(x3_last);
        x3_dot = x6_last;
        x4_dot = (1/Ms)*(u1_last/Re - Cf*alpha_f*sin(u2_last) + Ms*x5_last*x6_last);
        x5_dot = (1/Ms)*(Cf*alpha_f*cos(u2_last) + Cr*alpha_r - Ms*x4_last*x6_last);
        x6_dot = (1/Iz)*(Cf*Lf*alpha_f*cos(u2_last) - Cr*Lr*alpha_r);
        x7_dot = (x4*cos(heading_error) - x5*sin(heading_error))/(1 - kr_ref*lateral_error);
        Matrix_Xp(nX*(j-1)+1,i) = x1_last + x1_dot*Ts;
        Matrix_Xp(nX*(j-1)+2,i) = x2_last + x2_dot*Ts;
        Matrix_Xp(nX*(j-1)+3,i) = x3_last + x3_dot*Ts;
        Matrix_Xp(nX*(j-1)+4,i) = x4_last + x4_dot*Ts;
        Matrix_Xp(nX*(j-1)+5,i) = x5_last + x5_dot*Ts;
        Matrix_Xp(nX*(j-1)+6,i) = x6_last + x6_dot*Ts;
        Matrix_Xp(nX*(j-1)+7,i) = x7_last + x7_dot*Ts;
        Matrix_Lateral_Error(j,i) = lateral_error;
        Matrix_Heading_Error(j,i) = heading_error;
    end
end
disp('State Prediction Time:');
disp(toc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate Cost %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic;
for i = 1:1:nS
    for j = 2:1:Tp
        Matrix_Jp(1,i) = Matrix_Lateral_Error(j-1,i)+...
            Matrix_Lateral_Error(j,i)*Qe*Matrix_Lateral_Error(j,i)+...
            Matrix_Heading_Error(j,i)*Qp*Matrix_Heading_Error(j,i);
    end
    Matrix_Jp(1,i) = Matrix_Jp(1,i) + Matrix_Lateral_Error(Tp+1,i)*Qt*Matrix_Lateral_Error(Tp+1,i);
end
minCostID = find(Matrix_Jp==min(Matrix_Jp));
disp('Cost Calculation Time:')
disp(toc);
% U_opt = zeros(nU,Tp);
% rho = min(Matrix_Jp);
% eta = exp(-1/Lambda*(Matrix_Jp-rho));
% phi = eta/sum(eta);
% for i = 1:1:nS
%     U_opt = U_opt + phi(i)*Matrix_Up(i,1);
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Post Process %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Vector_S = zeros(length(RefLineInfo.Id),1);
for i = 1:1:length(RefLineInfo.Id)
    Vector_S(i) = sqrt((306.337 - RefLineInfo.X(i))^2+(-364.978 - RefLineInfo.Y(i))^2);
end
minDistanceID = find(Vector_S==min(Vector_S));
minDistance = sqrt((306.337 - RefLineInfo.X(minDistanceID))^2+(-364.978 - RefLineInfo.Y(minDistanceID))^2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Matrix_x1_Predict = zeros(Tp+1,nS);
Matrix_x2_Predict = zeros(Tp+1,nS);
Matrix_x3_Predict = zeros(Tp+1,nS);
Matrix_x4_Predict = zeros(Tp+1,nS);
Matrix_x5_Predict = zeros(Tp+1,nS);
Matrix_x6_Predict = zeros(Tp+1,nS);
Matrix_x7_Predict = zeros(Tp+1,nS);
Matrix_u1_Predict = zeros(Tp+1,nS);
Matrix_u2_Predict = zeros(Tp+1,nS);
Matrix_d1_Predict = zeros(Tp+1,nS);
Matrix_d2_Predict = zeros(Tp+1,nS);
subplot(2,4,1);hold on; title('Coordinates');
plot(RefLineInfo.X,RefLineInfo.Y,'r--');
plot(RefLineInfo.Xl,RefLineInfo.Yl,'k');
plot(RefLineInfo.Xr,RefLineInfo.Yr,'k');
subplot(2,4,2); hold on; title('Vx');
subplot(2,4,3); hold on; title('Torque');
subplot(2,4,4); hold on; title('Station');
subplot(2,4,5); hold on; title('Yaw Rate');
subplot(2,4,6); hold on; title('Vy');
subplot(2,4,7); hold on; title('Steer Angle');
subplot(2,4,8); hold on; title('Lateral Error');
for i = 1:1:nS
    for j = 1:1:Tp+1
        Matrix_x1_Predict(j,i) = Matrix_Xp(nX*(j-1)+1,i);
        Matrix_x2_Predict(j,i) = Matrix_Xp(nX*(j-1)+2,i);
        Matrix_x3_Predict(j,i) = Matrix_Xp(nX*(j-1)+3,i);
        Matrix_x4_Predict(j,i) = Matrix_Xp(nX*(j-1)+4,i);
        Matrix_x5_Predict(j,i) = Matrix_Xp(nX*(j-1)+5,i);
        Matrix_x6_Predict(j,i) = Matrix_Xp(nX*(j-1)+6,i);
        Matrix_x7_Predict(j,i) = Matrix_Xp(nX*(j-1)+7,i);
        Matrix_u1_Predict(j,i) = Matrix_Up(nU*(j-1)+1,i);
        Matrix_u2_Predict(j,i) = Matrix_Up(nU*(j-1)+2,i);
    end
    for j = 2:1:Tp+1
        Matrix_d1_Predict(j,i) = Matrix_Up(nU*(j-1)+1,i) - Matrix_Up(nU*(j-1),i);
        Matrix_d2_Predict(j,i) = Matrix_Up(nU*(j-1)+2,i) - Matrix_Up(nU*(j-1),i);
    end
    if i == minCostID
        subplot(2,4,1); plot(Matrix_x1_Predict(:,i),Matrix_x2_Predict(:,i),'g-','LineWidth',2);
        subplot(2,4,2); plot(Matrix_x4_Predict(:,i),'g-','LineWidth',2);
        subplot(2,4,3); plot(Matrix_u1_Predict(:,i),'g-','LineWidth',2);
        subplot(2,4,4); plot(Matrix_x7_Predict(:,i),'g-','LineWidth',2);
        subplot(2,4,5); plot(Matrix_x6_Predict(:,i),'g-','LineWidth',2);
        subplot(2,4,6); plot(Matrix_x5_Predict(:,i),'g-','LineWidth',2);
        subplot(2,4,7); plot(Matrix_u2_Predict(:,i),'g-','LineWidth',2);
        subplot(2,4,8); plot(Matrix_Lateral_Error(:,i),'g-','LineWidth',2);
    else
        subplot(2,4,1); plot(Matrix_x1_Predict(:,i),Matrix_x2_Predict(:,i),'k:');
        subplot(2,4,2); plot(Matrix_x4_Predict(:,i),'k:');
        subplot(2,4,3); plot(Matrix_u1_Predict(:,i),'k:');
        subplot(2,4,4); plot(Matrix_x7_Predict(:,i),'k:');
        subplot(2,4,5); plot(Matrix_x6_Predict(:,i),'k:');
        subplot(2,4,6); plot(Matrix_x5_Predict(:,i),'k:');
        subplot(2,4,7); plot(Matrix_u2_Predict(:,i),'k:');
        subplot(2,4,8); plot(Matrix_Lateral_Error(:,i),'k:');
    end
    pause(0.02);
end