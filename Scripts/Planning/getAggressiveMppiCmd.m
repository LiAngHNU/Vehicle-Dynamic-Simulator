%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Model Predictive Path Integral Planner %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Version: v1.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date: 2022/12/23 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Planner Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic;
% Set Steps
Ts = 1.00;
Tp =   40;
% Set Dimensions
nX = 8;
nU = 2;
nS = 100;
nL = 1001;
% Set Weights
Qe =     0;         ey_max = 4;
Qh =   100;
Qt =     0;
Qs =   -10;
Ru1 =    0;
Ru2 =   10;
Rd1 =    0;
Rd2 = 1000;
% Set Constraints
u1_max = +7500; u1_min = -7500; du1_max = +1000; du1_min = -1000;
u2_max = +0.60; u2_min = -0.60; du2_max = +0.20; du2_min = -0.20;
% Set Ref Line
load('Scenarios\2004_mcity_line_04.mat');
% Set Vehicle
load('Configs\Foton_Auman\Foton_Auman_Calib.mat');
% Set Tyre
load('Configs\Foton_Auman\Foton_Auman_Pacejka_Tyre.mat');
% Set Start Position
StartID = 2551;
disp('Loading Time:'); disp(toc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Vectors & Matrices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic;
% Initialize State Matrices
Mat_x1_Pre = zeros(1+Tp,nS);
Mat_x2_Pre = zeros(1+Tp,nS);
Mat_x3_Pre = zeros(1+Tp,nS);
Mat_x4_Pre = zeros(1+Tp,nS);
Mat_x5_Pre = zeros(1+Tp,nS);
Mat_x6_Pre = zeros(1+Tp,nS);
Mat_x7_Pre = zeros(1+Tp,nS);
Mat_x8_Pre = zeros(1+Tp,nS);
% Initialize Input Matrices
Mat_u1_Pre = zeros(1+Tp,nS);
Mat_u2_Pre = zeros(1+Tp,nS);
% Initialize Delta Matrices
Mat_d1_Pre = zeros(1+Tp,nS);
Mat_d2_Pre = zeros(1+Tp,nS);
% Initialize Cost Vector
Vec_E_Lat_Tmp = zeros(nL,1);
Mat_E_Lat_Pre = zeros(1+Tp,nS);
Mat_E_Hea_Pre = zeros(1+Tp,nS);
Mat_Sc_Pre = zeros(1+Tp,nS);
Vec_Je = zeros(1,nS);
Vec_Jh = zeros(1,nS);
Vec_Js = zeros(1,nS);
Vec_Ju = zeros(1,nS);
Vec_Jd = zeros(1,nS);
Vec_Jp = zeros(1,nS);
for i = 1:1:nS
    Mat_Sc_Pre(1,i) = RefLineInfo.S(StartID);
end
disp('Initialization Time:'); disp(toc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Update States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic;
% Update Vehicle Paramters
Ms = Vehicle_Calib.Ms;      Iz = Vehicle_Calib.Iz;
Lf = Vehicle_Calib.Lf;      Lr = Vehicle_Calib.Lr;              Re = 0.51;
Byf = Tyre_Calib.Lat_Front_Coeffs(1);
Cyf = Tyre_Calib.Lat_Front_Coeffs(2);
Dyf = Tyre_Calib.Lat_Front_Coeffs(3);
Byr = Tyre_Calib.Lat_Rear_Coeffs(1);
Cyr = Tyre_Calib.Lat_Rear_Coeffs(2);
Dyr = Tyre_Calib.Lat_Rear_Coeffs(3);
% Update States
x1 = RefLineInfo.X(StartID);    x4 = +7.50;
x2 = RefLineInfo.Y(StartID);    x5 = +0.00;
x3 = RefLineInfo.Tht(StartID);  x6 = +0.00;
x7 = +0.00;                     u1 = +0.00;
x8 = -0.30;                     u2 = -0.20;
for i = 1:1:nS
    Mat_x1_Pre(1,i) = x1;
    Mat_x2_Pre(1,i) = x2;
    Mat_x3_Pre(1,i) = x3;
    Mat_x4_Pre(1,i) = x4;
    Mat_x5_Pre(1,i) = x5;
    Mat_x6_Pre(1,i) = x6;
    Mat_x7_Pre(1,i) = x7;
    Mat_x8_Pre(1,i) = x8;
end
% Update Inputs
for i = 1:1:nS
    Mat_u1_Pre(1,i) = u1; 
    Mat_u2_Pre(1,i) = u2;
end
% Update Inputs Exp & Var
u1_var = +1.00*du1_max;
u2_var = +0.66*du2_max;
% Update Local Reference Line
Sc = RefLineInfo.S(StartID);
Local_S = Sc:0.25:Sc+50;
Local_X = interp1(RefLineInfo.S,RefLineInfo.X,Local_S,'linear');
Local_Y = interp1(RefLineInfo.S,RefLineInfo.Y,Local_S,'linear');
Local_Tht = interp1(RefLineInfo.S,RefLineInfo.Tht,Local_S,'linear');
Local_Kpa = interp1(RefLineInfo.S,RefLineInfo.Kpa,Local_S,'linear');
disp('Update Time:'); disp(toc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Generate Random Input Sequences %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                                                                      
% Generate Commands
for i = 1:1:nS
    for j = 2:1:1+Tp
        Mat_u1_Pre(j,i) = Mat_u1_Pre(j-1,i) + normrnd(0,u1_var);
        Mat_u2_Pre(j,i) = Mat_u2_Pre(j-1,i) + normrnd(0,u2_var);
        if Mat_u1_Pre(j,i) > u1_max
            Mat_u1_Pre(j,i) = u1_max;
        end
        if Mat_u1_Pre(j,i) < u1_min
            Mat_u1_Pre(j,i) = u1_min;
        end
        if Mat_u2_Pre(j,i) > u2_max
            Mat_u2_Pre(j,i) = u2_max;
        end
        if Mat_u2_Pre(j,i) < u2_min
            Mat_u2_Pre(j,i) = u2_min;
        end
    end
end
% Calculate Delayed Commands
Mat_u1_Pre = [u1*ones(Td,nS);Mat_u1_Pre]; Mat_u1_Pre(2+Tp:1+Td+Tp,:) = [];
Mat_u2_Pre = [u2*ones(Td,nS);Mat_u2_Pre]; Mat_u2_Pre(2+Tp:1+Td+Tp,:) = [];
disp('Input Sequences Generation Time:'); disp(toc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate State Sequences %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:nS
    for j = 2:1:1+Tp
        SF = (1-Kr*Ey)/(Vx*cos(Ephi) - Vy*sin(Ephi));
        % Calculate Slip Angle
        alphaF = (Mat_x5_Pre(j-1,i)+Lf*Mat_x6_Pre(j-1,i))/Mat_x4_Pre(j-1,i) - Mat_x8_Pre(j-1,i);
        alphaR = (Mat_x5_Pre(j-1,i)-Lr*Mat_x6_Pre(j-1,i))/Mat_x4_Pre(j-1,i);
        % Calculate Tyre Force
        Fxr = 4*Mat_x7_Pre(j-1,i)/Re;
        Fyf = Dyf*sin(Cyf*atan(Byf*alphaF));
        Fyr = Dyr*sin(Cyr*atan(Byr*alphaR));
        % Calculate State Derivatives
        x1_dot = Mat_x4_Pre(j-1,i)*cos(Mat_x3_Pre(j-1,i)) - Mat_x5_Pre(j-1,i)*sin(Mat_x3_Pre(j-1,i));
        x2_dot = Mat_x4_Pre(j-1,i)*sin(Mat_x3_Pre(j-1,i)) + Mat_x5_Pre(j-1,i)*cos(Mat_x3_Pre(j-1,i));
        x3_dot = Mat_x6_Pre(j-1,i);
        x4_dot = (Fxr - Fyf*sin(Mat_x8_Pre(j-1,i)) + Ms*Mat_x5_Pre(j-1,i)*Mat_x6_Pre(j-1,i))/Ms;
        x5_dot = (Fyr + Fyf*cos(Mat_x8_Pre(j-1,i)) - Ms*Mat_x4_Pre(j-1,i)*Mat_x6_Pre(j-1,i))/Ms;
        x6_dot = (Fyf*Lf*cos(Mat_x8_Pre(j-1,i)) - Fyr*Lr)/Iz;
        x7_dot = (-1/Tl)*Mat_x7_Pre(j-1,i) + (1/Tl)*Mat_u1_Pre(j-1,i);
        x8_dot = (-1/Tl)*Mat_x8_Pre(j-1,i) + (1/Tl)*Mat_u2_Pre(j-1,i);
        % Calculate State Sequences
        Mat_x1_Pre(j,i) = Mat_x1_Pre(j-1,i) + x1_dot*Ts;
        Mat_x2_Pre(j,i) = Mat_x2_Pre(j-1,i) + x2_dot*Ts;
        Mat_x3_Pre(j,i) = Mat_x3_Pre(j-1,i) + x3_dot*Ts;
        Mat_x4_Pre(j,i) = Mat_x4_Pre(j-1,i) + x4_dot*Ts;
        Mat_x5_Pre(j,i) = Mat_x5_Pre(j-1,i) + x5_dot*Ts;
        Mat_x6_Pre(j,i) = Mat_x6_Pre(j-1,i) + x6_dot*Ts;
        Mat_x7_Pre(j,i) = Mat_x7_Pre(j-1,i) + x7_dot*Ts;
        Mat_x8_Pre(j,i) = Mat_x8_Pre(j-1,i) + x8_dot*Ts;
        % Calculate Delta Sequences
        Mat_d1_Pre(j,i) = Mat_x7_Pre(j,i) - Mat_x7_Pre(j-1,i);
        Mat_d2_Pre(j,i) = Mat_x8_Pre(j,i) - Mat_x8_Pre(j-1,i);
    end
end
disp('State Prediction Time:'); disp(toc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate Cost %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate Station & Error
for i = 1:1:nS
    for j = 2:1:1+Tp
        Vec_E_Lat_Tmp = (Mat_x1_Pre(j,i) - Local_X).^2 + (Mat_x2_Pre(j,i) - Local_Y).^2;
        ID = find(Vec_E_Lat_Tmp == min(Vec_E_Lat_Tmp));
        Mat_E_Lat_Pre(j,i) = Vec_E_Lat_Tmp(ID(1));
        Mat_E_Hea_Pre(j,i) = Mat_x3_Pre(j,i) - Local_Tht(ID(1));
        Mat_Sc_Pre(j,i) = Local_S(ID(1));
    end
end
% Calculate Lateral Error Cost
for i = 1:1:nS
    for j = 2:1:1+Tp
        if Mat_E_Lat_Pre(j,i) < ey_max
            Vec_Je(1,i) = Vec_Je(1,i) + 0;
        else
            Vec_Je(1,i) = Vec_Je(1,i) + 1e3 + Mat_E_Lat_Pre(j,i)^2;
        end
    end
end
% Calculate Heading Error Cost
for i = 1:1:nS
    Vec_Jh(1,i) = Qh*Mat_E_Hea_Pre(1+Tp,i)^2;
end
% Calculate Driving Mileage Cost
for i = 1:1:nS
    Vec_Js(1,i) = Qs*(Mat_Sc_Pre(1+Tp,i)-Mat_Sc_Pre(1,i));
end
% Calculate Sequential Trajectory Consistency Cost
%
%
% Calculate Inputs Cost
for i = 1:1:nS
    for j = 2:1:1+Tp
        Vec_Ju(1,i) = Vec_Ju(1,i) + Ru1*Mat_u1_Pre(j,i)^2 + Ru2*Mat_u2_Pre(j,i)^2;
    end
end
% Calculate Deltas Cost
for i = 1:1:nS
    for j = 2:1:1+Tp
        Vec_Jd(1,i) = Vec_Jd(1,i) + Rd1*Mat_d1_Pre(j,i)^2 + Rd2*Mat_d2_Pre(j,i)^2;
    end
end
% Calculate Total Cost
Vec_Jp = Vec_Je + Vec_Jh + Vec_Js + Vec_Ju + Vec_Jd;
% Find MinCost Trajectory
minIdex = find(Vec_Jp == min(Vec_Jp));
minCost = Vec_Jp(minIdex);
for i = 1:1:nS
    omg(i) = exp(-(Vec_Jp(i)-minCost)/lam);
end
omg = omg/sum(omg);
Vec_u1_Opt = Mat_u1_Pre*omg;
Vec_u2_Opt = Mat_u2_Pre*omg;
disp('Cost Calculation Time:'); disp(toc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,4,1);hold on; title('Coordinates'); axis equal;
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
    if i == minIdex
        subplot(2,4,1); plot(Mat_x1_Pre(:,i),Mat_x2_Pre(:,i),'g-','LineWidth',2);
        subplot(2,4,2); plot(Mat_x4_Pre(:,i),'g-','LineWidth',2);
        subplot(2,4,3); plot(Mat_x7_Pre(:,i),'g-','LineWidth',2);
        subplot(2,4,4); plot(Mat_Sc_Pre(:,i),'g-','LineWidth',2);
        subplot(2,4,5); plot(Mat_x6_Pre(:,i),'g-','LineWidth',2);
        subplot(2,4,6); plot(Mat_x5_Pre(:,i),'g-','LineWidth',2);
        subplot(2,4,7); plot(Mat_x8_Pre(:,i),'g-','LineWidth',2);
        subplot(2,4,8); plot(sqrt(Mat_E_Lat_Pre(:,i)),'g-','LineWidth',2);
    else
        subplot(2,4,1); plot(Mat_x1_Pre(:,i),Mat_x2_Pre(:,i),'k:');
        subplot(2,4,2); plot(Mat_x4_Pre(:,i),'k:');
        subplot(2,4,3); plot(Mat_x7_Pre(:,i),'k:');
        subplot(2,4,4); plot(Mat_Sc_Pre(:,i),'k:');
        subplot(2,4,5); plot(Mat_x6_Pre(:,i),'k:');
        subplot(2,4,6); plot(Mat_x5_Pre(:,i),'k:');
        subplot(2,4,7); plot(Mat_x8_Pre(:,i),'k:');
        subplot(2,4,8); plot(sqrt(Mat_E_Lat_Pre(:,i)),'k:');
    end
    pause(0.02);
end