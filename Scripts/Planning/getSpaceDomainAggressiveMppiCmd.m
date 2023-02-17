%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Model Predictive Path Integral Planner (Space-Domain) %%%%%%%%%%%%%%%%%%%
% Version: v1.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date: 2023/01/04 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Planner Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic;
% Set Steps
dS =  0.25;             % Number of Sampled Intervals
Tp =   100;             % Number of Previewed Reference Line Points
% Set Dimensions
nS =     2;             % Number of Sampled Trajectories
% Set Weights
Qe =     1;             % Weight of Lateral Error
Qh =     5;             % Weight of Heading Error
Qt =     0;             % Weight of Terminal Error
Qs =   100;             % Weight of Driving Mileage
Qc =     0;             % Weight of Trajectory Consistency
Ru1 =    0;             % Weight of Longitudinal Input
Ru2 =   10;             % Weight of Lateral Input
Rd1 =    0;             % Weight of Longitudinal Input Rate
Rd2 =  100;             % Weight of Lateral Input Rate
% Set Constraints
u1_max = + 2e4; u1_min = - 2e4; du1_max = +7500; du1_min = -7500;
u2_max = +0.60; u2_min = -0.60; du2_max = +0.20; du2_min = -0.20;
% Set Ref Line
load('Scenarios\2004_mcity_line_04.mat');
% Set Vehicle
load('Configs\Foton_Auman\Foton_Auman_Calib.mat');
% Set Tyre
load('Configs\Foton_Auman\Foton_Auman_Pacejka_Tyre.mat');
% Set Start Position
StartID = 2751;
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
Mat_ts_Pre = zeros(1+Tp,nS);
Mat_Xv_Pre = zeros(1+Tp,nS);
Mat_Yv_Pre = zeros(1+Tp,nS);
% Initialize Input Matrices
Mat_u1_Pre = zeros(1+Tp,nS);
Mat_u2_Pre = zeros(1+Tp,nS);
% Initialize Delta Matrices
Mat_d1_Pre = zeros(1+Tp,nS);
Mat_d2_Pre = zeros(1+Tp,nS);
% Initialize Noise Matrices
Mat_v1_Pre = zeros(1+Tp,nS);
% Initialize Cost Vector
Vec_Je = zeros(1,nS);
Vec_Jh = zeros(1,nS);
Vec_Js = zeros(1,nS);
Vec_Ju = zeros(1,nS);
Vec_Jd = zeros(1,nS);
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
x1 = +5.00;
x2 = +0.00;
x3 = +0.00;
x4 = +0.00;
x5 = +0.00;
for i = 1:1:nS
    Mat_x1_Pre(1,i) = x1;
    Mat_x2_Pre(1,i) = x2;
    Mat_x3_Pre(1,i) = x3;
    Mat_x4_Pre(1,i) = x4;
    Mat_x5_Pre(1,i) = x5;
    Mat_Xv_Pre(1,i) = RefLineInfo.X(StartID);
    Mat_Yv_Pre(1,i) = RefLineInfo.Y(StartID);
end
% Update Inputs
u1 = +0.00;
u2 = +0.00;
for i = 1:1:nS
    Mat_u1_Pre(1,i) = u1; 
    Mat_u2_Pre(1,i) = u2;
end
% Update Inputs Exp & Var
u1_var = +1.00*du1_max;
u2_var = +0.33*du2_max;
% Update Local Reference Line
Local_S = RefLineInfo.S(StartID):dS:RefLineInfo.S(StartID)+Tp*dS;
Local_X = interp1(RefLineInfo.S,RefLineInfo.X,Local_S,'linear');
Local_Y = interp1(RefLineInfo.S,RefLineInfo.Y,Local_S,'linear');
Local_Tht = interp1(RefLineInfo.S,RefLineInfo.Tht,Local_S,'linear');
Local_Kpa = interp1(RefLineInfo.S,RefLineInfo.Kpa,Local_S,'linear');
% Update Kappas
for i = 1:1:nS
    for j = 1:1:Tp+1
        Mat_v1_Pre(j,i) = Local_Kpa(j);
    end
end
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
disp('Input Sequences Generation Time:'); disp(toc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate State Sequences %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:nS
    for j = 2:1:1+Tp
        % Calculate Time-Domain to Space-Domain Transformation Coefficient
        SF = (1-Mat_v1_Pre(j-1,i)*Mat_x4_Pre(j-1,i))/(Mat_x1_Pre(j-1,i)*cos(Mat_x5_Pre(j-1,i))-Mat_x2_Pre(j-1,i)*sin(Mat_x5_Pre(j-1,i)));
        % Calculate Slip Angle
        alphaF = (Mat_x2_Pre(j-1,i)+Lf*Mat_x3_Pre(j-1,i))/Mat_x1_Pre(j-1,i) - Mat_u2_Pre(j-1,i);
        alphaR = (Mat_x2_Pre(j-1,i)-Lr*Mat_x3_Pre(j-1,i))/Mat_x1_Pre(j-1,i);
        % Calculate Tyre Force
        Fxr = Mat_u1_Pre(j-1,i)/Re;
        Fyf = Dyf*sin(Cyf*atan(Byf*alphaF));
        Fyr = Dyr*sin(Cyr*atan(Byr*alphaR));
        % Calculate State Derivatives
        x1_dot = SF*(Fxr - Fyf*sin(Mat_u2_Pre(j-1,i)) + Ms*Mat_x2_Pre(j-1,i)*Mat_x3_Pre(j-1,i))/Ms;
        x2_dot = SF*(Fyr + Fyf*cos(Mat_u2_Pre(j-1,i)) - Ms*Mat_x1_Pre(j-1,i)*Mat_x3_Pre(j-1,i))/Ms;
        x3_dot = SF*(Fyf*Lf*cos(Mat_u2_Pre(j-1,i)) - Fyr*Lr)/Iz;
        x4_dot = SF*(Mat_x1_Pre(j-1,i)*sin(Mat_x5_Pre(j-1,i)+Mat_x2_Pre(j-1,i)*cos(Mat_x5_Pre(j-1,i))));
        x5_dot = SF*(Mat_x3_Pre(j-1,i)-Mat_x1_Pre(j-1,i)*Mat_v1_Pre(j-1,i));
        % Calculate State Sequences
        Mat_x1_Pre(j,i) = Mat_x1_Pre(j-1,i) + x1_dot*dS;
        Mat_x2_Pre(j,i) = Mat_x2_Pre(j-1,i) + x2_dot*dS;
        Mat_x3_Pre(j,i) = Mat_x3_Pre(j-1,i) + x3_dot*dS;
        Mat_x4_Pre(j,i) = Mat_x4_Pre(j-1,i) + x4_dot*dS;
        Mat_x5_Pre(j,i) = Mat_x5_Pre(j-1,i) + x5_dot*dS;
        % Velocity Check
        if Mat_x1_Pre(j,i) < 2.5
            Mat_x1_Pre(j,i) = 2.5;
        end
        % Calculate Speed Sequences
        Mat_ts_Pre(j,i) = SF;
        % Calculate Delta Sequences
        Mat_d1_Pre(j,i) = Mat_u1_Pre(j,i) - Mat_u1_Pre(j-1,i);
        Mat_d2_Pre(j,i) = Mat_u2_Pre(j,i) - Mat_u2_Pre(j-1,i);
    end
end
disp('State Prediction Time:'); disp(toc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate Cost %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate Lateral Error Cost
for i = 1:1:nS
    for j = 2:1:1+Tp
        if abs(Mat_x4_Pre(j,i)) < 2.00
            Vec_Je(1,i) = Vec_Je(1,i) + Qe*Mat_x4_Pre(j,i)^2;
        else
            Vec_Je(1,i) = Vec_Je(1,i) + Qe*Mat_x4_Pre(j,i)^2 + 1e4;
        end
    end
end
% Calculate Heading Error Cost
for i = 1:1:nS
    for j = 2:1:1+Tp
        Vec_Jh(1,i) = Vec_Jh(1,i) + Qh*Mat_x5_Pre(j,i)^2;
    end
end
% Calculate Time Cost
for i = 1:1:nS
    for j = 2:1:1+Tp
        Vec_Js(1,i) = Vec_Js(1,i) + Qs*Mat_ts_Pre(j,i)^2;
    end
end
% Calculate Sequential Trajectory Consistency Cost

% Calculate Inputs Cost
for i = 1:1:nS
    for j = 2:1:1+Tp
        Vec_Ju(1,i) = Vec_Ju(1,i) + Ru1*Mat_u1_Pre(j,i)^2 ...
                                  + Ru2*Mat_u2_Pre(j,i)^2;
    end
end
% Calculate Deltas Cost
for i = 1:1:nS
    for j = 2:1:1+Tp
        Vec_Jd(1,i) = Vec_Jd(1,i) + Rd1*Mat_d1_Pre(j,i)^2 ...
                                  + Rd2*Mat_d2_Pre(j,i)^2;
    end
end
% Calculate Total Cost
Vec_Jp = Vec_Je + Vec_Jh + Vec_Js + Vec_Ju + Vec_Jd;
% Find MinCost Trajectory
minIdex = find(Vec_Jp == min(Vec_Jp));
disp('Cost Calculation Time:'); disp(toc);
for i = 1:1:nS
    for j = 2:1:1+Tp
        Mat_Xv_Pre(j,i) = Local_X(j) - Mat_x4_Pre(j,i)*sin(Local_Tht(j));
        Mat_Xv_Pre(j,i) = Local_Y(j) + Mat_x4_Pre(j,i)*cos(Local_Tht(j));
    end
end
Traj_X = Local_X - Mat_x4_Pre(:,minIdex(1))'.*sin(Local_Tht);
Traj_Y = Local_Y + Mat_x4_Pre(:,minIdex(1))'.*cos(Local_Tht);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,4,1);hold on; title('Coordinates'); axis equal;
plot(RefLineInfo.X,RefLineInfo.Y,'r--');
plot(RefLineInfo.Xl,RefLineInfo.Yl,'k');
plot(RefLineInfo.Xr,RefLineInfo.Yr,'k');
plot(Traj_X,Traj_Y, 'g', 'LineWidth',2);
subplot(2,4,2); hold on; title('Lateral Error');
subplot(2,4,3); hold on; title('Heading Error');
subplot(2,4,4); hold on; title('Vx');
subplot(2,4,5); hold on; title('Vy');
subplot(2,4,6); hold on; title('Yaw Rate');
subplot(2,4,7); hold on; title('Torque');
subplot(2,4,8); hold on; title('Steer');
for i = 1:1:nS
    if i == minIdex
        subplot(2,4,2); plot(Mat_x4_Pre(:,i),'g-','LineWidth',2);
        subplot(2,4,3); plot(Mat_x5_Pre(:,i),'g-','LineWidth',2);
        subplot(2,4,4); plot(Mat_x1_Pre(:,i),'g-','LineWidth',2);
        subplot(2,4,5); plot(Mat_x2_Pre(:,i),'g-','LineWidth',2);
        subplot(2,4,6); plot(Mat_x3_Pre(:,i),'g-','LineWidth',2);
        subplot(2,4,7); plot(Mat_u1_Pre(:,i),'g-','LineWidth',2);
        subplot(2,4,8); plot(Mat_u2_Pre(:,i),'g-','LineWidth',2);
    else
        subplot(2,4,2); plot(Mat_x4_Pre(:,i),'k:');
        subplot(2,4,3); plot(Mat_x5_Pre(:,i),'k:');
        subplot(2,4,4); plot(Mat_x1_Pre(:,i),'k:');
        subplot(2,4,5); plot(Mat_x2_Pre(:,i),'k:');
        subplot(2,4,6); plot(Mat_x3_Pre(:,i),'k:');
        subplot(2,4,7); plot(Mat_u1_Pre(:,i),'k:');
        subplot(2,4,8); plot(Mat_u2_Pre(:,i),'k:');
    end
    pause(0.02);
end