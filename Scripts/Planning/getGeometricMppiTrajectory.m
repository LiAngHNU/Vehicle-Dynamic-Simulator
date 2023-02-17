%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Geometric Model Predictive Path Integral Planner %%%%%%%%%%%%%%%%%%%%%%%%
% Version: v1.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date: 2023/01/05 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Planner Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic;
% Set Steps
Ts = 2.00;
Tp = 270;
% Set Dimensions
nS = 1000;
% Set Weights
Ql = +0;     % Weight of Trajectory Length
Qc = +1;     % Weight of Trajectory Curvature
Qs = +0;     % Weight of Trajectory Similarity
Rd = +0;
% Set Constraints
u1_max = +4.50;
u1_min = -4.50;
eta = 0; lambda = 1e-12;
% Set Ref Line
load('Scenarios\6003_racetrack_line_03.mat');
% Set Start Position
StartID = 251;
disp('Loading Time:'); disp(toc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Vectors & Matrices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic;
% Initialize State Matrices
Mat_x1_Pre = zeros(1+Tp,nS);        % x1: X
Mat_x2_Pre = zeros(1+Tp,nS);        % x2: Y
Mat_x3_Pre = zeros(1+Tp,nS);        % x3: S
Mat_x4_Pre = zeros(1+Tp,nS);        % x4: Tht
Mat_x5_Pre = zeros(1+Tp,nS);        % x5: Kpa
Vec_x1_Opt = zeros(1+Tp, 1);
Vec_x2_Opt = zeros(1+Tp, 1);
% Initialize Input Matrices
Mat_u1_Pre = zeros(1+Tp,nS);        % u1: L
Vec_u1_Opt = zeros(1+Tp, 1);
% Initialize Delta Matrices
Mat_d1_Pre = zeros(1+Tp,nS);
% Initialize Cost Vector
Vec_Jc = zeros(1,nS);
Vec_Jd = zeros(1,nS);
Vec_Jl = zeros(1,nS);
Vec_Wt = zeros(1,nS);
disp('Initialization Time:'); disp(toc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Update States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic;
% Update States
x1 = RefLineInfo.X(StartID);
x2 = RefLineInfo.Y(StartID);
x3 = RefLineInfo.S(StartID);
x4 = RefLineInfo.Tht(StartID);
x5 = RefLineInfo.Kpa(StartID);
for i = 1:1:nS
    Mat_x1_Pre(1,i) = x1;
    Mat_x2_Pre(1,i) = x2;
    Mat_x3_Pre(1,i) = x3;
    Mat_x4_Pre(1,i) = x4;
    Mat_x5_Pre(1,i) = x5;
end
% Update Inputs
u1 = +0.00;
for i = 1:1:nS
    Mat_u1_Pre(1,i) = u1; 
end
% Update Inputs Exp & Var
u1_var = +0.10;
% Update Local Reference Line
Local_S = RefLineInfo.S(StartID):Ts:RefLineInfo.S(StartID)+Ts*Tp;
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
        if Mat_u1_Pre(j,i) > u1_max
            Mat_u1_Pre(j,i) = u1_max;
        end
        if Mat_u1_Pre(j,i) < u1_min
            Mat_u1_Pre(j,i) = u1_min;
        end
        Mat_d1_Pre(j,i) = Mat_u1_Pre(j,i) - Mat_u1_Pre(j-1,i);
    end
    Mat_u1_Pre(:,i) = smooth(Mat_u1_Pre(:,i));
end
disp('Input Sequences Generation Time:'); disp(toc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate State Sequences %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:nS
    for j = 2:1:1+Tp
        Mat_x1_Pre(j,i) = Local_X(j) - Mat_u1_Pre(j,i)*sin(Local_Tht(j));
        Mat_x2_Pre(j,i) = Local_Y(j) + Mat_u1_Pre(j,i)*cos(Local_Tht(j));
        Mat_x3_Pre(j,i) = Mat_x3_Pre(j-1,i) + sqrt((Mat_x1_Pre(j,i) - Mat_x1_Pre(j-1,i))^2 + (Mat_x2_Pre(j,i) - Mat_x2_Pre(j-1,i))^2);
        Mat_x4_Pre(j,i) = atan((Mat_x2_Pre(j,i) - Mat_x2_Pre(j-1,i))/(Mat_x1_Pre(j,i) - Mat_x1_Pre(j-1,i)));
        if Mat_x4_Pre(j,i) - Mat_x4_Pre(j-1,i) < -0.9*pi
            Mat_x4_Pre(j,i) = Mat_x4_Pre(j,i) + pi;
        end
        if Mat_x4_Pre(j,i) - Mat_x4_Pre(j-1,i) > +0.9*pi
            Mat_x4_Pre(j,i) = Mat_x4_Pre(j,i) - pi;
        end
    end
    tmp_x = spline(Mat_x3_Pre(:,i),Mat_x1_Pre(:,i));
    tmp_y = spline(Mat_x3_Pre(:,i),Mat_x2_Pre(:,i));
    dx = tmp_x.coefs(:,3);      dy = tmp_y.coefs(:,3);
    ddx = 2*tmp_x.coefs(:,2);   ddy = 2*tmp_y.coefs(:,2);
    for j = 2:1:1+Tp
        Mat_x5_Pre(j,i) = (dx(j-1,1)*ddy(j-1,1)-ddx(j-1,1)*dy(j-1,1))/((dx(j-1,1)^2+dy(j-1,1)^2)^(3/2));
    end
end
disp('State Prediction Time:'); disp(toc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate Cost %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:nS
    for j = 2:1:1+Tp
        Vec_Jc(1,i) = Vec_Jc(1,i) + Qc*Mat_x5_Pre(j,i)^2;
    end
end
for i = 1:1:nS
    for j = 2:1:1+Tp
        Vec_Jd(1,i) = Vec_Jd(1,i) + Rd*Mat_d1_Pre(j,i)^2;
    end
end
for i = 1:1:nS
    Vec_Jl(1,i) = Ql*(Mat_x3_Pre(1+Tp,i) - Mat_x3_Pre(1,i));
end
% Calculate Total Cost
Vec_Jp = Vec_Jc + Vec_Jd + Vec_Jl;
% Find MinCost Trajectory
minIdex = find(Vec_Jp == min(Vec_Jp));
minCost = Vec_Jp(minIdex);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Weighted Average %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:nS
    eta = eta + exp(-(Vec_Jp(i) - minCost)/lambda);
end
for i = 1:1:nS
    Vec_Wt(1,i) = (1/eta)*exp(-(Vec_Jp(i) - minCost)/lambda);
end
for i = 1:1:nS
    Vec_u1_Opt = Vec_u1_Opt + Vec_Wt(1,i)*Mat_u1_Pre(:,i);
end
Vec_x1_Opt(1,1) = RefLineInfo.X(StartID);
Vec_x2_Opt(1,1) = RefLineInfo.Y(StartID);
for i = 2:1:1+Tp
    Vec_x1_Opt(i,1) = Local_X(i) - Vec_u1_Opt(i)*sin(Local_Tht(i));
    Vec_x2_Opt(i,1) = Local_Y(i) + Vec_u1_Opt(i)*cos(Local_Tht(i));
end
disp('Cost Calculation Time:'); disp(toc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hold on; title('Coordinates'); axis equal;
for i = 1:1:nS
    if i ~= minIdex
        plot(Mat_x1_Pre(:,i),Mat_x2_Pre(:,i),'k:');
    end
    plot(Mat_x1_Pre(:,minIdex),Mat_x2_Pre(:,minIdex),'b-','LineWidth', 2);
end
plot(RefLineInfo.X,RefLineInfo.Y,'r--');
plot(RefLineInfo.Xl,RefLineInfo.Yl,'k');
plot(RefLineInfo.Xr,RefLineInfo.Yr,'k');
plot(Vec_x1_Opt,Vec_x2_Opt,'g-','LineWidth',2);