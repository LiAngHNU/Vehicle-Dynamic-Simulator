% SMPPI
% 
% Data: 2022年8月29日

clc;
clear;
close all;
%% 加载地图
load ../path.mat
load ../long_path3.mat
load ../s_gaosu.mat

%% 初始化车辆参数
param.v = 12;              %12m/s
param.m = 1400;
param.g = 9.81;
param.L = 4;
param.dt = 0.01;
%% 初始化控制器参数
param.lambda = 10;
param.variance = 0.3;
param.R = 1;
param.Q = [30 0 0 0;
           0 0 0 0;
           0 0 50 0;
           0 0 0 0];
param.r = 0.1;
param.omega = 1;
param.eta = 10;
%% 初始化仿真参数
K = 100;                % 每次迭代搜索的路径数量
N = 50;                 % 每条搜索路径的步数
iteration = 5000;

%% 路径处理
choose_path = 1;        % 选择仿真轨迹 1：爆堆  2：高速调头 3：S弯
switch choose_path
    case 1
        path = pathdata2(:,1:2);
        path = path(1:1500,:);
       %path = flip(path);
    case 2
        path = s_gaosu(4120:4300,[8 10]);
    case 3
        path = path(100:end,:);
end
% 定义参考轨迹
refPos_x = path(:,1);
refPos_y = path(:,2);
refPos = [refPos_x, refPos_y];
%障碍物
%obs = refPos(50,:);
% 计算航向角和曲率
diff_x = diff(refPos_x) ;
diff_x(end+1) = diff_x(end);
diff_y = diff(refPos_y) ;
diff_y(end+1) = diff_y(end);
derivative1 = gradient(refPos_y) ./ abs(diff_x);              % 一阶导数
derivative2 = del2(refPos_y) ./ abs(diff_x);                  % 二阶导数
refyaw = atan2(diff_y , diff_x);                              % 航向角

%% 绘制图像
% 画图
pathfig = figure(1);
set(pathfig, 'position', [240 90 1440 900]);
plot(refPos_x,refPos_y,'r')
axis equal;
hold on
%draw_circle(obs(1),obs(2),0.5,'b','k',.3);

h2 = figure(2);
set(h2, 'position', [30 560 560 420]);
axis([0 inf -1.2 1.2]);
title('控制量');xlabel('t/s');ylabel('u/deg');

h3 = figure(3);
hold on;
set(h3, 'position', [1360 490 560 420]);
axis([0 inf -1.2 1.2]);
title('横向误差');xlabel('t/s');ylabel('error/m');

h4 = figure(4);
set(h4, 'position', [30 50 560 420]);
axis([0 inf -1.2 1.2]);
title('控制量变化率');xlabel('t/s');ylabel('\Deltau/°/s');
%% 初始位置和初始化控制变量
x_init = [refPos_x(1) refPos_y(1) refyaw(1)];
U = zeros(1,N-1);
Action = zeros(1,N-1);
Vtk = zeros(K,N-1);
Atk = zeros(K,N-1);
Stk = zeros(1,K);
Xtk = zeros(3,N,K);
Epsilontk = zeros(K,N-1);
%% 记录控制循环中的变量
U_sys = zeros(1,iteration);
X_sys = zeros(3,iteration);
A_sys = zeros(1,iteration);
Laterror = zeros(1,iteration);
X_sys(:,1) = x_init;
x_cur = zeros(3,N-1);
a=[];
%% 设置变量最大值
delta_limit = deg2rad(45);                         % 最大车辆转角
delta_limit_rate = deg2rad(20000/40);                % 最大车辆转角速率,s
variance = 10;                                      % 标准差
%% SMPPI LOOP
for i = 1:iteration
    Vtk = Vtk.*0;
    Atk = Atk.*0;
    Stk = Stk.*0;
    Xtk = Xtk.*0;
    Epsilontk = Epsilontk.*0;
    for k = 1:K
        epsilon = normrnd(0,variance,[1, N-1]);
        Xtk(:,1,k) = x_init;
        for t = 1:N-1
            Epsilontk(k,t) = epsilon(t);
            temp_rate = U(t) + epsilon(t);
            % 限制车辆车轮转角速率
            if temp_rate > delta_limit_rate || temp_rate < -delta_limit_rate
                Epsilontk(k,t) = sign(temp_rate)*delta_limit_rate - U(t);
            end
            Vtk(k,t) = U(t) + Epsilontk(k,t);
            if t == 1
                Atk(k,t) = Action(1) + Vtk(k,t)*param.dt;
            else
                Atk(k,t) = Atk(k,t-1) + Vtk(k,t)*param.dt;
            end
            %Atk(k,t) = Action(t) + Vtk(k,t)*param.dt;
            % 限制车辆车轮最大转角
            Atk(k,t) = clamp_u(Atk(k,t), delta_limit, -delta_limit);
            % 更新车辆状态
            Xtk(:,t+1,k) = vehicle_kinetics(Xtk(:,t+1,k) ,Atk(k,t) ,param);
            [Cost ,~,~] = cost_func(Xtk(:,t+1,k), Atk(k,t), ...
                refPos_x, refPos_y, refyaw, param);
            Stk(k) = Stk(k) + Cost;
        end
        Stk(k) = Stk(k) + cost_action2(Atk(k,:), param);
    end
    % 计算控制量
    x_cur = x_cur.*0;
    x_cur(:,1) = X_sys(:, i);
    beta = min(Stk);
    w = exp(-(Stk-beta)/param.lambda)/param.eta;
    for j = 1:N-1
        %U(j) = U(j) + w * Epsilontk(:,j);
        U(j) = U(j) + totalEntropy(Stk(:) , Epsilontk(:,j),param);
        Action(j) = Action(j) + U(j)*param.dt;
        % 限制车辆车轮最大转角
        Action(j) = clamp_u(Action(j), delta_limit, -delta_limit);
        x_cur(:,j+1) =  vehicle_kinetics(x_cur(:,j) ,Action(j) ,param);
    end
    % 更新车辆状态
    X_sys(:,i+1) = vehicle_kinetics(X_sys(:, i), Action(1), param);
    % 记录控制量
    U_sys(i) = U(1);
    A_sys(i) = Action(1);
    U = [U(2:end) 0];
    Action = [Action(2:end) 0];
    x_init = X_sys(:,i+1);
    [~,idx,Laterror(i)] = cost_func(X_sys(:,i+1),U_sys(i), ...
        refPos_x,refPos_y,refyaw,param);
    % 绘制循环过程变量和运行轨迹
    figure(1);
    axis([X_sys(1, i)-8 X_sys(1, i)+8 X_sys(2, i)-8 X_sys(2, i)+8]);
    hold on;
    scatter(X_sys(1,i+1),X_sys(2,i+1),500,'g.');
    if isempty(a)
        hold on;
        a = draw_rollouts(Xtk,K,Stk);
        b = plot(x_cur(1,:),x_cur(2,:),'r','LineWidth',1.2);
    else
        delete(a);
        delete(b);
        hold on;
        a = draw_rollouts(Xtk,K,Stk);
        hold on;
        b = plot(x_cur(1,:),x_cur(2,:),'r','LineWidth',1.2);
    end

    figure(2);
    hold on;
    plot((1:i).*param.dt, rad2deg(A_sys(1:i)), 'r');
    axis([0 inf rad2deg(min(A_sys)-0.1) rad2deg(max(A_sys)+0.1)]);
    
    figure(3);
    hold on;
    plot((1:i).*param.dt, Laterror(1:i), 'b');
    axis([0 inf min(Laterror)-0.2 max(Laterror)+0.2]);

    figure(4);
    hold on;
    u_rate = diff([0 rad2deg(A_sys(1:i))])./param.dt;
    plot((1:i).*param.dt, u_rate, 'r');
    axis([0 inf min(u_rate)-20 max(u_rate)+20]);
end