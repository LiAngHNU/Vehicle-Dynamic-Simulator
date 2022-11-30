% SMPPI
% 
% 日期：20220815
clc
clear
close all
load ../path.mat
load ../long_path3.mat
load ../gaosu.mat


%% 相关参数定义
K = 200;
N = 50;
iteration = 5000;
param.dt = 0.01;
param.v = 0;              %Inital Speed
param.m = 1400;
param.g = 9.81;
param.L = 4;


% Variance and Lamda
param.lambda = 10;
param.variance = 0.3;
param.R = 1;

param.Q = [30 0 0 0;
           0 0 0 0;
           0 0 70 0;
           0 0 0 0];

param.r = 1;
param.omega = 2; %2

filter_param.ts = param.dt;
filter_param.epsilon = 0.78;
filter_param.cutfreq = 4;
%% 轨迹处理
choose_path = 1;
switch choose_path
    case 1
        path = pathdata2(:,1:2);
        path = path(1:1500,:);
    case 2
        path = gaosu(1200:1500,7:8);
    case 3
        %path = path(100:end,:);
end
% path = flip(path);
% 定义参考轨迹
refPos_x = path(:,1);
refPos_y = path(:,2);
% refPos_x = refPos_x(120:225,:);
% refPos_y = refPos_y(120:225,:);
refPos = [refPos_x, refPos_y];
% 画图
pathfig = figure(1);
set(pathfig, 'position', [240 90 1440 900]);
plot(refPos_x,refPos_y,'r')
axis equal;
hold on
plot_random_path = [];
h2 = figure(2);
set(h2, 'position', [30 390 560 420]);
axis([0 inf -1.2 1.2]);
title('控制量u');xlabel('t/s');ylabel('\omega/deg');

h3 = figure(3);
hold on;
set(h3, 'position', [1360 390 560 420]);
axis([0 inf -1.2 1.2]);
title('横向误差');xlabel('t/s');ylabel('m');

h4 = figure(4);
set(h4, 'position', [30 50 560 420]);
axis([0 inf -1.2 1.2]);
title('控制量变化率');xlabel('t/s');ylabel('\Deltau/deg/s');
% 计算航向角和曲率
diff_x = diff(refPos_x) ;
diff_x(end+1) = diff_x(end);
diff_y = diff(refPos_y) ;
diff_y(end+1) = diff_y(end);
derivative1 = gradient(refPos_y) ./ abs(diff_x);              % 一阶导数
derivative2 = del2(refPos_y) ./ abs(diff_x);                  % 二阶导数
refyaw = atan2(diff_y , diff_x);                   % 航向角
refK = abs(derivative2) ./ (1+derivative1.^2).^(3/2);  % 计算曲率

% 根据阿克曼转向原理，计算参考前轮转角
refPos_Delta = atan(param.L*refK);



%% MPPI
% 赋初值
x = refPos_x(1); 
y = refPos_y(1); 
yaw = refyaw(1);
idx = 1;
step = 1;
%-------------------------------------------------------
% Initial State
x_init = [x y yaw];

% % Final state for Cart Pole
% x_fin = [0 0 pi 0];

% Variables To store the system state
X_sys = zeros(3,iteration);
U_sys = zeros(1,iteration);
A_sys = zeros(1,iteration);
cost  = zeros(1,iteration);
cost_avg = zeros(1,iteration);
laterr = zeros(1,iteration);
predict_path = cell(K,iteration);
X_sys(:,1) = x_init;

% Initialization of Variables
x = zeros(3,N);
delta_u = zeros(K,N-1);
u_init = 0;

X_sys(:,1) = x_init;

% Initialization of input for N time horizone
u = zeros(1,N-1);
v = zeros(K,N-1);
a = zeros(K,N-1);
control_a = zeros(1,N-1);
x_cur = zeros(3,N-1);
u_opt = zeros(1,N-1);
u_all = zeros(1,N-1,K);
d_d_u = zeros(2,N-1,K);
last_u = 0;
%-------------------------------------------------------
delta_limit = deg2rad(30);
d_delta_limit = deg2rad(200);
variance = 1;
for i = 1:iteration
    tic;
    % Initialization of Cost for K Samples
    Stk = zeros(1,K);
    delta_u = delta_u.*0;
    x = x.*0;
%     d_u = d_u.*0;
    % Calculating cost for K samples and N finite horizone
    for k = 1:K
        x(:,1) = x_init;
        d_d_u = normrnd(0,variance,[2, N-1]);
%         u_all(:,:,k) = u + d_u;
        if i ~= 1
            last_u = A_sys(i-1);
        end
        for j = 1:N-1
            %d_d_u(j) = clamp_u(d_d_u(j),d_delta_limit,-d_delta_limit);
            delta_u(k,j) = d_d_u(j);
            v(k,j) = u(j)+delta_u(k,j);
            v(k,j) = clamp_u(v(k,j),d_delta_limit,-d_delta_limit);
            %a(k,j) = control_a(j) + v(k,j)*param.dt;
            if j == 1
                a(k,j) = control_a(1) + v(k,j)*param.dt;
            else
                a(k,j) = a(k,j-1) + v(k,j)*param.dt;
            end
            %限制车辆转角极限值
            a(k,j) = clamp_u(a(k,j), delta_limit, -delta_limit);
% %  %--------------------------------------------------------------------           
            
     %--------------------------------------------------------------------       
            %x(:,j+1) = vehicle_kinetics(x(:,j) ,a(k,j) ,param);
            x(:,j+1) = x(:,j) + myNeuralNetworkFunctionS([x(3,j);a(k,j);param.v])*param.dt;
            [Cost ,~,~] = cost_func(x(:,j+1), a(k,j), refPos_x,refPos_y,refyaw, param);
            Stk(k) = Stk(k) + Cost;
        end
        predict_path{k,i} = x;
        Stk(k) = Stk(k) + cost_action(a(k,:), last_u, param);
    end
    % Average cost over iteration
%     cost_avg(i) = sum(Stk)/K;
    x_cur = x_cur.*0;
    x_cur(:,1) = X_sys(:, i);
    for n = 1:N-1
        u(n) = u(n) + totalEntropy(Stk(:) , delta_u(:,n), param);
        control_a(n) = control_a(n)+u(n)*param.dt;
        %限制车辆转角极限值
        %control_a(n) = clamp_u(control_a(n), delta_limit, -delta_limit);
        x_cur(:,n+1) =  vehicle_kinetics(x_cur(:,n) ,control_a(n) ,param);
    end
    %---------------------------------------------
%     rho = min(Stk);
%     eta = exp(-1/param.lambda*(Stk-rho));
%     w = eta/sum(Stk);
%     u_opt = 0.*u_opt;
%     for p = 1:K
%         u_opt = u_opt + w(p)*(u+delta_u(p,:));
%     end
%     %u = u_opt;
%     for g = 1:N-1
%         u(g) = u(g) + u_opt(g);
%         control_a(g) = control_a(g) + u(g)*param.dt;
%         x_cur(:,g+1) =  vehicle_kinetics(x_cur(:,g) ,control_a(g) ,param);
%     end
    %---------------------------------------------
    % Input to the system 
    if control_a(1) > delta_limit
        A_sys(i) = delta_limit;
    elseif control_a(1) < -delta_limit
        A_sys(i) = -delta_limit;
    else
        A_sys(i) = control_a(1);
    end
    U_sys(i) = u(1);

    A_sys(i) = double_lowpass_filter(A_sys(i), filter_param, i);
    A_sys(i) = clamp_u(A_sys(i), delta_limit, -delta_limit);
    disp(['当前控制量： ',num2str(A_sys(i))]);
    % System Updatation because of input
    X_sys(:,i+1) = vehicle_kinetics(X_sys(:, i), A_sys(i), param);
    
    
    predict_path2 = zeros(3,N,K);
    for g = 1:K
        predict_path2(:,:,g) = predict_path{g,i};
    end
    figure(1);
    %axis([X_sys(1, i)-1 X_sys(1, i)+9 X_sys(2, i)-5 X_sys(2, i)+5]);
    hold on;
    scatter(X_sys(1,i+1),X_sys(2,i+1),500,'g.');
    if isempty(plot_random_path)
        hold on;
        plot_random_path = draw_rollouts(predict_path2,K,Stk);
        plot_predict_path = plot(x_cur(1,:),x_cur(2,:),'r','LineWidth',1.2);
    else
        delete(plot_random_path);
        delete(plot_predict_path);
        hold on;
        plot_random_path = draw_rollouts(predict_path2,K,Stk);
        hold on;
        plot_predict_path = plot(x_cur(1,:),x_cur(2,:),'r','LineWidth',1.2);
    end

%     figure(2);
%     hold on;
% %     scatter(curr_step, u(1), 'b');
%     plot((1:i).*param.dt, rad2deg(A_sys(1:i)), 'r');
%     axis([0 inf min(rad2deg(A_sys(1:i)))-0.1 max(rad2deg(A_sys(1:i)))+0.1]);
%     title('控制量u');
%     % Calculating state cost function
% 
%     
%     figure(3);
%     hold on;
%     plot((1:i).*param.dt, laterr(1:i), 'r');
%     axis([0 inf min(laterr)-0.2 max(laterr)+0.2]);
%     title('横向误差');xlabel('t/s');ylabel('m');
% 
%     figure(4);
%     hold on;
%     u_rate = diff([0 rad2deg(A_sys(1:i))])./param.dt;
%     plot((1:i).*param.dt, u_rate, 'r');
%     axis([0 inf min(u_rate)-20 max(u_rate)+20]);
    % Input updatation for next time step
%     for b = 1:N-1
%         u(b) = u(b+1);
%     end
    [cost(i),idx,laterr(i)] = cost_func(X_sys(:,i+1),A_sys(i), refPos_x,refPos_y,refyaw,param);
    u = [u(2:N-1) 0];
    a = [a(2:N-1) 0];
    % Updating the input in Last time step
    %u(N) = u_init;
    
    % initial state for calculating the expectency over trajectory in next
    % step
    x_init = X_sys(:,i+1);  
    time_comsume(i) = toc;
    disp(['第', num2str(i),'次迭代，耗时：' ,num2str(time_comsume(i))]);
    
    if idx == length(path)-1
        break;
    end
end

    figure(2);
    hold on;
%     scatter(curr_step, u(1), 'b');
    plot((1:i).*param.dt, rad2deg(A_sys(1:i)), 'r');
    axis([0 inf min(rad2deg(A_sys(1:i)))-0.1 max(rad2deg(A_sys(1:i)))+0.1]);
    title('控制量u');
    % Calculating state cost function

    
    figure(3);
    hold on;
    plot((1:i).*param.dt, laterr(1:i), 'r');
    axis([0 inf min(laterr)-0.2 max(laterr)+0.2]);
    title('横向误差');xlabel('t/s');ylabel('m');

    figure(4);
    hold on;
    u_rate = diff([0 rad2deg(A_sys(1:i))])./param.dt;
    plot((1:i).*param.dt, u_rate, 'r');
    axis([0 inf min(u_rate)-20 max(u_rate)+20]);














