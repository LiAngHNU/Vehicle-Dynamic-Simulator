% MPPI
% 
% 日期：20220815
clc
clear
close all
load path.mat
load long_path3.mat
load s_gaosu.mat

%net = importKerasNetwork('keras_model_recurrent.h5');
%% 相关参数定义
K = 300;            %路径条数300
N = 30;             %预测步长50
iteration = 5000;   %仿真步数5000
param.dt = 0.01;
param.v = 20;              %5m/s
param.m = 1400;
param.g = 9.81;
param.L = 4;


% Variance and Lamda
param.lambda = 10;
param.variance = 0.3;
param.R = 1;

param.Q = [30 0 0 0;
           0 0 0 0;
           0 0 50 0;
           0 0 0 0];

param.r = 0.1;
filter_param.ts = param.dt;
filter_param.epsilon = 0.78;
filter_param.cutfreq = 4;
%% 轨迹处理
choose_path = 3;               %3
switch choose_path
    case 1
        path = pathdata2(:,1:2);
        path = path(1:1500,:);
       %path = flip(path);
    case 2
        path = s_gaosu(4120:4300,[8 10]);
    case 3
        %path = path(100:end,:);
end

% 定义参考轨迹
refPos_x = path(:,1);
refPos_y = path(:,2);
% refPos_x = refPos_x(120:225,:);
% refPos_y = refPos_y(120:225,:);
refPos = [refPos_x, refPos_y];
%障碍物
obs = refPos(50,:);

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
title('控制量变化率');xlabel('t/s');ylabel('\Deltau/deg/s');
% 计算航向角和曲率
diff_x = diff(refPos_x) ;
diff_x(end+1) = diff_x(end);
diff_y = diff(refPos_y) ;
diff_y(end+1) = diff_y(end);
derivative1 = gradient(refPos_y) ./ abs(diff_x);              % 一阶导数
derivative2 = del2(refPos_y) ./ abs(diff_x);                  % 二阶导数
refyaw = atan2(diff_y , diff_x);                   % 航向角
% refK = abs(derivative2) ./ (1+derivative1.^2).^(3/2);  % 计算曲率
% 
% % 根据阿克曼转向原理，计算参考前轮转角
% refPos_Delta = atan(param.L*refK);



%% MPPI
% 赋初值
x = refPos_x(1); 
y = refPos_y(1); 
yaw = refyaw(1);
% x=18.8780;y=13.3453;
% yaw = 0.0357;
v = 10;
Delta = 0;
idx = 1;
step = 1;
% 轨迹跟踪实际量
pos_actual = [x,y];
v_actual  = v;
Delta_actual = Delta;
idx_actual = 1;
latError_LQR = [];
yaw_error_actual = [];
a =[];
%-------------------------------------------------------
% Initial State
x_init = [x y yaw];

% % Final state for Cart Pole
% x_fin = [0 0 pi 0];

% Variables To store the system state
X_sys = zeros(3,iteration+1);
U_sys = zeros(1,iteration);
cost  = zeros(1,iteration);
cost_avg = zeros(1,iteration);
laterr = zeros(1,iteration);

predict_path = cell(K,iteration);


% Initialization of Variables
x = zeros(3,N);
delta_u = zeros(N-1,K);
u_init = 0;

X_sys(:,1) = x_init;

% Initialization of input for N time horizone
u = zeros(1,N-1);
x_cur = zeros(3,N-1);
u_opt = zeros(1,N-1);
u_all = zeros(1,N-1,K);
d_u = zeros(1,N-1,K);
last_u = 0;
%-------------------------------------------------------
delta_limit = deg2rad(45);
d_delta_limit = 3.4907*param.dt*5;
variance = 1;                                                   %1
for i = 1:iteration
    i
    tic;
    % Initialization of Cost for K Samples
    Stk = zeros(1,K);
    delta_u = delta_u.*0;
    x = x.*0;
%     d_u = d_u.*0;
    % Calculating cost for K samples and N finite horizone
    for k = 1:K 
        x(:,1) = x_init;
        d_u = normrnd(0,variance,[1, N-1]);
%         u_all(:,:,k) = u + d_u;
        if i ~= 1
            last_u = U_sys(i-1);
        end
        for j = 1:N-1
            %delta_u(j,k) = normrnd(0,variance);
             delta_u(j,k) = d_u(j);
% % %             %delta_u(j,k) = param.variance*(randn(1)); %随机控制量
            %限制车辆转角极限值
            if delta_u(j,k)+u(j) > delta_limit
                delta_u(j,k) = delta_limit-u(j);
            elseif delta_u(j,k)+u(j) < -delta_limit
                delta_u(j,k) = -delta_limit-u(j);
            end
%             %限制车辆转角变化率
%             if abs(delta_u(j,k)+u(j)-last_u)/param.dt > d_delta_limit
%                 delta_u(j,k) = d_delta_limit*sign(delta_u(j,k)+u(j)-last_u);
%             end
%             last_u = delta_u(j,k)+u(j);
% %  %--------------------------------------------------------------------           
            
     %--------------------------------------------------------------------       
%             u_all(:,j,k) = clamp_u(u_all(:,j,k),delta_limit, -delta_limit);
            x(:,j+1) = vehicle_kinetics(x(:,j) ,u(j)+delta_u(j,k) ,param);
            if j == 1
                du = u(j)+delta_u(j,k) - last_u;
            else
                du = u(j)+delta_u(j,k) - u(j-1)-delta_u(j-1,k);
            end
            [Cost ,~,~] = cost_func(x(:,j+1), du, refPos_x,refPos_y,refyaw, param);
            Stk(k) = Stk(k) + Cost;
            %Stk(k) = Stk(k) + cost_obs(x(1:2,j+1)', obs, 0.5);
%             figure(1);
%             hold on;
%             plot(x(1,j:j+1),x(2,j:j+1),'b');
        end
        predict_path{k,i} = x;
%         delta_u(N,k) = normrnd(0,variance);
%         %delta_u(N,k) = param.variance*(randn(1));
%         if delta_u(N,k) > delta_limit
%             delta_u(N,k) = delta_limit;
%         elseif delta_u(N,k) < -delta_limit
%             delta_u(N,k) = -delta_limit;
%         end
    end
    % Average cost over iteration
%     cost_avg(i) = sum(Stk)/K;
    x_cur = x_cur.*0;
    x_cur(:,1) = X_sys(:, i);
    for n = 1:N-1
        u(n) = u(n) + totalEntropy(Stk(:) , delta_u(n,:),param);
        x_cur(:,n+1) =  vehicle_kinetics(x_cur(:,n) ,u(n) ,param);
    end
    %---------------------------------------------
%     rho = min(Stk);
%     eta = exp(-1/param.lambda*(Stk-rho));
%     w = eta/sum(Stk);
%     u_opt = 0.*u_opt;
%     for p = 1:K
%         u_opt = u_opt + w(p)*(u+delta_u(:,p)');
%     end
%     u = u_opt;
%     for g = 1:N-1
%         x_cur(:,g+1) =  vehicle_kinetics(x_cur(:,g) ,u(g) ,param);
%     end
    %---------------------------------------------
    % Input to the system 
    if u(1) > delta_limit
        U_sys(i) = delta_limit;
    elseif u(1) < -delta_limit
        U_sys(i) = -delta_limit;
    else
        U_sys(i) = u(1);
    end
%     U_sys(i) = u(1);
    U_sys(i) = double_lowpass_filter(U_sys(i), filter_param, i);
    disp(['当前控制量： ',num2str(U_sys(i))]);
    % System Updatation because of input
    X_sys(:,i+1) = vehicle_kinetics(X_sys(:, i), U_sys(i), param);
    
    
    predict_path2 = zeros(3,N,K);
    for g = 1:K
        predict_path2(:,:,g) = predict_path{g,i};
    end
    figure(1);
    %axis([X_sys(1, i)-8 X_sys(1, i)+8 X_sys(2, i)-8 X_sys(2, i)+8]);
    hold on;
    scatter(X_sys(1,i+1),X_sys(2,i+1),500,'g.');
    if isempty(a)
        hold on;
        a = draw_rollouts(predict_path2,K,Stk);
        b = plot(x_cur(1,:),x_cur(2,:),'r','LineWidth',1.2);
    else
        delete(a);
        delete(b);
        hold on;
        a = draw_rollouts(predict_path2,K,Stk);
        hold on;
        b = plot(x_cur(1,:),x_cur(2,:),'r','LineWidth',1.2);
    end

    figure(2);
    hold on;
%     scatter(curr_step, u(1), 'b');
    plot((1:i).*param.dt, rad2deg(U_sys(1:i)), 'r');
    axis([0 inf rad2deg(min(U_sys)-0.1) rad2deg(max(U_sys)+0.1)]);
    % Calculating state cost function
    [cost(i),idx,laterr(i)] = cost_func(X_sys(:,i+1),U_sys(i), refPos_x,refPos_y,refyaw,param);
    
    figure(3);
    hold on;
    plot((1:i).*param.dt, laterr(1:i), 'b');
    axis([0 inf min(laterr)-0.2 max(laterr)+0.2]);

    figure(4);
    hold on;
    u_rate = diff([0 rad2deg(U_sys(1:i))])./param.dt;
    plot((1:i).*param.dt, u_rate, 'r');
    axis([0 inf min(u_rate)-20 max(u_rate)+20]);
   % Input updatation for next time step
%     for b = 1:N-1
%         u(b) = u(b+1);
%     end

    u = [u(2:N-1) 0];
    % Updating the input in Last time step
    %u(N) = u_init;
    
    % initial state for calculating the expectency over trajectory in next
    % step
    x_init = X_sys(:,i+1);  
    disp(['第', num2str(i),'次迭代，耗时：' ,num2str(toc)]);
    if idx == length(path)-1
        break;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
    end
end
















