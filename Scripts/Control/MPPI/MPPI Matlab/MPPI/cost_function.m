function [Cost ,delta_r] = cost_function(state, u, refPos_x,refPos_y, refyaw,refPos_Delta, param)
persistent step last_latError last_yaw_error
dt = param.dt;
x = state(1);
y = state(2);
yaw = state(3);
idx = calc_target_index(x,y, refPos_x,refPos_y);


% 求位置、航向角参考量
x_r = refPos_x(idx);
y_r = refPos_y(idx);
heading_r = refyaw(idx);
delta_r = refPos_Delta(idx); 

% 求位置、航向角的误差
x_error  = x - x_r;
y_error = y - y_r;
yaw_error =  yaw - heading_r;

% 根据百度Apollo，计算横向误差
latError = y_error*cos(heading_r) - x_error*sin(heading_r);

% 将误差值赋值到状态量
% X(1,1) = x_error; 
% X(2,1) = y_error;  
% X(3,1) = yaw_error;

if isempty(step)
    latError_rate = 0;
    yaw_error_rate = 0;
    step = 1;
else
    latError_rate = (last_latError-latError)/dt;
    yaw_error_rate = (last_yaw_error-yaw_error)/dt;
end

X(1,1) = latError; 
X(2,1) = latError_rate;  
X(3,1) = yaw_error;
X(4,1) = yaw_error_rate;

Q = param.Q;

R = param.r;

Cost = X'*Q*X + u*R*u;
last_yaw_error = yaw_error;
last_latError = latError;
end