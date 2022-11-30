function [Cost ,idx,latError] = cost_func(state, u, refPos_x,refPos_y, refyaw, param)
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
% Q1 = -5.127*exp(-36.37*abs(latError))+5.127*exp(0.7583*abs(latError));
% Q2 = -5.127*exp(-36.37*abs(latError)*dt)+5.127*exp(0.7583*abs(latError)*dt);
% Q3 = -5.127*exp(-36.37*abs(yaw_error))+5.127*exp(0.7583*abs(yaw_error));
% Q4 = -5.127*exp(-36.37*abs(yaw_error_rate*dt))+5.127*exp(0.7583*abs(yaw_error_rate*dt));
% Q = [Q1 0 0 0;
%      0 Q2 0 0;
%      0 0 Q3 0;
%      0 0 0 Q4];
Q = param.Q;

R = param.r;

Cost = X'*Q*X + u*R*u;
last_yaw_error = yaw_error;
last_latError = latError;
end