function state_dot = BicycleModelMPPI(state, control)
% myFun - Description
% 
% Syntax: output = myFun(input)
%
% Long description

% X_dot = v0*cos(phi)
% y_dot = v0*sin(phi)
% phi_dot = wd
% v_dot   = Ka*v0*(vd-v0)

% Set parameters
x   = state(1);
y   = state(2);
phi = state(3);
v0  = state(4);
Ka  = 2.54;
vd  = control(1);
wd  = control(2);

% Dynamic model
x_dot   = v0*cos(phi);
y_dot   = v0*sin(phi);
phi_dot = wd;
v_dot   = Ka*v0*(vd-v0);

% Build state dot
state_dot = [x_dot;y_dot;phi_dot;v_dot];
end