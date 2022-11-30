%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Published by: Li.Ang
% Email: li.ang@cidi.ai
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Controller Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ts = 0.05;                          Tp = 20;
Qp = diag([   1    0   10    0]);
Qt = diag([   1    0   10    0]);
R  = diag([2000 4000]);
% Constraints
u1_max = +0.625;                    u1_min = -0.625;
u2_max = +0.000;                    u2_min = -0.000;
du1_max = +0.10;                    du2_min = -0.10;
x_max = [0.5;2;2;2];                x_min = [-0.5;-2;-2;-2];
Matrix_Xmax = zeros(4*Tp,1);
Matrix_Xmin = zeros(4*Tp,1);
for i = 1:1:Tp
Matrix_Xmax(4*i-3:4*i,1) = x_max;
Matrix_Xmin(4*i-3:4*i,1) = x_min;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Update States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle Parameters
Ms = 1000;               Iz = 2167;
Lf = 0.9;               Lr = 0.80;
Cf = -45000;            Cr = -45000;
Lp = 0;                  Ly = 0.20;
% Update States
Vx = 5.0;
x1 = 0.25;
x2 = 1.00;
x3 = 0.05;
x4 = 0.20;
x5 = 0.00;
x6 = 0.00;
u_last = 0.00;
% Update States
Matrix_Xo = [x1; x2; x3; x4];
Matrix_Vo = 0.15*ones(Tp, 1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matrix Initialization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Matrix_Ac = ...
[0,                           1,                      0,                               0;...
 0,       (2*Cf + 2*Cr)/(Ms*Vx),      -(2*Cf + 2*Cr)/Ms,     (2*Cf*Lf - 2*Cr*Lr)/(Ms*Vx);...
 0,                           0,                      0,                               1;...
 0, (2*Cf*Lf - 2*Cr*Lr)/(Iz*Vx), (2*Cf*Lf - 2*Cr*Lr)/Iz, (2*Cf*Lf^2 + 2*Cr*Lr^2)/(Iz*Vx);];

Matrix_Bc = ...
[            0,            0;...
    -(2*Cf)/Ms,   -(2*Cf)/Ms;...
             0,            0;...
 -(2*Cf*Lf)/Iz, (2*Cr*Lr)/Iz;];

Matrix_Gc = ...
[                                0;...
 - 2*Vx^2 + (2*Cf*Lf - 2*Cr*Lr)/Ms;...
                                 0;...
        (2*(Cf*Lf^2 + Cr*Lr^2))/Iz;];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Discretization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Matrix_Ad = eye(4) + Ts*Matrix_Ac;
Matrix_Bd =          Ts*Matrix_Bc;
Matrix_Gd =          Ts*Matrix_Gc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Prediction Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Matrix_Ap = zeros(4*Tp,   4);
Matrix_Bp = zeros(4*Tp,2*Tp);
Matrix_Gp = zeros(4*Tp,  Tp);

Matrix_Bp(1:4, 1:2) = Matrix_Bd;
Matrix_Gp(1:4, 1) = Matrix_Gd;

for i = 1:1:Tp
    Matrix_Ap(4*i-3:4*i, :) = Matrix_Ad^i;
end

for i = 2:1:Tp
    Matrix_Bp(4*i-3:4*i,:) = ...
        [Matrix_Ad^(i-1)*Matrix_Bd, Matrix_Bp(4*(i-1)-3:4*(i-1), 1:2*(Tp-1))];
    Matrix_Gp(4*i-3:4*i,:) = ...
        [Matrix_Ad^(i-1)*Matrix_Gd, Matrix_Gp(4*(i-1)-3:4*(i-1), 1:Tp-1)];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% QP Construction %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Weights
Matrix_Q = zeros(4*Tp, 4*Tp);
Matrix_R = zeros(2*Tp, 2*Tp);
Matrix_P = zeros(4*Tp, 4*Tp);
for i = 1:1:Tp-1
    Matrix_Q(4*i-3:4*i,4*i-3:4*i) = 1.2^i*Qp;       % Progress Weights
    Matrix_R(2*i-1:2*i,2*i-1:2*i) = 1.0^i*R;
end
Matrix_Q(4*Tp-3:4*Tp,4*Tp-3:4*Tp) = Qt;             % Terminal Weights
Matrix_R(2*Tp-1:2*Tp,2*Tp-1:2*Tp) = R;
% Cost Function
Matrix_H = Matrix_Bp'*Matrix_Q*Matrix_Bp + Matrix_R;
Matrix_f = 2*Matrix_Bp'*Matrix_Q*(Matrix_Ap*Matrix_Xo + Matrix_Gp*Matrix_Vo);
% Constraints: Input Saturation
Matrix_A1 = [eye(Tp), zeros(Tp,Tp);...
             zeros(Tp,Tp), eye(Tp);...
            -eye(Tp), zeros(Tp,Tp);...
             zeros(Tp,Tp),-eye(Tp)];
Matrix_b1 = [u1_max*ones(Tp,1);...
             u2_max*ones(Tp,1);...            
            -u1_min*ones(Tp,1);...
            -u2_min*ones(Tp,1);];
% Constraints: Input Rate

% Constraints: State
Matrix_A3 = [Matrix_Bp;...
            -Matrix_Bp;];
Matrix_b3 = [Matrix_Xmax - Matrix_Ap*Matrix_Xo - Matrix_Gp*Matrix_Vo;...
            -Matrix_Xmin + Matrix_Ap*Matrix_Xo + Matrix_Gp*Matrix_Vo;];

% Constraints: Total
Matrix_A = [Matrix_A1;...
            Matrix_A3;];
Matrix_b = [Matrix_b1;...   
            Matrix_b3;];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Solve QP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
options = optimoptions('quadprog', 'Display', 'iter');
ans = quadprog(Matrix_H, Matrix_f, Matrix_A, Matrix_b, [], [], [], [], [], options);
delta_f = 0;
delta_r = 0;
for i = 1:2:40
    delta_f = [delta_f, ans(i)];
end
for i = 2:2:40
    delta_r = [delta_r, ans(i)];
end
hold on;
plot(delta_f);
plot(delta_r);