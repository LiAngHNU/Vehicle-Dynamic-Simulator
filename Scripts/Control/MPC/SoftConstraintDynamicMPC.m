%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Published by: Li.Ang
% Email: li.ang@cidi.ai
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Controller Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ts = 0.05;                          Tp = 20;
Umax = 0.625;                       Umin = -0.625;
dUmax = 0.10;                       dUmin = -0.10;
Xmax = zeros(4*Tp,1);               Xmin = zeros(4*Tp,1);
for i = 1:1:Tp
Xmax(4*Tp-3:4*Tp,1) = [+0.5,+0.5,+0.5,+0.5];
Xmin(4*Tp-3:4*Tp,1) = [-0.5,-0.5,-0.5,-0.5];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Update States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle Parameters
Ms = 7870;               Iz = 35429;
Lf = 2.09;               Lr = 3.06;
Cf = -150000;            Cr = -300000;
Lp = -Lr;
Lx = 0.01;               Ly = 0.20;
% Update States
x1 = 0.0;
x2 = 1.00;
x3 = 0.05;
x4 = 0.20;
Vx = 15;
LastInput = 0;
% Update States
Matrix_Xo = [x1; x2; x3; x4];
Matrix_Vo = zeros(Tp, 1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matrix Initialization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Matrix_Ac = ...
[0,                           1,                      0,                               0;...
 0,       (2*Cf + 2*Cr)/(Ms*Vx),      -(2*Cf + 2*Cr)/Ms,     (2*Cf*Lf - 2*Cr*Lr)/(Ms*Vx);...
 0,                           0,                      0,                               1;...
 0, (2*Cf*Lf - 2*Cr*Lr)/(Iz*Vx), (2*Cf*Lf - 2*Cr*Lr)/Iz, (2*Cf*Lf^2 + 2*Cr*Lr^2)/(Iz*Vx);];

Matrix_Bc = ...
[            0;...
    -(2*Cf)/Ms;...
             0;...
 -(2*Cf*Lf)/Iz;];

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
Matrix_Bp = zeros(4*Tp,  Tp);
Matrix_Gp = zeros(4*Tp,  Tp);

Matrix_Bp(1:4, 1) = Matrix_Bd;
Matrix_Gp(1:4, 1) = Matrix_Gd;

for i = 1:1:Tp
    Matrix_Ap(4*i-3:4*i, :) = Matrix_Ad^i;
end

for i = 2:1:Tp
    Matrix_Bp(4*i-3:4*i,:) = ...
        [Matrix_Ad^(i-1)*Matrix_Bd, Matrix_Bp(4*(i-1)-3:4*(i-1), 1:Tp-1)];
    Matrix_Gp(4*i-3:4*i,:) = ...
        [Matrix_Ad^(i-1)*Matrix_Gd, Matrix_Gp(4*(i-1)-3:4*(i-1), 1:Tp-1)];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% QP Construction %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Weights
Matrix_Q = zeros(4*Tp, 4*Tp);
Matrix_P = zeros(4*Tp, 4*Tp);
Qp = diag([   1   10   10   50]);
Qt = diag([4000    0 4000    0]);
Rho= diag([100000  0    0    0]);
R  = diag([1000]);
for i = 1:1:Tp-1
    Matrix_Q(4*i-3:4*i,4*i-3:4*i) = 1.0^i*Qp;       % Progress Weights
    Matrix_P(4*i-3:4*i,4*i-3:4*i) = 1.0^i*Rho;      % Relaxation Weights
    Matrix_R(i,i) = R;
end
Matrix_Q(4*Tp-3:4*Tp,4*Tp-3:4*Tp) = Qt;             % Terminal Weights
Matrix_R(Tp,Tp) = R;
Matrix_P(4*Tp-3:4*Tp,4*Tp-3:4*Tp) = Rho;
% Cost Function
Matrix_H = blkdiag(Matrix_Bp'*Matrix_Q*Matrix_Bp + Matrix_R, 2*Matrix_P);
Matrix_f = [2*Matrix_Bp'*Matrix_Q*(Matrix_Ap*Matrix_Xo + Matrix_Gp*Matrix_Vo);...
            zeros(4*Tp, 1)];
% Constraints: Input Saturation
Matrix_A1 = [blkdiag( eye(Tp), zeros(4*Tp, 4*Tp));...
             blkdiag(-eye(Tp), zeros(4*Tp, 4*Tp));];
Matrix_b1 = [Umax*ones(Tp,1);...
             zeros(4*Tp,1);...
            -Umin*ones(Tp,1);...
             zeros(4*Tp,1)];
% Constraints: Input Rate
Matrix_E = tril(ones(Tp,Tp));
Matrix_F = ones(Tp,1);
Matrix_A2 = [blkdiag( inv(Matrix_E), zeros(4*Tp, 4*Tp));...
             blkdiag(-inv(Matrix_E), zeros(4*Tp, 4*Tp));];
Matrix_b2 = [dUmax*ones(Tp,1) + inv(Matrix_E)*Matrix_F*LastInput/25;...
             zeros(4*Tp,1);...
            -dUmin*ones(Tp,1) - inv(Matrix_E)*Matrix_F*LastInput/25;...
             zeros(4*Tp,1);];
% Constraints: State
Matrix_A3 = [blkdiag( Matrix_Bp,  eye(4*Tp));...
             blkdiag(-Matrix_Bp,  eye(4*Tp));];
Matrix_b3 = [Xmax - Matrix_Ap*Matrix_Xo - Matrix_Gp*Matrix_Vo;...
                                                zeros(4*Tp,1);...
             -Xmin + Matrix_Ap*Matrix_Xo + Matrix_Gp*Matrix_Vo;...
                                                zeros(4*Tp,1);];
% Constraints: Total
Matrix_A = [Matrix_A1;...
            Matrix_A2;...
            Matrix_A3;];
Matrix_b = [Matrix_b1;...   
            Matrix_b2;...
            Matrix_b3];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Solve QP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ans = quadprog(Matrix_H, Matrix_f);
plot(ans(1:20));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
states = Matrix_Ap*Matrix_Xo + Matrix_Bp*ans(1:20) + Matrix_Gp*Matrix_Vo;
ey = 0;
ep = 0;
for i = 1:4:80
    ey = [ey, states(i)];
end
for i = 3:4:80
    ep = [ep, states(i)];
end
figure;
hold on;
plot(ey);
plot(ep);