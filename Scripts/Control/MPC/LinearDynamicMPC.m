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
Xmax = zeros(5*Tp,1);               Xmin = zeros(5*Tp,1);
for i = 1:1:Tp
Xmax(5*Tp-4:5*Tp,1) = [+0.5,+0.5,+0.5,+0.5,+inf];
Xmin(5*Tp-4:5*Tp,1) = [-0.5,-0.5,-0.5,-0.5,-inf];
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
x1 = 0.25;
x2 = 1.00;
x3 = 0.05;
x4 = 0.20;
x5 = 0;
Vx = 15;
LastInput = 0;
% Update States
Matrix_Xo = [x1; x2; x3; x4; x5];
Matrix_Vo = zeros(Tp, 1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matrix Initialization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
a22 = -2*(Cf + Cr)/(Ms*Vx) - 2*(Cf*Lf*Lp - Cr*Lr*Lp)/(Iz*Vx);
a23 =  2*(Cf + Cr)/(Ms)    + 2*(Cf*Lf*Lp - Cr*Lr*Lp)/(Iz);              
a24 = -2*(Cf*(Lf - Lp)       - Cr*(Lr + Lp))/(Ms*Vx)...
      -2*(Cf*Lf*Lp*(Lf - Lp) + Cr*Lr*Lp*(Lr + Lp))/(Iz*Vx);
a25 =  2*Cf/Ms + 2*Cf*Lf*Lp/Iz;

a42 = -2*(Cf*Lf - Cr*Lr)/(Iz*Vx);
a43 =  2*(Cf*Lf - Cr*Lr)/(Iz);
a44 = -2*(Cf*Lf*(Lf - Lp) + Cr*Lr*(Lr + Lp))/(Iz*Vx);
a45 =  2*Cf*Lf/Iz;

a55 = -1/Ly;

b51 =  1/Ly;

g21 =  -Vx*Vx - 2*(Cf*(Lf - Lp) - Cr*(Lr + Lp))/(Ms*Vx)...
              - 2*(Cf*Lf*Lp*(Lf - Lp) + Cr*Lr*Lp*(Lr + Lp))/(Iz*Vx);  
g41 =  -2*(Cf*Lf*(Lf - Lp) + Cr*Lr*(Lr + Lp))/(Iz*Vx);

Matrix_Ac = [0,  1,  0,  0,  0;...
             0,a22,a23,a24,a25;...
             0,  0,  0,  1,  0;...
             0,a42,a43,a44,a45;...
             0,  0,  0,  0,a55;];
Matrix_Bc = [0;  0;  0;  0;b51;];
Matrix_Gc = [0;g21;  0;g41;  0;];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Discretization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Matrix_Ad = eye(5) + Ts*Matrix_Ac;
Matrix_Bd =          Ts*Matrix_Bc;
Matrix_Gd =          Ts*Matrix_Gc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Prediction Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Matrix_Ap = zeros(5*Tp, 5);
Matrix_Bp = zeros(5*Tp,Tp);
Matrix_Gp = zeros(5*Tp,Tp);

Matrix_Bp(1:5, 1) = Matrix_Bd;
Matrix_Gp(1:5, 1) = Matrix_Gd;

for i = 1:1:Tp
    Matrix_Ap(5*i-4:5*i, :) = Matrix_Ad^i;
end

for i = 2:1:Tp
    Matrix_Bp(5*i-4:5*i,:) = ...
        [Matrix_Ad^(i-1)*Matrix_Bd, Matrix_Bp(5*(i-1)-4:5*(i-1), 1:Tp-1)];
    Matrix_Gp(5*i-4:5*i,:) = ...
        [Matrix_Ad^(i-1)*Matrix_Gd, Matrix_Gp(5*(i-1)-4:5*(i-1), 1:Tp-1)];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% QP Construction %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Weights
Matrix_Q = zeros(5*Tp, 5*Tp);
Qp = diag([   1   10   10   50    0]);
Qt = diag([4000    0 4000    0    0]);
for i = 1:1:Tp-1
    Matrix_Q(5*i-4:5*i,5*i-4:5*i) = 1.0^i*Qp;       % Progress Weights
end
Matrix_Q(5*Tp-4:5*Tp,5*Tp-4:5*Tp) = Qt;             % Terminal Weights
Matrix_R = diag(1500*ones(1, Tp));
Matrix_P = diag([1000 1000 1000 1000 0]);
% Cost Function
Matrix_H = Matrix_Bp'*Matrix_Q*Matrix_Bp + Matrix_R;
Matrix_f = 2*Matrix_Bp'*Matrix_Q*(Matrix_Ap*Matrix_Xo + Matrix_Gp*Matrix_Vo);
% Constraints: Input Saturation
Matrix_A1 = [eye(Tp);...
            -eye(Tp);];
Matrix_b1 = [Umax*ones(Tp,1);... 
            -Umin*ones(Tp,1);];
% Constraints: Input Rate
Matrix_E = tril(ones(Tp,Tp));
Matrix_F = ones(Tp,1);
Matrix_A2 = [inv(Matrix_E);...
            -inv(Matrix_E)];
Matrix_b2 = [dUmax*ones(Tp,1) + inv(Matrix_E)*Matrix_F*LastInput/25;...
            -dUmin*ones(Tp,1) - inv(Matrix_E)*Matrix_F*LastInput/25];
% Constraints: Total
Matrix_A = [Matrix_A1;...
            Matrix_A2;];
Matrix_b = [Matrix_b1;...   
            Matrix_b2;];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Solve QP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ans = quadprog(Matrix_H, Matrix_f, Matrix_A, Matrix_b);
plot(ans);