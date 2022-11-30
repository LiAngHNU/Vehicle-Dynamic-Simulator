%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Model Predictive Countouring Controller %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Version: v1.1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
% Date: 2022/10/19 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Controller Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set Steps
Ts = 0.05;
Tp =   40;
% Set Dimensions
nX =    5;
nU =    2;
nV =    1;
% Set Weights
Q1 =    1;
Q2 =    0;
Q3 =    1;
Q4 =    0;
Qp =   -1;
R1 =  100;
R2 =  100;
% Set Constraints
Vector_X_max = [];
Vector_X_min = [];
Vector_U_max = [];
Vector_U_min = [];
% Set Ref Line
load('Scenarios\Scenario_Course\Path_AutoX.mat');
% Set Vehicle
load('Models\Vehicles\VehicleSim\RacingVehicles\HUR_A18\HUR_A18_Calib.mat');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Update States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Update Vehicle Parameters
Ms = Vehicle_Calib.Mass;        Iz = Vehicle_Calib.Inertia;
Lf = Vehicle_Calib.Lf;          Lr = Vehicle_Calib.Lr;
Cf = Vehicle_Calib.Cyf;         Cr = Vehicle_Calib.Cyr;
Re = 0.500;
% Update States
x1 =   0.1000;      % x1: Vehicle Lateral Error 
x2 =   0.0000;      % x2: Vehicle Lateral Error Rate
x3 =   0.1000;      % x3: Vehicle Heading Error
x4 =   0.0000;      % x4: Vehicle Heading Error Rate
x5 =   0.0000;      % x5: Vehicle Driving Mileage
% Update Inputs
u1 =   5.0000;      % u1: Vehicle Longitudinal Velocity
u2 =   0.0000;      % u2: Vehicle Front Steering Angle
% Update Disturbances
v1 =   0.1000;      % v1: Curvature of Reference Line
% Update States
Vector_Xo = [x1; x2; x3; x4; x5;];
Vector_Uo = [u1; u2;];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Update System Matrices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize System Matrices
Matrix_Ad = zeros(nX,nX);
Matrix_Bd = zeros(nX,nU);
Matrix_Gd = zeros(nX,nV);
% Initialize Vectors
Vector_So = zeros(Tp,1);
Vector_Vo = zeros(Tp,1);
% Update System Matrices
Matrix_Ad(1,1) =  1;
Matrix_Ad(1,2) =  Ts;
Matrix_Ad(2,2) =  (2*Ts*(Cf + Cr))/(Ms*u1) + 1;
Matrix_Ad(2,3) = -(2*Ts*(Cf + Cr))/Ms;
Matrix_Ad(2,4) =  (2*Ts*(Cf*Lf - Cr*Lr))/(Ms*u1);
Matrix_Ad(3,3) =  1;
Matrix_Ad(3,4) =  Ts;
Matrix_Ad(4,1) = -(2*Ts*x4*(Cf*Lf^2 + Cr*Lr^2))/(Iz*x1^2);
Matrix_Ad(4,2) =  (2*Ts*(Cf*Lf - Cr*Lr))/(Iz*u1);
Matrix_Ad(4,3) = -(2*Ts*(Cf*Lf - Cr*Lr))/Iz;
Matrix_Ad(4,4) =  (2*Ts*(Cf*Lf^2 + Cr*Lr^2))/(Iz*x1) + 1;
Matrix_Ad(5,1) =  (Ts*v1*(u1*x3^2 - x2*x3 + u1))/(v1*x1 - 1)^2;
Matrix_Ad(5,2) =  (Ts*x3)/(v1*x1 - 1);
Matrix_Ad(5,3) =  (Ts*(x2 - 2*u1*x3))/(v1*x1 - 1);
Matrix_Ad(5,5) =  1;

Matrix_Bd(2,1) = -Ts*(2*u1*v1 + (2*x2*(Cf + Cr))/(Ms*u1^2) + (2*x4*(Cf*Lf - Cr*Lr))/(Ms*u1^2));
Matrix_Bd(2,2) = -(2*Cf*Ts)/Ms;
Matrix_Bd(4,1) = -(2*Ts*x2*(Cf*Lf - Cr*Lr))/(Iz*u1^2);
Matrix_Bd(4,2) = -(2*Cf*Lf*Ts)/Iz;
Matrix_Bd(5,1) = -(Ts*(x3^2 + 1))/(v1*x1 - 1);

Matrix_Gd(2,1) =  Ts*(- u1^2 + (2*Cf*Lf - 2*Cr*Lr)/Ms);
Matrix_Gd(4,1) =  (2*Ts*(Cf*Lf^2 + Cr*Lr^2))/Iz;
Matrix_Gd(5,1) =  (Ts*x1*(u1*x3^2 - x2*x3 + u1))/(v1*x1 - 1)^2;
% Update Vectors
for i = 1:1:Tp
Vector_So(i) = x5 + i*Ts*u1;
end
Vector_Vo = ...
    interp1(Path.S, Path.Kappa, Vector_So, 'linear', 'extrap');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Quadratic Programming Construction %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Quadratic Programming Construction: Cost Function %%%%%%%%%%%%%%%%%%%%%%
Matrix_h = zeros(nX,nX);
Matrix_f = zeros(nX, 1);
Matrix_H = zeros(Tp*(nX+nU),Tp*(nX+nU));
Matrix_F = zeros(Tp*(nX+nU),1);

for i = 1:1:Tp
Matrix_h = diag([Q1 Q2 Q3 Q4  0 R1 R2]);
Matrix_f =      [ 0  0  0  0 Qp  0  0];
Matrix_H(7*i-6:7*i,7*i-6:7*i) = Matrix_h;
Matrix_F(7*i-6:7*i,        1) = Matrix_f;
end
%% Quadratic Programming Construction: Equality Constraints %%%%%%%%%%%%%%%
Matrix_Aeq = zeros(Tp*nX,Tp*(nX+nU));
Matrix_Beq = zeros(Tp*nX,1);
Matrix_Ap  = zeros(Tp*nX,nX);
Matrix_Gp  = zeros(Tp*nX,Tp);
for i = 1:1:Tp
Matrix_Ap(5*i-4:5*i,:) = Matrix_Ad^i;
if i == 1
    Matrix_Gp(5*i-4:5*i,1) = Matrix_Gd;
else
    Matrix_Gp(5*i-4:5*i,:) = [Matrix_Ad^(i-1)*Matrix_Gd, Matrix_Gp(5*(i-1)-4:5*(i-1), 1:Tp-1)];
end
end

for i = 1:1:Tp
if i == 1
Matrix_Aeq(1:nX,1:nX+nU) = [eye(nX), -Matrix_Bd];
else
Matrix_Aeq(5*i-4:5*i,:) = [zeros(5,5), -Matrix_Ad^(i-1)*Matrix_Bd, Matrix_Aeq(5*(i-1)-4:5*(i-1),1:7*(Tp-1))];
end
end

Matrix_Beq = [Matrix_Ap, Matrix_Gp]*[Vector_Xo; Vector_Vo];
%% Quadratic Programming Construction: Inequality Constraints %%%%%%%%%%%%%
Matrix_Aneq_Ub = zeros(Tp*(nX+nU),Tp*(nX+nU));
Matrix_Aneq_Lb = zeros(Tp*(nX+nU),Tp*(nX+nU));
Matrix_Bneq_Ub = zeros(Tp*(nX+nU),1);
Matrix_Bneq_Lb = zeros(Tp*(nX+nU),1);

for i = 1:1:Tp
Matrix_Aneq_Ub(7*i-6:7*i,7*i-6:7*i) =  diag([1 0 0 0 0 1 1]);
Matrix_Aneq_Lb(7*i-6:7*i,7*i-6:7*i) = -diag([1 0 0 0 0 1 1]);
Matrix_Bneq_Ub(7*i-6:7*i,1) = [0.2; 0; 0; 0; 0; 20; 0.5;];
Matrix_Bneq_Lb(7*i-6:7*i,1) = [0.2; 0; 0; 0; 0; -1; 0.5;];
end

Matrix_Aneq = [Matrix_Aneq_Ub;...
               Matrix_Aneq_Lb;];
Matrix_Bneq = [Matrix_Bneq_Ub;...
               Matrix_Bneq_Lb;];
%% Solve QP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Qp_Ans = quadprog(Matrix_H, Matrix_F, [], [], Matrix_Aeq, Matrix_Beq);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Post Process %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Vector_x1_Predict = zeros(Tp + 1,1); Vector_x1_Predict(1) = x1;
Vector_x2_Predict = zeros(Tp + 1,1); Vector_x2_Predict(1) = x2;
Vector_x3_Predict = zeros(Tp + 1,1); Vector_x3_Predict(1) = x3;
Vector_x4_Predict = zeros(Tp + 1,1); Vector_x4_Predict(1) = x4;
Vector_x5_Predict = zeros(Tp + 1,1); Vector_x5_Predict(1) = x5;
Vector_u1_Predict = zeros(Tp + 1,1); Vector_u1_Predict(1) = u1;
Vector_u2_Predict = zeros(Tp + 1,1); Vector_u2_Predict(1) = u2;

for i = 1:1:Tp
Vector_x1_Predict(i+1,1) = Qp_Ans(7*(i-1) + 1);
Vector_x2_Predict(i+1,1) = Qp_Ans(7*(i-1) + 2);
Vector_x3_Predict(i+1,1) = Qp_Ans(7*(i-1) + 3);
Vector_x4_Predict(i+1,1) = Qp_Ans(7*(i-1) + 4);
Vector_x5_Predict(i+1,1) = Qp_Ans(7*(i-1) + 5);
Vector_u1_Predict(i+1,1) = Qp_Ans(7*(i-1) + 6);
Vector_u2_Predict(i+1,1) = Qp_Ans(7*(i-1) + 7);
end

figure;
subplot(2,4,1);
plot(Vector_x1_Predict);
subplot(2,4,2);
plot(Vector_x2_Predict);
subplot(2,4,3);
plot(Vector_x3_Predict);
subplot(2,4,4);
plot(Vector_x4_Predict);
subplot(2,4,5);
plot(Vector_x5_Predict);
subplot(2,4,6);
plot(Vector_u1_Predict);
subplot(2,4,7);
plot(Vector_u2_Predict);