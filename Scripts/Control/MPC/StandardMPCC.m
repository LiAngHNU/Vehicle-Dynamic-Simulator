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
Ts =  0.02;
Tp =   100;
% Set Dimensions
nX =    10;
nU =     3;
nV =     0;
% Set Weights
Qc =   0.2;     Rt =     0;     Rdt=  1e0;
Ql =   0.2;     Rd =     0;     Rdd=  1e0;
Qp =  1000;     Rv =     0;     Rdv=  1e0;
% Set Constraints
Vector_X_max = [+0.00; +0.00; +3.14; +0.00; +0.00; +0.00; +0.00; + 200; +0.60; +20.0];
Vector_X_min = [+0.00; +0.00; -6.28; +0.00; -0.00; -0.00; +0.00; - 500; -0.60; +1.00];
Vector_U_max = [+50.0; +0.20; +0.00];
Vector_U_min = [-50.0; -0.20; -0.00];
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
Re = 0.260;
% Update States
Index = 1;
x1 =   Path.X(Index);       % x1: Vehicle Global X Coordinate 
x2 =   Path.Y(Index);       % x2: Vehicle Global Y Coordinate
x3 =   Path.Theta(Index);   % x3: Vehicle Yaw Angle
x4 =   1.0000;              % x4: Vehicle Lon Velocity
x5 =   0.0000;              % x5: Vehicle Lat Velocity
x6 =   0.0000;              % x6: Vehicle Yaw Rate
x7 =   Path.S(Index);       % x7: Vehicle Driving Mileage
x8 =   0.0000;              % x8: Vehicle Torque
x9 =   0.0000;              % x9; Vehicle Steer Angle
x10=   1.0000;              % x10:Vehicle S Velocity
% Update Inputs
u1 =   0.0000;              % u1: Vehicle Torque Augment
u2 =   0.0000;              % u2: Vehicle Steer Angle Augment
u3 =   0.0000;              % u3: Vehicle S Velocity Augment
% Update States
Vector_Xo = [x1; x2; x3; x4; x5; x6; x7; x8; x9; x10];
Vector_Uo = [u1; u2; u3;];
% Update Station
Vector_So = zeros(Tp,1);
for i = 1:1:Tp
    Vector_So(i,1) = x7 + i*Ts*x4;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Matrix Initialization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize System Matrices
Matrix_Ad = zeros(nX,nX);
Matrix_Bd = zeros(nX,nU);
% Initialize Ref Line Info
Vector_X_Ref =      zeros(Tp,1);
Vector_Xl_Ref =     zeros(Tp,1);
Vector_Xr_Ref =     zeros(Tp,1);
Vector_Y_Ref =      zeros(Tp,1);
Vector_Yl_Ref =     zeros(Tp,1);
Vector_Yr_Ref =     zeros(Tp,1);
Vector_Theta_Ref =  zeros(Tp,1);
Vector_dX_Ref =     zeros(Tp,1);
Vector_dY_Ref =     zeros(Tp,1);
Vector_dTheta_Ref = zeros(Tp,1);
% Initialize Errors
Vector_Ec = zeros(Tp,1);
Vector_El = zeros(Tp,1);
% Initialize Error Gradients
Matrix_Grad_Ec = zeros(nX,Tp);
Matrix_Grad_El = zeros(nX,Tp);
% Initialize Cost Function
Matrix_h = zeros(nX,nX);    Matrix_H = zeros(Tp*(nX+nU),Tp*(nX+nU));
Matrix_f = zeros(nX, 1);    Matrix_F = zeros(Tp*(nX+nU),         1);
% Initialize Equality Constraint
Matrix_Aeq_Sys = zeros(Tp*nX,Tp*(nX+nU));
Matrix_Beq_Sys = zeros(Tp*nX,         1);
% Initialize Inequality Constraint
Matrix_Aneq_Total = zeros(2*(nX+nU),nX+nU);
Matrix_bneq_Total = zeros(2*(nX+nU),    1);
Matrix_Aneq_TrackUb = zeros(Tp,Tp*(nX+nU));
Matrix_Bneq_TrackUb = zeros(Tp,         1);
Matrix_Aneq_TrackLb = zeros(Tp,Tp*(nX+nU));
Matrix_Bneq_TrackLb = zeros(Tp,         1);
Matrix_Aneq_Input = zeros(Tp,Tp*(nX+nU));
Matrix_Bneq_Input = zeros(Tp,         1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Matrix Update %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Update System Matrices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Matrix_Ad(1,1) =  1;
Matrix_Ad(1,3) = -Ts*(x5*cos(x3) + x4*sin(x3));
Matrix_Ad(1,4) =  Ts*cos(x3);
Matrix_Ad(1,5) = -Ts*sin(x3);

Matrix_Ad(2,2) =  1;
Matrix_Ad(2,3) =  Ts*(x4*cos(x3) - x5*sin(x3));
Matrix_Ad(2,4) =  Ts*sin(x3);
Matrix_Ad(2,5) =  Ts*cos(x3);

Matrix_Ad(3,3) =  1;
Matrix_Ad(3,6) =  Ts;

Matrix_Ad(4,4) =  (Cf*Ts*sin(u2)*(x5 + Lf*x6))/(Ms*x4^2) + 1;
Matrix_Ad(4,5) =  (Ts*(Ms*x6 - (Cf*sin(u2))/x4))/Ms;
Matrix_Ad(4,6) =  (Ts*(Ms*x5 - (Cf*Lf*sin(u2))/x4))/Ms;
Matrix_Ad(4,8) =  Ts/(Ms*Re);
Matrix_Ad(4,9) =  (Ts*(Cf*sin(u2) + Cf*cos(u2)*(u2 - (x5 + Lf*x6)/x4)))/Ms;

Matrix_Ad(5,4) = -(Ts*((x5*(Cf + Cr))/x4^2 + (x6*(Cf*Lf - Cr*Lr))/x4^2))/Ms;
Matrix_Ad(5,5) =  (Ts*(Cf + Cr))/(Ms*x4) + 1;
Matrix_Ad(5,6) =  (Ts*(Cf*Lf - Cr*Lr))/(Ms*x4);
Matrix_Ad(5,9) = -(Cf*Ts)/Ms;

Matrix_Ad(6,4) =  (Ts*((Cr*Lr*(x5 - Lr*x6))/x4^2 - (Cf*Lf*cos(u2)*(x5 + Lf*x6))/x4^2))/Iz;
Matrix_Ad(6,5) = -(Ts*((Cr*Lr)/x4 - (Cf*Lf*cos(u2))/x4))/Iz;
Matrix_Ad(6,6) =  (Ts*((Cf*cos(u2)*Lf^2)/x4 + (Cr*Lr^2)/x4))/Iz + 1;
Matrix_Ad(6,9) = -(Ts*(Cf*Lf*cos(u2) - Cf*Lf*sin(u2)*(u2 - (x5 + Lf*x6)/x4)))/Iz;

Matrix_Ad(7,7) =  1;
Matrix_Ad(7,10)=  Ts;
Matrix_Ad(8,8) =  1;
Matrix_Ad(9,9) =  1;
Matrix_Ad(10,10)= 1;

Matrix_Bd(8,1) =  Ts;
Matrix_Bd(9,2) =  Ts;
Matrix_Bd(10,3) = Ts;
%% Update Ref Line Info %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:Tp
Vector_X_Ref(i,1)    = ...
    interp1(Path.S, Path.X,     Vector_So(i), 'linear', 'extrap');
Vector_Xl_Ref(i,1)   = ...
    interp1(Path.S, Path.Xl,    Vector_So(i), 'linear', 'extrap');
Vector_Xr_Ref(i,1)   = ...
    interp1(Path.S, Path.Xr,    Vector_So(i), 'linear', 'extrap');
Vector_Y_Ref(i,1)    = ...
    interp1(Path.S, Path.Y,     Vector_So(i), 'linear', 'extrap');
Vector_Yl_Ref(i,1)    = ...
    interp1(Path.S, Path.Yl,    Vector_So(i), 'linear', 'extrap');
Vector_Yr_Ref(i,1)    = ...
    interp1(Path.S, Path.Yr,    Vector_So(i), 'linear', 'extrap');
Vector_Theta_Ref(i,1)  = ...
    interp1(Path.S, Path.Theta, Vector_So(i), 'linear', 'extrap');
Vector_dX_Ref(i,1)   = ...
    interp1(Path.S, Path.dX,    Vector_So(i), 'linear', 'extrap');
Vector_dY_Ref(i,1)   = ...
    interp1(Path.S, Path.dY,    Vector_So(i), 'linear', 'extrap');
Vector_dTheta_Ref(i,1) = ...
    interp1(Path.S, Path.dTheta,Vector_So(i), 'linear', 'extrap');
end
%% Update Errors %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:Tp
Vector_Ec(i,1) = ...
    + (Vector_Xo(1) - Vector_X_Ref(i,1))*sin(Vector_Theta_Ref(i,1))...
    - (Vector_Xo(2) - Vector_Y_Ref(i,1))*cos(Vector_Theta_Ref(i,1));
Vector_El(i,1) = ...
    - (Vector_Xo(1) - Vector_X_Ref(i,1))*cos(Vector_Theta_Ref(i,1))...
    - (Vector_Xo(2) - Vector_Y_Ref(i,1))*sin(Vector_Theta_Ref(i,1));
end
%% Update Error Gradients %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Vector_PD_EcX = +sin(Vector_Theta_Ref);
Vector_PD_EcY = +cos(Vector_Theta_Ref);
Vector_PD_EcS = ...
    - Vector_dX_Ref.*sin(Vector_Theta_Ref)...
    + Vector_dTheta_Ref.*cos(Vector_Theta_Ref).*(x1*ones(Tp,1) - Vector_X_Ref)...
    - Vector_dY_Ref.*cos(Vector_Theta_Ref)...
    + Vector_dTheta_Ref.*sin(Vector_Theta_Ref).*(x2*ones(Tp,1) - Vector_Y_Ref);

Vector_PD_ElX = -cos(Vector_Theta_Ref);
Vector_PD_ElY = -sin(Vector_Theta_Ref);
Vector_PD_ElS = ...
    + Vector_dX_Ref.*cos(Vector_Theta_Ref)...
    + Vector_dTheta_Ref.*sin(Vector_Theta_Ref).*(x1*ones(Tp,1) - Vector_X_Ref)...
    + Vector_dY_Ref.*sin(Vector_Theta_Ref)...
    - Vector_dTheta_Ref.*cos(Vector_Theta_Ref).*(x2*ones(Tp,1) - Vector_Y_Ref);

for i = 1:1:Tp
Matrix_Grad_Ec(1,i) = Vector_PD_EcX(i);
Matrix_Grad_Ec(2,i) = Vector_PD_EcY(i);
Matrix_Grad_Ec(7,i) = Vector_PD_EcS(i);
Matrix_Grad_El(1,i) = Vector_PD_ElX(i);
Matrix_Grad_El(2,i) = Vector_PD_ElY(i);
Matrix_Grad_El(7,i) = Vector_PD_ElS(i);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Quadratic Programming Construction %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Quadratic Programming Construction: Cost Function %%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:Tp
Matrix_h = [Matrix_Grad_Ec(:,i), Matrix_Grad_El(:,i)]*diag([Qc Ql])*[Matrix_Grad_Ec(:,i)'; Matrix_Grad_El(:,i)'];
Matrix_f = 2*Matrix_Grad_Ec(:,i)*Qc*(Vector_Ec(i,1) - Matrix_Grad_Ec(:,i)'*Vector_Xo) + ...
           2*Matrix_Grad_El(:,i)*Ql*(Vector_El(i,1) - Matrix_Grad_El(:,i)'*Vector_Xo);
Matrix_f(10,1) = - Qp;
Matrix_H((nX+nU)*i-(nX+nU-1):(nX+nU)*i, (nX+nU)*i-(nX+nU-1):(nX+nU)*i) = blkdiag(Matrix_h, diag([Rdt Rdd Rdv]));
Matrix_F((nX+nU)*i-(nX+nU-1):(nX+nU)*i,                                   1) = [Matrix_f; 0; 0; 0;];
end
%% Quadratic Programming Construction: Equality Constraints %%%%%%%%%%%%%%%
Matrix_Aeq_Sys(1:nX,1:nX+nU) = [eye(nX), -Matrix_Bd];

for i = 2:1:Tp
Matrix_Aeq_Sys(nX*(i-1)+1:nX*i,:) = [zeros(nX,nX), -Matrix_Ad^(i-1)*Matrix_Bd, Matrix_Aeq_Sys(nX*(i-2)+1:nX*(i-1),1:(nX+nU)*(Tp-1))];
end

for i = 1:1:Tp
Matrix_Beq_Sys(nX*(i-1)+1:nX*i,1) = (Matrix_Ad^i)*Vector_Xo;
end
%% Quadratic Programming Construction: Inequality Constraints %%%%%%%%%%%%%
% Track Constraint
for i = 1:1:Tp
    if cos(Vector_Theta_Ref(i)) < 0
    Matrix_Aneq_TrackUb(i,(nX+nU)*(i-1)+1:(nX+nU)*i) = [-tan(Vector_Theta_Ref(i)),1,0,0,0,0,0,0,0,0,0,0,0];
    Matrix_Bneq_TrackUb(i,           1) = -tan(Vector_Theta_Ref(i))*Vector_Xr_Ref(i)+Vector_Yr_Ref(i);
    Matrix_Aneq_TrackLb(i,(nX+nU)*(i-1)+1:(nX+nU)*i) = [tan(Vector_Theta_Ref(i)),-1,0,0,0,0,0,0,0,0,0,0,0];
    Matrix_Bneq_TrackLb(i,           1) = tan(Vector_Theta_Ref(i))*Vector_Xl_Ref(i)-Vector_Yl_Ref(i);
    else
    Matrix_Aneq_TrackUb(i,(nX+nU)*(i-1)+1:(nX+nU)*i) = [-tan(Vector_Theta_Ref(i)),1,0,0,0,0,0,0,0,0,0,0,0];
    Matrix_Bneq_TrackUb(i,          1)  = -tan(Vector_Theta_Ref(i))*Vector_Xl_Ref(i)+Vector_Yl_Ref(i);
    Matrix_Aneq_TrackLb(i,(nX+nU)*(i-1)+1:(nX+nU)*i) = [tan(Vector_Theta_Ref(i)),-1,0,0,0,0,0,0,0,0,0,0,0];
    Matrix_Bneq_TrackLb(i,          1)  = tan(Vector_Theta_Ref(i))*Vector_Xr_Ref(i)-Vector_Yr_Ref(i);
    end 
end
Matrix_Aneq_Track = [Matrix_Aneq_TrackUb;...
                     Matrix_Aneq_TrackLb];
Matrix_Bneq_Track = [Matrix_Bneq_TrackUb;...
                     Matrix_Bneq_TrackLb];
% State Constraint & Input Constraint & Rate Constraint
for i = 1:1:Tp
Matrix_Aneq_Input_UB((nX+nU)*i-(nX+nU-1):(nX+nU)*i,(nX+nU)*i-(nX+nU-1):(nX+nU)*i) = ...
                                                                           diag([0 0 +1 0 0 0 0 +1 +1 +1 +1 +1 0]);
Matrix_Bneq_Input_UB((nX+nU)*i-(nX+nU-1):(nX+nU)*i,                            1) = ...
                                                                           [+Vector_X_max;+Vector_U_max];
Matrix_Aneq_Input_LB((nX+nU)*i-(nX+nU-1):(nX+nU)*i,(nX+nU)*i-(nX+nU-1):(nX+nU)*i) = ...
                                                                           diag([0 0 -1 0 0 0 0 -1 -1 -1 -1 -1 0]);
Matrix_Bneq_Input_LB((nX+nU)*i-(nX+nU-1):(nX+nU)*i,                            1) = ...
                                                                           [-Vector_X_min;-Vector_U_min];
end
Matrix_Aneq_Input = [Matrix_Aneq_Input_UB;...
                     Matrix_Aneq_Input_LB;];
Matrix_Bneq_Input = [Matrix_Bneq_Input_UB;...
                     Matrix_Bneq_Input_LB;];
% Total Constraint
Matrix_Aneq_Total = [Matrix_Aneq_Track;...
                     Matrix_Aneq_Input;];
Matrix_Bneq_Total = [Matrix_Bneq_Track;...
                     Matrix_Bneq_Input;];
%% Solve QP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Qp_Ans = quadprog(Matrix_H, Matrix_F, Matrix_Aneq_Total, Matrix_Bneq_Total, Matrix_Aeq_Sys, Matrix_Beq_Sys);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Post Process %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Vector_x1_Predict = zeros(Tp + 1,1); Vector_x1_Predict(1) = x1;
Vector_x2_Predict = zeros(Tp + 1,1); Vector_x2_Predict(1) = x2;
Vector_x3_Predict = zeros(Tp + 1,1); Vector_x3_Predict(1) = x3;
Vector_x4_Predict = zeros(Tp + 1,1); Vector_x4_Predict(1) = x4;
Vector_x5_Predict = zeros(Tp + 1,1); Vector_x5_Predict(1) = x5;
Vector_x6_Predict = zeros(Tp + 1,1); Vector_x6_Predict(1) = x6;
Vector_x7_Predict = zeros(Tp + 1,1); Vector_x7_Predict(1) = x7;
Vector_u1_Predict = zeros(Tp + 1,1); Vector_u1_Predict(1) = x8;
Vector_u2_Predict = zeros(Tp + 1,1); Vector_u2_Predict(1) = x9;
Vector_u3_Predict = zeros(Tp + 1,1); Vector_u3_Predict(1) = x10;
Vector_du1_Predict = zeros(Tp + 1,1); Vector_du1_Predict(1) = u1;
Vector_du2_Predict = zeros(Tp + 1,1); Vector_du2_Predict(1) = u2;
Vector_du3_Predict = zeros(Tp + 1,1); Vector_du3_Predict(1) = u3;

for i = 1:1:Tp
Vector_x1_Predict(i+1,1) = Qp_Ans((nX+nU)*(i-1) + 1);
Vector_x2_Predict(i+1,1) = Qp_Ans((nX+nU)*(i-1) + 2);
Vector_x3_Predict(i+1,1) = Qp_Ans((nX+nU)*(i-1) + 3);
Vector_x4_Predict(i+1,1) = Qp_Ans((nX+nU)*(i-1) + 4);
Vector_x5_Predict(i+1,1) = Qp_Ans((nX+nU)*(i-1) + 5);
Vector_x6_Predict(i+1,1) = Qp_Ans((nX+nU)*(i-1) + 6);
Vector_x7_Predict(i+1,1) = Qp_Ans((nX+nU)*(i-1) + 7);
Vector_u1_Predict(i+1,1) = Qp_Ans((nX+nU)*(i-1) + 8);
Vector_u2_Predict(i+1,1) = Qp_Ans((nX+nU)*(i-1) + 9);
Vector_u3_Predict(i+1,1) = Qp_Ans((nX+nU)*(i-1) + 10);
Vector_du1_Predict(i+1,1) = Qp_Ans((nX+nU)*(i-1) + 11);
Vector_du2_Predict(i+1,1) = Qp_Ans((nX+nU)*(i-1) + 12);
Vector_du3_Predict(i+1,1) = Qp_Ans((nX+nU)*(i-1) + 13);
end

figure; hold on;
plot(Path.X, Path.Y, 'b--');
plot(Path.Xl,Path.Yl,'r');
plot(Path.Xr,Path.Yr,'r');
plot(Vector_X_Ref, Vector_Y_Ref, 'g');
plot(Vector_Xl_Ref, Vector_Yl_Ref, '-g');
plot(Vector_Xr_Ref, Vector_Yr_Ref, '-g');
plot(Vector_x1_Predict, Vector_x2_Predict, '-ko');

figure;
subplot(4,4,1);
plot(Vector_x1_Predict);legend('X');
subplot(4,4,2);
plot(Vector_x2_Predict);legend('Y');
subplot(4,4,3);
plot(Vector_x3_Predict);legend('Phi');
subplot(4,4,4);
plot(Vector_x4_Predict);legend('Vx');
subplot(4,4,5);
plot(Vector_x5_Predict);legend('Vy');
subplot(4,4,6);
plot(Vector_x6_Predict);legend('Phi Dot');
subplot(4,4,7);
plot(Vector_x7_Predict);legend('S');
subplot(4,4,8);
plot(Vector_u1_Predict);legend('T');
subplot(4,4,9);
plot(Vector_u2_Predict);legend('D');
subplot(4,4,10);
plot(Vector_u3_Predict);legend('Vs');
subplot(4,4,11);
plot(Vector_du1_Predict);legend('dT');
subplot(4,4,12);
plot(Vector_du2_Predict);legend('dD');
subplot(4,4,13);
plot(Vector_du2_Predict);legend('dVs');
