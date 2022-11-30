%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Model Predictive Countouring Controller %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Version: v1.1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Date: 2022/11/02 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
nX =        7;      % Number of States
nU =        3;      % Number of Inputs
nA =    nX+nU;      % Number of Augment States
nD =        3;      % Number of Inputs Augments
nT = nX+nU+nD;      % Number of Total Variables
nI =        0;      % Number of Iteration Times
% Set Weights
Qc = 0.001;         % Weight for Coutouring Error
Qt =  0.01;         % Weight for Terminal Coutouring Error
Ql =  1000;         % Weight for Lag Error
Qv = -0.05;         % Weight for S-Direction Velocity
Qe = 99999;         % Weight for Relaxation Factors
Rt =  1e-4;         % Weight for Torque
Rd =  1e-4;         % Weight for Delta
Rv =  1e-6;         % Weight for S-Direction Velocity
Rdt = 1e-2;         % Weight for Torque Augment
Rdd =  1e0;         % Weight for Delta Augment
Rdv = 1e-3;         % Weight for S-Direction Velocity Augment
% Set Constraints
Upper_Bounds = [+75;+70;+2*pi;25;+5;+pi;1250;+250;+0.6;25;+100;+0.3;+5;];
Lower_Bounds = [-45;-75;-2*pi; 1;-5;-pi;   0;-250;-0.6; 1;-100;-0.3;-5;];
% Set Ref Line
load('Scenarios\Scenario_Course\Course_Autocross_No2.mat');
% Set Vehicle
load('Models\Vehicles\VehicleSim\RacingVehicles\HUR_A18\HUR_A18_Calib.mat');
% Set Start Point
Start_ID = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Vectors & Matrices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
stage = struct([]);
for i = 1:1:Tp+1
    stage(i).States = zeros(nX,1);
    stage(i).Inputs = zeros(nU,1);
    stage(i).Deltas = zeros(nD,1);
    stage(i).Ref.X = 0;
    stage(i).Ref.Y = 0;
    stage(i).Ref.Theta = 0;
    stage(i).Ref.Kappa = 0;
    stage(i).Ref.Xl = 0;
    stage(i).Ref.Yl = 0;
    stage(i).Ref.Xr = 0;
    stage(i).Ref.Yr = 0;
    stage(i).Model.Matrix_f  = zeros(nX,  1);
    stage(i).Model.Matrix_Ac = zeros(nX, nX);
    stage(i).Model.Matrix_Bc = zeros(nX,  1);
    stage(i).Model.Matrix_Gc = zeros(nX,  1);
    stage(i).Model.Matrix_Ad = zeros(nX, nX);
    stage(i).Model.Matrix_Bd = zeros(nX,  1);
    stage(i).Model.Matrix_Gd = zeros(nX,  1);
    stage(i).Model.Matrix_Aa = zeros(nA, nA);
    stage(i).Model.Matrix_Ba = zeros(nA,  1);
    stage(i).Model.Matrix_Ga = zeros(nA,  1);
    stage(i).Derivatives.dXdS = 0;
    stage(i).Derivatives.dYdS = 0;
    stage(i).Derivatives.dTdS = 0;
    stage(i).Derivatives.dEcdX = 0;
    stage(i).Derivatives.dEcdY = 0;
    stage(i).Derivatives.dEcdS = 0;
    stage(i).Derivatives.dEldX = 0;
    stage(i).Derivatives.dEldY = 0;
    stage(i).Derivatives.dEldS = 0;
    stage(i).Ec = 0;
    stage(i).El = 0;
    stage(i).Grad_Ec = zeros(nX,1);
    stage(i).Grad_El = zeros(nX,1);
    if i < Tp+1
        stage(i).Matrix_h = zeros(nT,nT);
        stage(i).Matrix_f = zeros(nT, 1);
    else
        stage(i).Matrix_h = zeros(nA,nA);
        stage(i).Matrix_f = zeros(nA, 1);
    end
    stage(i).Ck = zeros(1,nT);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Update States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Update Vehicle Parameters
Ms = Vehicle_Calib.Mass;        Iz = Vehicle_Calib.Inertia;
Lf = Vehicle_Calib.Lf;          Lr = Vehicle_Calib.Lr;
Cf = Vehicle_Calib.Cyf;         Cr = Vehicle_Calib.Cyr;
Re = 0.260;
% Update States
x1 = RefLineInfo.X(Start_ID);      % x1: Vehicle X Coordinate
x2 = RefLineInfo.Y(Start_ID);      % x2: Vehicle Y Coordinate
x3 = RefLineInfo.Tht(Start_ID);  % x3: Vehicle Yaw Angle
x4 = +3.0000;               % x4: Vehicle Lon Velocity
x5 = +0.0000;               % x5: Vehicle Lat Velocity
x6 = +0.0000;               % x6: Vehicle Yaw Rate
x7 = RefLineInfo.S(Start_ID);      % x7: Vehicle Driving Mileage
% Update Inputs
u1 = +0.0000;       % u1: Vehicle Torque
u2 = +0.0000;       % u2: Vehicle Steering Angle
u3 = +0.0000;       % u3: Vehicle S-Direction Velocity
% Update Disturbances
d1 = +0.0000;       % d1: Vehicle Torque Augment
d2 = +0.0000;       % d2: Vehicle Steering Angle Augment
d3 = +0.0000;       % d3: Vehicle S-Direciton Velocity Augment
% Update States
stage(1).States = [x1;x2;x3;x4;x5;x6;x7;];
stage(1).Inputs = [u1;u2;u3;];
stage(1).Deltas = [d1;d2;d3;];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate Ref Line Infos %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:Tp+1
% Update S
stage(i).States(7,1) = x7 + x4*Ts*(i-1);
% Update Ref Line Infos
stage(i).Ref.X = interp1(RefLineInfo.S, RefLineInfo.X, stage(i).States(7));
stage(i).Ref.Y = interp1(RefLineInfo.S, RefLineInfo.Y, stage(i).States(7));
stage(i).Ref.Xl = interp1(RefLineInfo.S, RefLineInfo.Xl, stage(i).States(7));
stage(i).Ref.Yl = interp1(RefLineInfo.S, RefLineInfo.Yl, stage(i).States(7));
stage(i).Ref.Xr = interp1(RefLineInfo.S, RefLineInfo.Xr, stage(i).States(7));
stage(i).Ref.Yr = interp1(RefLineInfo.S, RefLineInfo.Yr, stage(i).States(7));
stage(i).Ref.Theta = interp1(RefLineInfo.S, RefLineInfo.Tht, stage(i).States(7));
stage(i).Ref.Kappa = interp1(RefLineInfo.S, RefLineInfo.Kpa, stage(i).States(7));
% Update States
stage(i).States(1) = stage(i).Ref.X;
stage(i).States(2) = stage(i).Ref.Y;
stage(i).States(3) = stage(i).Ref.Theta;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate Errors %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:Tp+1
stage(i).Ec = - (stage(i).Ref.X - stage(1).States(1))*sin(stage(i).Ref.Theta)...
              + (stage(i).Ref.Y - stage(1).States(2))*cos(stage(i).Ref.Theta);
stage(i).El = + (stage(i).Ref.X - stage(1).States(1))*cos(stage(i).Ref.Theta)...
              + (stage(i).Ref.Y - stage(1).States(2))*sin(stage(i).Ref.Theta);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate Derivatives %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:Tp+1
% Update Ref Line Infos
stage(i).Derivatives.dXdS = interp1(RefLineInfo.S, RefLineInfo.dX, stage(i).States(7));
stage(i).Derivatives.dYdS = interp1(RefLineInfo.S, RefLineInfo.dY, stage(i).States(7));
stage(i).Derivatives.dTdS = interp1(RefLineInfo.S, RefLineInfo.dTht, stage(i).States(7));
% Calculate Derivatives
dX = stage(1).States(1,1) - stage(i).Ref.X;
dY = stage(1).States(2,1) - stage(i).Ref.Y;
dXdS = stage(i).Derivatives.dXdS;
dYdS = stage(i).Derivatives.dYdS;
dTdS = stage(i).Derivatives.dTdS;
stage(i).Derivatives.dEcdX = + sin(stage(i).Ref.Theta);
stage(i).Derivatives.dEcdY = - cos(stage(i).Ref.Theta);
stage(i).Derivatives.dEcdS = [stage(i).Derivatives.dTdS,1]*...
                             [dX,dY;dYdS, -dXdS]*...
                             [cos(stage(i).Ref.Theta);sin(stage(i).Ref.Theta)];
stage(i).Derivatives.dEldX = - cos(stage(i).Ref.Theta);
stage(i).Derivatives.dEldY = - sin(stage(i).Ref.Theta);
stage(i).Derivatives.dEldS = [stage(i).Derivatives.dTdS,1]*...
                             [-dY,dX;dXdS, dYdS]*...
                             [cos(stage(i).Ref.Theta);sin(stage(i).Ref.Theta)];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate Error Gradients %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:Tp+1
% Calculate Gradients
stage(i).Grad_Ec = [stage(i).Derivatives.dEcdX;
                    stage(i).Derivatives.dEcdY;
                                             0;
                                             0;
                                             0;
                                             0;
                    stage(i).Derivatives.dEcdS;];
stage(i).Grad_El = [stage(i).Derivatives.dEldX;
                    stage(i).Derivatives.dEldY;
                                             0;
                                             0;
                                             0;
                                             0;
                    stage(i).Derivatives.dEldS;];
hold on; plot(stage(i).Ref.X,stage(i).Ref.Y,'kx');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Linearize Vehicle Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:Tp+1
% Update Reference Point
x1 = stage(i).States(1,1);      u1 = stage(i).Inputs(1,1);
x2 = stage(i).States(1,1);      u2 = stage(i).Inputs(2,1);
x3 = stage(i).States(1,1);      u3 = stage(i).Inputs(3,1);
x4 = stage(i).States(1,1);
x5 = stage(i).States(1,1);      d1 = stage(i).Deltas(1,1);
x6 = stage(i).States(1,1);      d2 = stage(i).Deltas(2,1);
x7 = stage(i).States(1,1);      d3 = stage(i).Deltas(3,1);
% Update 
Af = (x5 + Lf*x6)/(x4) - u2;
Ar = (x5 - Lr*x6)/(x4);
% System Dynamics
stage(i).Model.Matrix_f(1,1) = x4*cos(x3) - x5*sin(x3);
stage(i).Model.Matrix_f(2,1) = x4*sin(x3) + x5*cos(x3);
stage(i).Model.Matrix_f(3,1) = x6;
stage(i).Model.Matrix_f(4,1) = (1/Ms)*((1/Re)*u1 - Cf*Af*sin(u2) + Ms*x5*x6);
stage(i).Model.Matrix_f(5,1) = (1/Ms)*((Cf+Cr)*x5/x4 + (Cf*Lf-Cr*Lr)*x6/x4 - Cf*u2);
stage(i).Model.Matrix_f(6,1) = (1/Iz)*(Cf*Af*Lf*cos(u2) - Cr*Ar*Lr);
stage(i).Model.Matrix_f(7,1) = u3;
% Linearization & Discretization
stage(i).Model.Matrix_Ac(1,3) = - x5*cos(x3) - x4*sin(x3);
stage(i).Model.Matrix_Ac(1,4) = + cos(x3);
stage(i).Model.Matrix_Ac(1,5) = - sin(x3);

stage(i).Model.Matrix_Ac(2,3) = + x4*cos(x3) - x5*sin(x3);
stage(i).Model.Matrix_Ac(2,4) = + sin(x3);
stage(i).Model.Matrix_Ac(2,5) = + cos(x3);

stage(i).Model.Matrix_Ac(3,6) = + 1;

stage(i).Model.Matrix_Ac(4,4) = + (Cf*sin(u2)*(x5 + Lf*x6))/(Ms*x4^2);
stage(i).Model.Matrix_Ac(4,5) = + (Ms*x6 - (Cf*sin(u2))/x4)/Ms;
stage(i).Model.Matrix_Ac(4,6) = + (Ms*x5 - (Cf*Lf*sin(u2))/x4)/Ms;

stage(i).Model.Matrix_Ac(5,4) = - ((x5*(Cf + Cr))/x4^2 + (x6*(Cf*Lf - Cr*Lr))/x4^2)/Ms;
stage(i).Model.Matrix_Ac(5,5) = + (Cf + Cr)/(Ms*x4);
stage(i).Model.Matrix_Ac(5,6) = + (Cf*Lf - Cr*Lr)/(Ms*x4);

stage(i).Model.Matrix_Ac(6,4) = + ((Cr*Lr*(x5 - Lr*x6))/x4^2 - (Cf*Lf*cos(u2)*(x5 + Lf*x6))/x4^2)/Iz;
stage(i).Model.Matrix_Ac(6,5) = - ((Cr*Lr)/x4 - (Cf*Lf*cos(u2))/x4)/Iz;
stage(i).Model.Matrix_Ac(6,6) = + ((Cf*cos(u2)*Lf^2)/x4 + (Cr*Lr^2)/x4)/Iz;

stage(i).Model.Matrix_Bc(4,1) = + 1/(Ms*Re);
stage(i).Model.Matrix_Bc(4,2) = + (Cf*sin(u2) + Cf*cos(u2)*(u2 - (x5 + Lf*x6)/x4))/Ms;
stage(i).Model.Matrix_Bc(5,2) = - Cf/Ms;
stage(i).Model.Matrix_Bc(6,2) = - (Cf*Lf*cos(u2) - Cf*Lf*sin(u2)*(u2 - (x5 + Lf*x6)/x4))/Iz;
stage(i).Model.Matrix_Bc(7,3) = + 1;

stage(i).Model.Matrix_Gc = stage(i).Model.Matrix_f - stage(i).Model.Matrix_Ac*stage(i).States + stage(i).Model.Matrix_Bc*stage(i).Inputs;

Matrix_Tmp = expm(Ts*[stage(i).Model.Matrix_Ac,stage(i).Model.Matrix_Bc,stage(i).Model.Matrix_Gc;zeros(4,11)]);

stage(i).Model.Matrix_Ad = Matrix_Tmp(1:nX,1:nX);
stage(i).Model.Matrix_Bd = Matrix_Tmp(1:nX,nX+1:nA);
stage(i).Model.Matrix_Gd = Matrix_Tmp(1:nX,nA+1:11);

stage(i).Model.Matrix_Aa = [stage(i).Model.Matrix_Ad, stage(i).Model.Matrix_Bd;...
                                        zeros(nU,nX),                  eye(nU);];
stage(i).Model.Matrix_Ba = [stage(i).Model.Matrix_Bd;...
                                             eye(nU);];
stage(i).Model.Matrix_Ga = [stage(i).Model.Matrix_Gd;...
                                         zeros(nU,1);];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Quadratic Programming Construction %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Quadratic Programming Construction: Cost Function %%%%%%%%%%%%%%%%%%%%%%
% Initialize Matrix_H & Matrix_F
Matrix_H = zeros(nT*Tp+nA, nT*Tp+nA);
Matrix_F = zeros(nT*Tp+nA,        1);
% Calculate Matrix_H & Matrix_F
for i = 1:1:Tp+1
    Matrix_P = [stage(i).Grad_Ec,stage(i).Grad_El]*diag([Qc Ql])*[stage(i).Grad_Ec';stage(i).Grad_El';];
    Matrix_T = [stage(i).Grad_Ec,stage(i).Grad_El]*diag([Qt Ql])*[stage(i).Grad_Ec';stage(i).Grad_El';];
    if i < Tp+1
    % Process Cost
        Matrix_H(nT*(i-1)+1:nT*(i-1)+nX, nT*(i-1)+1:nT*(i-1)+nX) = Matrix_P;
        Matrix_H(nT*(i-1)+8:nT*(i-1)+nT, nT*(i-1)+8:nT*(i-1)+nT) = diag([Rt,Rd,Rv,Rdt,Rdd,Rdv;]);
    else
    % Terminal Cost
        Matrix_H(nT*(i-1)+1:nT*(i-1)+7,  nT*(i-1)+1:nT*(i-1)+7)  = Matrix_T;
        Matrix_H(nT*(i-1)+8:nT*(i-1)+10, nT*(i-1)+8:nT*(i-1)+10) = diag([Rt,Rd,Rv;]);
    end
end
%% Quadratic Programming Construction: Equality Constraints %%%%%%%%%%%%%%%
% Initialize Matrix_Aeq & Matrix_Beq
Matrix_Aeq = zeros((nX+nU)*(Tp+1),nT*Tp+nX+nU);
Matrix_Beq = zeros((nX+nU)*(Tp+1),          1);
Matrix_Aeq(1:nX+nU,1:nX+nU) = eye(nX+nU);
Matrix_Beq(1:nX+nU,      1) = [stage(i).States;stage(i).Inputs];
% Calculate Matrix_Aeq & Matrix_Beq
for i = 1:1:Tp+1
    if i < Tp+1
        Matrix_Aeq((nX+nU)*(i-1)+1:(nX+nU)*i,nT*(i-1)+1:nT*(i-1)+10) = eye(nX+nU);
        Matrix_Aeq((nX+nU)*i+1:(nX+nU)*(i+1),nT*(i-1)+1:nT*(i-1)+10) = -stage(i).Model.Matrix_Aa;
        Matrix_Aeq((nX+nU)*i+1:(nX+nU)*(i+1),nT*(i-1)+11:nT*i) = -stage(i).Model.Matrix_Ba;
    else
        Matrix_Aeq((nX+nU)*(i-1)+1:(nX+nU)*i,nT*(i-1)+1:nT*(i-1)+10) = eye(nX+nU);
    end
    if i < 2
        Matrix_Beq(1:nX+nU,1) = [stage(1).States;stage(1).Inputs];
    else
        Matrix_Beq((nX+nU)*(i-1)+1:(nX+nU)*i,1) = stage(i).Model.Matrix_Ga;
    end
end
%% Quadratic Programming Construction: Inequality Constraints %%%%%%%%%%%%%
% State Constraints
for i = 1:1:Tp+1
    if i < Tp+1
        Matrix_UB(nT*(i-1)+1:nT*i,1) = Upper_Bounds;
        Matrix_LB(nT*(i-1)+1:nT*i,1) = Lower_Bounds;
    else
        Matrix_UB(nT*(i-1)+1:nT*(i-1)+nA) = Upper_Bounds(1:nA,1);
        Matrix_LB(nT*(i-1)+1:nT*(i-1)+nA) = Lower_Bounds(1:nA,1);
    end
end
% Track Contraint
for i = 1:1:Tp+1
    stage(i).Ck(1,1) = -(stage(i).Ref.Xr - stage(i).Ref.Xl);
    stage(i).Ck(1,2) = -(stage(i).Ref.Yr - stage(i).Ref.Yl);
    stage(i).UB = max(-(stage(i).Ref.Xr - stage(i).Ref.Xl)*stage(i).Ref.Xl - (stage(i).Ref.Yr - stage(i).Ref.Yl)*stage(i).Ref.Yl,...
                      -(stage(i).Ref.Xr - stage(i).Ref.Xl)*stage(i).Ref.Xr - (stage(i).Ref.Yr - stage(i).Ref.Yl)*stage(i).Ref.Yr);
    stage(i).LB = min(-(stage(i).Ref.Xr - stage(i).Ref.Xl)*stage(i).Ref.Xl - (stage(i).Ref.Yr - stage(i).Ref.Yl)*stage(i).Ref.Yl,...
                      -(stage(i).Ref.Xr - stage(i).Ref.Xl)*stage(i).Ref.Xr - (stage(i).Ref.Yr - stage(i).Ref.Yl)*stage(i).Ref.Yr);
end
Matrix_Atr_UB = zeros(Tp+1,nT*Tp+nA);
Matrix_Atr_LB = zeros(Tp+1,nT*Tp+nA);
Matrix_Btr_UB = zeros(Tp+1,1);
Matrix_Btr_LB = zeros(Tp+1,1);
for i = 1:1:Tp+1
    if i < Tp+1
        Matrix_Atr_UB(i,nT*(i-1)+1:nT*i) = +stage(i).Ck;
        Matrix_Atr_LB(i,nT*(i-1)+1:nT*i) = -stage(i).Ck;
        Matrix_Btr_UB(i,1) = +stage(i).UB;
        Matrix_Btr_LB(i,1) = -stage(i).LB;
    else
        Matrix_Atr_UB(i,nT*(i-1)+1:nT*i-3) = +stage(i).Ck(:,1:10);
        Matrix_Atr_LB(i,nT*(i-1)+1:nT*i-3) = -stage(i).Ck(:,1:10);
    end
end
Matrix_Atr = [Matrix_Atr_UB;...
              Matrix_Atr_LB;];
Matrix_Btr = [Matrix_Btr_UB;...
              Matrix_Btr_LB;];
% Input Constraint
Matrix_Ain_UB = zeros(nT*Tp+nA,nT*Tp+nA);
Matrix_Ain_LB = zeros(nT*Tp+nA,nT*Tp+nA);
Matrix_Bin_UB = zeros(nT*Tp+nA,1);
Matrix_Bin_LB = zeros(nT*Tp+nA,1);
for i = 1:1:Tp+1
    if i < Tp+1
        Matrix_Ain_UB(nT*(i-1)+8:nT*i,nT*(i-1)+8:nT*i) = +diag([0 1 0 0 0 0]);
        Matrix_Ain_LB(nT*(i-1)+8:nT*i,nT*(i-1)+8:nT*i) = -diag([0 1 0 0 0 0]);
        Matrix_Bin_UB(nT*(i-1)+8:nT*i,1) = [0;0.6;0;0;0;0;];
        Matrix_Bin_LB(nT*(i-1)+8:nT*i,1) = [0;0.6;0;0;0;0;];
    else
        Matrix_Ain_UB(nT*(i-1)+8:nT*(i-1)+10,nT*(i-1)+8:nT*(i-1)+10) = +diag([0 1 0]);
        Matrix_Ain_LB(nT*(i-1)+8:nT*(i-1)+10,nT*(i-1)+8:nT*(i-1)+10) = -diag([0 1 0]);
        Matrix_Bin_UB(nT*(i-1)+8:nT*(i-1)+10,1) = [0;0.6;0;];
        Matrix_Bin_LB(nT*(i-1)+8:nT*(i-1)+10,1) = [0;0.6;0;];
    end
end
Matrix_Ain = [Matrix_Ain_UB;...
              Matrix_Ain_LB;];
Matrix_Bin = [Matrix_Bin_UB;...
              Matrix_Bin_LB;];
% Incorporate Constraint
Matrix_Aneq = [Matrix_Atr;...
               Matrix_Ain;];
Matrix_Bneq = [Matrix_Btr;...
               Matrix_Bin;];
%% Quadratic Programming Solution %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Qp_Ans = quadprog(Matrix_H, Matrix_F,...
                  [], [],...
                  Matrix_Aeq, Matrix_Beq,...
                  [], []);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Post Process %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:Tp
    x1_Predict(i,1) = Qp_Ans(nT*(i-1)+1,:);
    x2_Predict(i,1) = Qp_Ans(nT*(i-1)+2,:);
    x3_Predict(i,1) = Qp_Ans(nT*(i-1)+3,:);
    x4_Predict(i,1) = Qp_Ans(nT*(i-1)+4,:);
    x5_Predict(i,1) = Qp_Ans(nT*(i-1)+5,:);
    x6_Predict(i,1) = Qp_Ans(nT*(i-1)+6,:);
    x7_Predict(i,1) = Qp_Ans(nT*(i-1)+7,:);
    u1_Predict(i,1) = Qp_Ans(nT*(i-1)+8,:);
    u2_Predict(i,1) = Qp_Ans(nT*(i-1)+9,:);
    u3_Predict(i,1) = Qp_Ans(nT*(i-1)+10,:);
    d1_Predict(i,1) = Qp_Ans(nT*(i-1)+11,:);
    d2_Predict(i,1) = Qp_Ans(nT*(i-1)+12,:);
    d3_Predict(i,1) = Qp_Ans(nT*(i-1)+13,:);
end
plot(RefLineInfo.X, RefLineInfo.Y, 'b--');
plot(RefLineInfo.Xl,RefLineInfo.Yl,'r');
plot(RefLineInfo.Xr,RefLineInfo.Yr,'r');
plot(x1_Predict, x2_Predict, 'g');