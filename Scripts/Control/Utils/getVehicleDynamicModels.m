%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create Vehicle Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Version: v1.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date: 2022/10/13 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Matrix_Ad, Matrix_Bd, Matrix_Gd] = ...
    getVehicleDynamicModels(ModelType, VehicleType, Vx, Ts)
% ModelType = 1: 2DOF, Linear, Error-Based, FWS, w/o Integral, w/o Lag, w/o Delay, w/o Preview
% ModelType = 2: 2DOF, Linear, Error-Based, RWS, w/o Integral, w/o Lag, w/o Delay, w/o Preview
% ModelType = 3: 2DOF, Linear, Error-Based, 4WS, w/o Integral, w/o Lag, w/o Delay, w/o Preview
% ModelType = 4: 2DOF, Linear, Error-Based, FWS, w/o Integral, w/o Lag, w/o Delay, w/  Single Preview
% ModelType = 5: 2DOF, Linear, Error-Based, RWS, w/o Integral, w/o Lag, w/o Delay, w/  Single Preview
% ModelType = 6: 2DOF, Linear, Error-Based, 4WS, w/o Integral, w/o Lag, w/o Delay, w/  Single Preview
% ModelType = 7: 2DOF, Linear, Error-Based, 4WS, w/o Integral, w/o Lag, w/o Delay, w/  Double Preview (Dev)
% ModelType = 8: 2DOF, Linear, Error-Based, 4WS, w/  Integral. w/  Lag, w/  Delay, w/  Multi Preview
% ModelType = 99:6DOF,
load("Models\Vehicles\VehicleSim\RacingVehicles\HUR_A18\HUR_A18_Calib.mat");
Ms = Vehicle_Calib.Mass;
Iz = Vehicle_Calib.Inertia;
Lf = Vehicle_Calib.Lf;
Lr = Vehicle_Calib.Lr;
Cf = Vehicle_Calib.Cyf;
Cr = Vehicle_Calib.Cyr;
switch ModelType
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Model Type: 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Not Verified %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
case 1
nX = 4;         Matrix_Ac = zeros(nX,nX);
nU = 1;         Matrix_Bc = zeros(nX,nU);
nV = 1;         Matrix_Gc = zeros(nX,nV);

Matrix_Ac(1,2) =  1;
Matrix_Ac(2,2) =  2*(Cf+Cr)/(Ms*Vx);
Matrix_Ac(2,3) = -2*(Cf+Cr)/(Ms);
Matrix_Ac(2,4) =  2*(Cf*Lf-Cr*Lr)/(Ms*Vx);
Matrix_Ac(3,4) =  1;
Matrix_Ac(4,2) =  2*(Cf*Lf-Cr*Lr)/(Iz*Vx);
Matrix_Ac(4,3) = -2*(Cf*Lf-Cr*Lr)/(Iz);
Matrix_Ac(4,4) =  2*(Cf*Lf*Lf+Cr*Lr*Lr)/(Iz*Vx);

Matrix_Bc(2,1) = -2*Cf/Ms;
Matrix_Bc(4,1) = -2*Cf*Lf/Iz;

Matrix_Gc(2,1) = 2*(Cf*Lf-Cr*Lr)/Ms - Vx^2;
Matrix_Gc(4,1) = 2*(Cf*Lf*Lf+Cr*Lr*Lr)/(Iz*Vx);

Matrix_Ad = eye(size(Matrix_Ac)) + Ts*Matrix_Ac;
Matrix_Bd =                        Ts*Matrix_Bc;
Matrix_Gd =                        Ts*Matrix_Gc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Model Type: 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Not Verified %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
case 2
nX = 4;         Matrix_Ac = zeros(nX,nX);
nU = 1;         Matrix_Bc = zeros(nX,nU);
nV = 1;         Matrix_Gc = zeros(nX,nV);

Matrix_Ac(1,2) =  1;
Matrix_Ac(2,2) =  2*(Cf+Cr)/(Ms*Vx);
Matrix_Ac(2,3) = -2*(Cf+Cr)/(Ms);
Matrix_Ac(2,4) =  2*(Cf*Lf-Cr*Lr)/(Ms*Vx);
Matrix_Ac(3,4) =  1;
Matrix_Ac(4,2) =  2*(Cf*Lf-Cr*Lr)/(Iz*Vx);
Matrix_Ac(4,3) = -2*(Cf*Lf-Cr*Lr)/(Iz);
Matrix_Ac(4,4) =  2*(Cf*Lf*Lf+Cr*Lr*Lr)/(Iz*Vx);

Matrix_Bc(2,1) = -2*Cr/Ms;
Matrix_Bc(4,1) =  2*Cr*Lr/Iz;

Matrix_Gc(2,1) = 2*(Cf*Lf-Cr*Lr)/Ms - Vx^2;
Matrix_Gc(4,1) = 2*(Cf*Lf*Lf+Cr*Lr*Lr)/(Iz*Vx);

Matrix_Ad = eye(size(Matrix_Ac)) + Ts*Matrix_Ac;
Matrix_Bd =                        Ts*Matrix_Bc;
Matrix_Gd =                        Ts*Matrix_Gc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Model Type: 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Not Verified %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
case 3
nX = 4;         Matrix_Ac = zeros(nX,nX);
nU = 2;         Matrix_Bc = zeros(nX,nU);
nV = 1;         Matrix_Gc = zeros(nX,nV);

Matrix_Ac(1,2) =  1;
Matrix_Ac(2,2) =  2*(Cf+Cr)/(Ms*Vx);
Matrix_Ac(2,3) = -2*(Cf+Cr)/(Ms);
Matrix_Ac(2,4) =  2*(Cf*Lf-Cr*Lr)/(Ms*Vx);
Matrix_Ac(3,4) =  1;
Matrix_Ac(4,2) =  2*(Cf*Lf-Cr*Lr)/(Iz*Vx);
Matrix_Ac(4,3) = -2*(Cf*Lf-Cr*Lr)/(Iz);
Matrix_Ac(4,4) =  2*(Cf*Lf*Lf+Cr*Lr*Lr)/(Iz*Vx);

Matrix_Bc(2,1) = -2*Cf/Ms;
Matrix_Bc(2,2) = -2*Cr/Ms;
Matrix_Bc(4,1) = -2*Cf*Lf/Iz;
Matrix_Bc(4,2) =  2*Cr*Lr/Iz;

Matrix_Gc(2,1) = 2*(Cf*Lf-Cr*Lr)/Ms - Vx^2;
Matrix_Gc(4,1) = 2*(Cf*Lf*Lf+Cr*Lr*Lr)/(Iz*Vx);

Matrix_Ad = eye(size(Matrix_Ac)) + Ts*Matrix_Ac;
Matrix_Bd =                        Ts*Matrix_Bc;
Matrix_Gd =                        Ts*Matrix_Gc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Model Type: 4 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Not Verified %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
case 4
nX = 4;         Matrix_Ac = zeros(nX,nX);
nU = 1;         Matrix_Bc = zeros(nX,nU);
nV = 1;         Matrix_Gc = zeros(nX,nV);

Matrix_Ac(1,2) =  1;
Matrix_Ac(2,2) =  2*(Cf+Cr)/(Ms*Vx) + 2*(Cf*Lf*Lp-Cr*Lr*Lp)/(Iz*Vx);
Matrix_Ac(2,3) = -2*(Cf+Cr)/Ms - 2*(Cf*Lf*Lp-Cr*Lr*Lp)/(Iz);
Matrix_Ac(2,4) =  2*(Cf*(Lf-Lp)-Cr*(Lr+Lp))/(Ms*Vx) + 2*(Cf*Lf*Lp*(Lf-Lp)+Cr*Lr*Lp*(Lr+Lp))/(Iz*Vx);
Matrix_Ac(3,4) =  1;
Matrix_Ac(4,2) =  2*(Cf*Lf-Cr*Lr)/(Iz*Vx);
Matrix_Ac(4,3) = -2*(Cf*Lf-Cr*Lr)/(Iz);
Matrix_Ac(4,4) =  2*(Cf*Lf*(Lf-Lp)+Cr*Lr*(Lr+Lp))/(Iz*Vx);

Matrix_Bc(2,1) = -2*Cf/Ms - 2*Cf*Lf*Lp/Iz;
Matrix_Bc(4,1) = -2*Cf*Lf/Iz;

Matrix_Gc(2,1) = 2*(Cf*(Lf-Lp)-Cr*(Lr+Lp))/Ms + 2*(Cf*Lf*Lp*(Lf-Lp) + Cr*Lr*Lp*(Lr+Lp))/Iz - Vx^2;
Matrix_Gc(4,1) = 2*(Cf*Lf*(Lf-Lp)+Cr*Lr*(Lr+Lp))/(Iz*Vx);

Matrix_Ad = eye(size(Matrix_Ac)) + Ts*Matrix_Ac;
Matrix_Bd =                        Ts*Matrix_Bc;
Matrix_Gd =                        Ts*Matrix_Gc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Model Type: 5 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Not Verified %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
case 5
nX = 4;         Matrix_Ac = zeros(nX,nX);
nU = 1;         Matrix_Bc = zeros(nX,nU);
nV = 1;         Matrix_Gc = zeros(nX,nV);

Matrix_Ac(1,2) =  1;
Matrix_Ac(2,2) =  2*(Cf+Cr)/(Ms*Vx) + 2*(Cf*Lf*Lp-Cr*Lr*Lp)/(Iz*Vx);
Matrix_Ac(2,3) = -2*(Cf+Cr)/Ms - 2*(Cf*Lf*Lp-Cr*Lr*Lp)/(Iz);
Matrix_Ac(2,4) =  2*(Cf*(Lf-Lp)-Cr*(Lr+Lp))/(Ms*Vx) + 2*(Cf*Lf*Lp*(Lf-Lp)+Cr*Lr*Lp*(Lr+Lp))/(Iz*Vx);
Matrix_Ac(3,4) =  1;
Matrix_Ac(4,2) =  2*(Cf*Lf-Cr*Lr)/(Iz*Vx);
Matrix_Ac(4,3) = -2*(Cf*Lf-Cr*Lr)/(Iz);
Matrix_Ac(4,4) =  2*(Cf*Lf*(Lf-Lp)+Cr*Lr*(Lr+Lp))/(Iz*Vx);

Matrix_Bc(2,2) = -2*Cr/Ms + 2*Cr*Lr*Lp/Iz;
Matrix_Bc(4,2) =  2*Cr*Lr/Iz;

Matrix_Gc(2,1) = 2*(Cf*(Lf-Lp)-Cr*(Lr+Lp))/Ms + 2*(Cf*Lf*Lp*(Lf-Lp) + Cr*Lr*Lp*(Lr+Lp))/Iz - Vx^2;
Matrix_Gc(4,1) = 2*(Cf*Lf*(Lf-Lp)+Cr*Lr*(Lr+Lp))/(Iz*Vx);

Matrix_Ad = eye(size(Matrix_Ac)) + Ts*Matrix_Ac;
Matrix_Bd =                        Ts*Matrix_Bc;
Matrix_Gd =                        Ts*Matrix_Gc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Model Type: 6 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Not Verified %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
case 6
nX = 4;         Matrix_Ac = zeros(nX,nX);
nU = 2;         Matrix_Bc = zeros(nX,nU);
nV = 1;         Matrix_Gc = zeros(nX,nV);

Matrix_Ac(1,2) =  1;
Matrix_Ac(2,2) =  2*(Cf+Cr)/(Ms*Vx) + 2*(Cf*Lf*Lp-Cr*Lr*Lp)/(Iz*Vx);
Matrix_Ac(2,3) = -2*(Cf+Cr)/Ms - 2*(Cf*Lf*Lp-Cr*Lr*Lp)/(Iz);
Matrix_Ac(2,4) =  2*(Cf*(Lf-Lp)-Cr*(Lr+Lp))/(Ms*Vx) + 2*(Cf*Lf*Lp*(Lf-Lp)+Cr*Lr*Lp*(Lr+Lp))/(Iz*Vx);
Matrix_Ac(3,4) =  1;
Matrix_Ac(4,2) =  2*(Cf*Lf-Cr*Lr)/(Iz*Vx);
Matrix_Ac(4,3) = -2*(Cf*Lf-Cr*Lr)/(Iz);
Matrix_Ac(4,4) =  2*(Cf*Lf*(Lf-Lp)+Cr*Lr*(Lr+Lp))/(Iz*Vx);

Matrix_Bc(2,1) = -2*Cf/Ms - 2*Cf*Lf*Lp/Iz;
Matrix_Bc(2,2) = -2*Cr/Ms + 2*Cr*Lr*Lp/Iz;
Matrix_Bc(4,1) = -2*Cf*Lf/Iz;
Matrix_Bc(4,2) =  2*Cr*Lr/Iz;

Matrix_Gc(2,1) = 2*(Cf*(Lf-Lp)-Cr*(Lr+Lp))/Ms + 2*(Cf*Lf*Lp*(Lf-Lp) + Cr*Lr*Lp*(Lr+Lp))/Iz - Vx^2;
Matrix_Gc(4,1) = 2*(Cf*Lf*(Lf-Lp)+Cr*Lr*(Lr+Lp))/(Iz*Vx);

Matrix_Ad = eye(size(Matrix_Ac)) + Ts*Matrix_Ac;
Matrix_Bd =                        Ts*Matrix_Bc;
Matrix_Gd =                        Ts*Matrix_Gc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Model Type: 7 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Not Verified %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
case 7
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Model Type: 7 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Not Verified %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
case 8
nX = 7;         Matrix_Ac = zeros(nX,nX);
nU = 2;         Matrix_Bc = zeros(nX,nU);
nV = 1;         Matrix_Gc = zeros(nX,nV);

Matrix_Ac(1,2) =  1;
Matrix_Ac(2,3) =  1;
Matrix_Ac(3,3) =  2*(Cf+Cr)/(Ms*Vx) + 2*(Cf*Lf*Lp-Cr*Lr*Lp)/(Iz*Vx);
Matrix_Ac(3,4) = -2*(Cf+Cr)/Ms - 2*(Cf*Lf*Lp-Cr*Lr*Lp)/(Iz);
Matrix_Ac(3,5) =  2*(Cf*(Lf-Lp)-Cr*(Lr+Lp))/(Ms*Vx) + 2*(Cf*Lf*Lp*(Lf-Lp)+Cr*Lr*Lp*(Lr+Lp))/(Iz*Vx);
Matrix_Ac(3,6) =  -2*Cf/Ms - 2*Cf*Lf*Lp/Iz;
Matrix_Ac(3,7) =  -2*Cr/Ms + 2*Cr*Lr*Lp/Iz;
Matrix_Ac(4,5) =  1;
Matrix_Ac(5,3) =  2*(Cf*Lf-Cr*Lr)/(Iz*Vx);
Matrix_Ac(5,4) = -2*(Cf*Lf-Cr*Lr)/(Iz);
Matrix_Ac(5,5) =  2*(Cf*Lf*(Lf-Lp)+Cr*Lr*(Lr+Lp))/(Iz*Vx);
Matrix_Ac(5,6) = -2*Cf*Lf/Iz;
Matrix_Ac(5,7) =  2*Cr*Lr/Iz;
Matrix_Ac(6,6) = -1/Tl;
Matrix_Ac(6,7) = -1/Tl;

Matrix_Bc(6,1) =  1/Tl;
Matrix_Bc(7,2) =  1/Tl;

Matrix_Gc(3,1) = 2*(Cf*(Lf-Lp)-Cr*(Lr+Lp))/Ms + 2*(Cf*Lf*Lp*(Lf-Lp) + Cr*Lr*Lp*(Lr+Lp))/Iz - Vx^2;
Matrix_Gc(5,1) = 2*(Cf*Lf*(Lf-Lp)+Cr*Lr*(Lr+Lp))/(Iz*Vx);

Matrix_Ad = eye(size(Matrix_Ac)) + Ts*Matrix_Ac;
Matrix_Bd =                        Ts*Matrix_Bc;
Matrix_Gd =                        Ts*Matrix_Gc;
end