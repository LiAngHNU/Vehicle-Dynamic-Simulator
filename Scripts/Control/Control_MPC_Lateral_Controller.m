%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hunan University
% Published by: Ang Li
% Email: ang1997@hnu.edu.cn
% Copyright (c) 2022 . All rights reserved.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
TS = 0.01;
% Time Step
TS = 0.025;
VX = 15.00;

% Truck: 3A Daimler Auman
MT =   8800;
IZ =54167.2;

LF =  2.048;
LR =  2.575;

KF = -111154;
KR = -146332;

% Vehicle-Road Error Model
A1 =  2*(KF+KR)/(MT*VX);				B1 = -2*KF/MT;
A2 = -2*(KF+KR)/MT;						B2 = -2*KF*LF/IZ;
A3 =  2*(KF*LF-KR*LR)/(MT*VX);
A4 =  2*(KF*LF-KR*LR)/(IZ*VX);			
A5 = -2*(KF*LF-KR*LR)/IZ;				
A6 =  2*(KF*LF*LF+KR*LR*LR)/(IZ*VX);

Ac = [0,1,0,0;0,A1,A2,A3;0,0,0,1;0,A4,A5,A6];
Bc = [0;B1;0;B2];
Cc = diag([1,1,1,1]);
Dc = [0;0;0;0];

sysc = ss(Ac,Bc,Cc,Dc);

% Discrete Vehicle-Road Error Model
sysd = c2d(sysc,TS);

% MPC Controller
Q = diag([10, 0, 0 ,0]);
R = 1;

mpcobj = mpc(sysc,TS);

mpcobj.PredictionHorizon = 80;
mpcobj.ControlHorizon    =  1;
mpcobj.Weights.OutputVariables = {Q};
mpcobj.Weights.ManipulatedVariables = {R};
