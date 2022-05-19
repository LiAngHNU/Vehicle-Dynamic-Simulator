%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hunan University
% Published by: Ang Li
% Email: ang1997@hnu.edu.cn
% Copyright (c) 2022 . All rights reserved.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Passenger Vehicles Template
Vehicle = struct;

Vehicle.Mass = 9070;
Vehicle.Inertia = 35428.5;
Vehicle.Lf = 1.460;
Vehicle.Lr = 2.675;
Vehicle.Cf = -70000;
Vehicle.Cr = -70000;

% Vehicle.Body.Mass     = 1820.0;
% Vehicle.Body.InertiaX = 1023.8;
% Vehicle.Body.InertiaY = 3567.2;
% Vehicle.Body.InertiaZ = 4095.0;
% 
% Vehicle.Axle.Num      = 2;
% Vehicle.Axle.Mass     = [ 116.9   89.5];
% Vehicle.Axle.InertiaZ = [  0.62   0.62];
% Vehicle.Axle.Position = [+1.265 -1.895];
% 
% Vehicle.Tyre.Num      = 4;
% Vehicle.Tyre.Mass     = [28 28 28 28];
% Vehicle.Tyre.InertiaX = [1.3 1.3 1.3 1.3];
% Vehicle.Tyre.InertiaY = [2.1 2.1 2.1 2.1];
% Vehicle.Tyre.InertiaZ = [1.3 1.3 1.3 1.3];
% Vehicle.Tyre.Postion  = [+0.803 -0.803 +0.803 -0.803];
% 
% Vehicle.NominalInfo.Mass = Vehicle.Body.Mass + ...
%                           sum(Vehicle.Axle.Mass) + ...
%                           sum(Vehicle.Tyre.Mass);
% Vehicle.NominalInfo.InertiaZ = Vehicle.Body.InertiaZ + ...
%                                Vehicle.Axle;

% Vehicle.Tyre.Model = load("Exotic_Rear_275_30_19.mat");

% Commercial Vehicles Template