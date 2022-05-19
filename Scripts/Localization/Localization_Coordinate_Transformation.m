%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hunan University
% Published by: Ang Li
% Email: ang1997@hnu.edu.cn
% Copyright (c) 2022 . All rights reserved.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all; clear; clc;

% Load Path Info
load("Scenarios\Path_HandlingCourse.mat");

% Update Vehicle Position & Attitude
Veh_X_Ref = 40.8;
Veh_Y_Ref = 32.5;
Veh_Yz_Ref = 0.26;

% Coordinate Transformation
Path_X_VEH = zeros(length(Path.Index), 1);
Path_Y_VEH = zeros(length(Path.Index), 1);
for i = 1:1:length(Path.Index)
    Path_X_VEH(i,1) = (Path.X(i,1) - Veh_X_Ref)*cos(Veh_Yz_Ref) + (Path.Y(i,1) - Veh_Y_Ref)*sin(Veh_Yz_Ref);
    Path_Y_VEH(i,1) =-(Path.X(i,1) - Veh_X_Ref)*sin(Veh_Yz_Ref) + (Path.Y(i,1) - Veh_Y_Ref)*cos(Veh_Yz_Ref);
end

% Visualization
hold on;
plot(Path.X, Path.Y);
plot(Path_X_VEH, Path_Y_VEH);
plot(Veh_X_Ref, Veh_Y_Ref, 'o');