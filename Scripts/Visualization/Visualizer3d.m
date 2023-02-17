%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3D Visualizer %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Version: v1.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
% Date: 2023/01/13 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualizer Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic;
% Basic Settings
DrawSize = 500;
StartID =  751;
% Render Settings
hold on; axis equal; grid on;
xlim([-15 +15]); ylim([-10 +90]); zlim([0 10]);
% Axis Settings
set(gca,'XTick',[-15,-10,-5,0,+5,+10,+15]);
set(gca,'YTick',[-10,0,+90]);
set(gca,'ZTick',[0,+5,+10]);
% Color Settings
set(gcf,'Color','#0a3156');
set(gca,'Color','none');
set(gca,'XColor','#aaaaaa');
set(gca,'YColor','#aaaaaa');
set(gca,'ZColor','#aaaaaa');
set(gca,'GridColor','#aaaaaa');
set(gca,'MinorGridColor','#aaaaaa');
% Alpha Settings
set(gca,'GridAlpha',1.00);
set(gca,'MinorGridAlpha',0.75);
% Grid Settings
set(gca,'XMinorGrid','on');
set(gca,'YMinorGrid','on');
set(gca,'ZMinorGrid','on');
% Camera Settings
set(gca,'Projection','perspective'); view(0,85);
set(gca,'CameraPositionMode','manual'); set(gca,'CameraPosition',[0,-50,30]);
set(gca,'CameraTargetMode','manual'); set(gca,'CameraTarget',[0,20,0]);
set(gca,'CameraViewAngleMode','manual'); set(gca,'CameraViewAngle',35);
% Vehicle Shape Settings
Veh_Length = 10;
Veh_Width = 2;
Veh_Height = 2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualizer Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('Scenarios\2001_mcity_line_01.mat');
Veh_X = RefLineInfo.X(StartID);
Veh_Y = RefLineInfo.Y(StartID);
Veh_Tht = RefLineInfo.Tht(StartID);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Vectors & Matrices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Xm = zeros(DrawSize,1); Xm_New = zeros(DrawSize,1);
Xl = zeros(DrawSize,1); Xl_New = zeros(DrawSize,1);
Xr = zeros(DrawSize,1); Xr_New = zeros(DrawSize,1);
Ym = zeros(DrawSize,1); Ym_New = zeros(DrawSize,1);
Yl = zeros(DrawSize,1); Yl_New = zeros(DrawSize,1);
Yr = zeros(DrawSize,1); Yr_New = zeros(DrawSize,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Update Coordinates %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:DrawSize
    Xm(i,1) = RefLineInfo.X(StartID+i-101);
    Xl(i,1) = RefLineInfo.Xl(StartID+i-101);
    Xr(i,1) = RefLineInfo.Xr(StartID+i-101);
    Ym(i,1) = RefLineInfo.Y(StartID+i-101);
    Yl(i,1) = RefLineInfo.Yl(StartID+i-101);
    Yr(i,1) = RefLineInfo.Yr(StartID+i-101);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% HUD Rendering %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plot3([-15 -15],[-10 +90],[0,0],'g-','LineWidth',2);
plot3([+15 +15],[-10 +90],[0,0],'g-','LineWidth',2);
plot3([-15 +15],[0 0],[0,0],'g-','LineWidth',2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lane Rendering %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Coordinate Transformation
Xm_New = +(Xm - Veh_X)*cos(Veh_Tht - pi/2)...
         +(Ym - Veh_Y)*sin(Veh_Tht - pi/2);
Xl_New = +(Xl - Veh_X)*cos(Veh_Tht - pi/2)...
         +(Yl - Veh_Y)*sin(Veh_Tht - pi/2);
Xr_New = +(Xr - Veh_X)*cos(Veh_Tht - pi/2)...
         +(Yr - Veh_Y)*sin(Veh_Tht - pi/2);
Ym_New = -(Xm - Veh_X)*sin(Veh_Tht - pi/2)...
         +(Ym - Veh_Y)*cos(Veh_Tht - pi/2);
Yl_New = -(Xl - Veh_X)*sin(Veh_Tht - pi/2)...
         +(Yl - Veh_Y)*cos(Veh_Tht - pi/2);
Yr_New = -(Xr - Veh_X)*sin(Veh_Tht - pi/2)...
         +(Yr - Veh_Y)*cos(Veh_Tht - pi/2);
plot3(Xm_New,Ym_New,zeros(DrawSize,1),'y--','LineWidth',2);
plot3(Xl_New,Yl_New,zeros(DrawSize,1),'w-','LineWidth',1.5);
plot3(Xr_New,Yr_New,zeros(DrawSize,1),'w-','LineWidth',1.5);
disp(toc);