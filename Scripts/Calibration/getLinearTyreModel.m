%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Linear Tyre Model Generator %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Version: v1.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date: 2022/12/01 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Generator Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set Vehicle
Vehicle = "JMC_N800";
% Set Linear Zone
kappa_LinearZone = (-0.07:0.01:0.07)';
alpha_LinearZone = (-5:0.25:5)';
% Load Vehicle
load(strcat("Configs\",Vehicle,"\",Vehicle,"_Calib.mat"));
Fzf = 9.8*Vehicle_Calib.Mf;
Fzr = 9.8*Vehicle_Calib.Mr;
% Load Original Tyre Data
load(strcat("Configs\",Vehicle,"\",Vehicle,"_TyreFx_Original.mat"));
load(strcat("Configs\",Vehicle,"\",Vehicle,"_TyreFy_Original.mat"));
len_Fx = 2*length(Tyre_Fx)-1;   len_Fx_Linear = length(kappa_LinearZone);
len_Fy = 2*length(Tyre_Fy)-1;   len_Fy_Linear = length(alpha_LinearZone);
wid_Fx = width(Tyre_Fx);
wid_Fy = width(Tyre_Fy);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
kappa = [-flip(Tyre_Fx(2:end,1));0;Tyre_Fx(2:end,1)];
alpha = [flip(Tyre_Fy(2:end,1));0;-Tyre_Fy(2:end,1)];
Fz = Tyre_Fy(1,:);
Fx = [-flip(Tyre_Fx(2:end,2:end));zeros(1,wid_Fx-1);Tyre_Fx(2:end,2:end)];
Fx = [zeros(len_Fx,1),Fx]';
Fy = [-flip(Tyre_Fy(2:end,2:end));zeros(1,wid_Fy-1);Tyre_Fy(2:end,2:end)];
Fy = [zeros(len_Fy,1),Fy]';
subplot(2,2,1); hold on; grid on; xlabel('slip ratio'); zlabel('Fx front');
surf(kappa,Fz,Fx,'FaceAlpha',0.5); view(0,0);
subplot(2,2,2); hold on; grid on; xlabel('slip angle'); zlabel('Fy front'); 
surf(alpha,Fz,Fy,'FaceAlpha',0.5); view(0,0);
subplot(2,2,3); hold on; grid on; xlabel('slip ratio'); zlabel('Fx rear'); 
surf(kappa,Fz,Fx,'FaceAlpha',0.5); view(0,0);
subplot(2,2,4); hold on; grid on; xlabel('slip angle'); zlabel('Fy rear'); 
surf(alpha,Fz,Fy,'FaceAlpha',0.5); view(0,0);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Linear Tyre Model Fitting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Front Longitudinal
Fx_Real = interp2(kappa,Fz,Fx,kappa,Fzf*ones(len_Fx,1));
Fx_LinearZone = interp2(kappa,Fz,Fx,kappa_LinearZone,Fzf*ones(len_Fx_Linear,1));
Cxf = polyfit(kappa_LinearZone,Fx_LinearZone,1); Cxf = Cxf(1,1);
subplot(2,2,1); 
plot3(kappa,Fzf*ones(len_Fx,1),Fx_Real,'r-','LineWidth',2);
plot3(kappa_LinearZone,Fzf*ones(len_Fx_Linear,1),Cxf*kappa_LinearZone,'g-','LineWidth',2);
% Front Lateral
Fy_Real = interp2(alpha,Fz,Fy,alpha,Fzf*ones(len_Fy,1));
Fy_LinearZone = interp2(alpha,Fz,Fy,alpha_LinearZone,Fzf*ones(len_Fy_Linear,1));
Cyf = polyfit(alpha_LinearZone,Fy_LinearZone,1); Cyf = Cyf(1,1);
subplot(2,2,2); 
plot3(alpha,Fzf*ones(len_Fy,1),Fy_Real,'r-','LineWidth',2);
plot3(alpha_LinearZone,Fzf*ones(len_Fy_Linear,1),Cyf*alpha_LinearZone,'g-','LineWidth',2);
% Rear Longitudinal
Fx_Real = interp2(kappa,Fz,Fx,kappa,Fzr*ones(len_Fx,1));
Fx_LinearZone = interp2(kappa,Fz,Fx,kappa_LinearZone,Fzr*ones(len_Fx_Linear,1));
Cxr = polyfit(kappa_LinearZone,Fx_LinearZone,1); Cxr = Cxr(1,1);
subplot(2,2,3); 
plot3(kappa,Fzr*ones(len_Fx,1),Fx_Real,'r-','LineWidth',2);
plot3(kappa_LinearZone,Fzr*ones(len_Fx_Linear,1),Cxr*kappa_LinearZone,'g-','LineWidth',2);
% Rear Lateral
Fy_Real = interp2(alpha,Fz,Fy,alpha,Fzr*ones(len_Fy,1));
Fy_LinearZone = interp2(alpha,Fz,Fy,alpha_LinearZone,Fzr*ones(len_Fy_Linear,1));
Cyr = polyfit(alpha_LinearZone,Fy_LinearZone,1); Cyr = Cyr(1,1);
subplot(2,2,4); 
plot3(alpha,Fzr*ones(len_Fy,1),Fy_Real,'r-','LineWidth',2);
plot3(alpha_LinearZone,Fzr*ones(len_Fy_Linear,1),Cyr*alpha_LinearZone,'g-','LineWidth',2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Save Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Tyre_Calib = struct("Cxf",Cxf,...
                    "Cyf",57.3*Cyf,...
                    "Cxr",Cxr,...
                    "Cyr",57.3*Cyr);
save(strcat("Configs\",Vehicle,"\",Vehicle,"_Linear_Tyre.mat"), "Tyre_Calib");