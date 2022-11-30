%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simplified Pacejka Tyre Model Generator %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Version: v1.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date: 2022/11/29 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Generator Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set Vehicle
Vehicle = "JMC_N800";
% Load Vehicle
load(strcat("Configs\",Vehicle,"\",Vehicle,"_Calib.mat"));
Fzf = 9.8*Vehicle_Calib.Mf;
Fzr = 9.8*Vehicle_Calib.Mr;
% Load Original Tyre Data
load(strcat("Configs\",Vehicle,"\",Vehicle,"_TyreFx_Original.mat"));
load(strcat("Configs\",Vehicle,"\",Vehicle,"_TyreFy_Original.mat"));
len_Fx = 2*length(Tyre_Fx)-1;
len_Fy = 2*length(Tyre_Fy)-1;
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
%% Simlified Pacejka Tyre Model Fitting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
kappa_Pacejka = kappa;
alpha_Pacejka = alpha;
Fx_Pacejka = interp2(kappa,Fz,Fx,kappa_Pacejka,Fzf*ones(len_Fx,1));
subplot(2,2,1); 
plot3(kappa_Pacejka,Fzf*ones(len_Fx,1),Fx_Pacejka,'r-','LineWidth',2);
Bxf = +450;
Cxf = +1.600;
Dxf = +0.800*Fzf;
Fx_Pacejka = Dxf*sin(Cxf*atan(Bxf*kappa_Pacejka/57.3));
subplot(2,2,1); 
plot3(kappa_Pacejka,Fzf*ones(len_Fx,1),Fx_Pacejka,'go:','LineWidth',2);

Fy_Pacejka = interp2(alpha,Fz,Fy,alpha_Pacejka,Fzf*ones(len_Fy,1));
subplot(2,2,2); 
plot3(alpha_Pacejka,Fzf*ones(len_Fy,1),Fy_Pacejka,'r-','LineWidth',2);
Byf = +6.875;
Cyf = +1.625;
Dyf = -0.775*Fzf;
Fy_Pacejka = Dyf*sin(Cyf*atan(Byf*alpha_Pacejka/57.3));
subplot(2,2,2); 
plot3(alpha_Pacejka,Fzf*ones(len_Fy,1),Fy_Pacejka,'go:','LineWidth',2);

Fx_Pacejka = interp2(kappa,Fz,Fx,kappa_Pacejka,Fzr*ones(len_Fx,1));
subplot(2,2,3); 
plot3(kappa_Pacejka,Fzr*ones(len_Fx,1),Fx_Pacejka,'r-','LineWidth',2);
Bxr = +450;
Cxr = +1.600;
Dxr = +0.825*Fzr;
Fx_Pacejka = Dxr*sin(Cxr*atan(Bxr*kappa_Pacejka/57.3));
subplot(2,2,3); 
plot3(kappa_Pacejka,Fzr*ones(len_Fx,1),Fx_Pacejka,'go:','LineWidth',2);

Fy_Pacejka = interp2(alpha,Fz,Fy,alpha_Pacejka,Fzr*ones(len_Fy,1));
subplot(2,2,4); 
plot3(alpha_Pacejka,Fzr*ones(len_Fy,1),Fy_Pacejka,'r-','LineWidth',2);
Byr = +6.875;
Cyr = +1.625;
Dyr = -0.800*Fzr;
Fy_Pacejka = Dyr*sin(Cyr*atan(Byr*alpha_Pacejka/57.3));
subplot(2,2,4); 
plot3(alpha_Pacejka,Fzr*ones(len_Fy,1),Fy_Pacejka,'go:','LineWidth',2);

Tyre_Calib = struct("Lon_Front_Coeffs",[Bxf;Cxf;Dxf],...
                    "Lat_Front_Coeffs",[Byf;Cyf;Dyf],...
                    "Lon_Rear_Coeffs",[Bxr;Cxr;Dxr],...
                    "Lat_Rear_Coeffs",[Byr;Cyr;Dyr]);
save(strcat("Configs\",Vehicle,"\",Vehicle,"_Pacejka_Tyre.mat"), "Tyre_Calib");