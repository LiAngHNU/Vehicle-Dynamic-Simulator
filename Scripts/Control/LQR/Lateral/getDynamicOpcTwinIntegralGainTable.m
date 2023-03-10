%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Offline Lateral Dynamic Twin Integral Optimal Preview Controller %%%%%%%%
% Version: v1.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date: 2023/03/10 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc; tic;                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Controller Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%**** Set Vehicle ********************************************************%
FLAG_VEHICLE_ = "Audi_R8_Etron";                                          %
FLAG_STEER_TYPE_ = "4WS";                                                 %
%**** Load Vehicle *******************************************************%
load(strcat("Configs\",FLAG_VEHICLE_,"\Vehicle\Vehicle_Calib.mat"));      %
Ms = Vehicle_Calib.Ms;          Iz = Vehicle_Calib.Iz;                    %
Lf = Vehicle_Calib.Lf;          Lr = Vehicle_Calib.Lr;                    %
Cf = Vehicle_Calib.Cyf;         Cr = Vehicle_Calib.Cyr;                   %
%**** Set Steps **********************************************************%
tS =  0.01;     % Time Gap of Discretization                              %
tL =  0.20;     % First-Order Lag Coefficient of Front Steer Actuator     % 
tP =    50;     % Horinzon Length of Preview                              % 
%**** Set Weights ********************************************************%
Vx_Table = [ 5.0,10.0,15.0,20.0,25.0,30.0,35.0,40.0,45.0,50.0]/2;         %
q1_Table = [   1,   1,   1,   1,   1,   1,   1,   1,   1,   1];           %
q2_Table = [   1,   1,   1,   1,   1,   1,   1,   1,   1,   1];           %
q3_Table = [   1,   1,   1,   1,   1,   1,   1,   1,   1,   1];           %
q4_Table = [   1,   1,   1,   1,   1,   1,   1,   1,   1,   1];           %
q5_Table = [   1,   1,   1,   1,   1,   1,   1,   1,   1,   1];           %
q6_Table = [   1,   1,   1,   1,   1,   1,   1,   1,   1,   1];           %
r1_Table = [   1,   1,   1,   1,   1,   1,   1,   1,   1,   1]*50;        %
r2_Table = [   1,   1,   1,   1,   1,   1,   1,   1,   1,   1]*50;        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Matrices & Vectors %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%**** Initialize Continuous Matrices *************************************%
Matrix_Ac_4ws = zeros(6,6);             Matrix_Ad_4ws = zeros(6,6);
Matrix_Bc_4ws = zeros(6,2);             Matrix_Bd_4ws = zeros(6,2);
Matrix_Gc_4ws = zeros(6,1);             Matrix_Gd_4ws = zeros(6,1);
%**** Initialize Preview Matrices ****************************************%
Matrix_Ap_4ws = zeros(6+tP,6+tP);
Matrix_Bp_4ws = zeros(6+tP,   2);
%**** Initialize Weight Matrices *****************************************%
Matrix_Qp_4ws = zeros(6+tP,6+tP);
Matrix_Rp_4ws = zeros(2,2);
%**** Initialize Gain Matrices *******************************************%
Matrix_Kf_aug = zeros(10,6+tP);
Matrix_Kr_aug = zeros(10,6+tP);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate OPC Gain Table %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:10
    Vx = Vx_Table(i);
    q1 = q1_Table(i);
    q2 = q2_Table(i);
    q3 = q3_Table(i);
    q4 = q4_Table(i);
    q5 = q5_Table(i);
    q6 = q6_Table(i);
    r1 = r1_Table(i);
    r2 = r2_Table(i);
%**** Calculate Matrix Ac, Bc & Gc ***************************************%
    Matrix_Ac_4ws(1,2) = +1;
    Matrix_Ac_4ws(2,3) = +1;
    Matrix_Ac_4ws(3,3) = +2*(Cf+Cr)/(Ms*Vx);
    Matrix_Ac_4ws(3,5) = -2*(Cf+Cr)/(Ms);
    Matrix_Ac_4ws(3,6) = +2*(Cf*Lf-Cr*Lr)/(Ms*Vx);
    Matrix_Ac_4ws(4,5) = +1;
    Matrix_Ac_4ws(5,6) = +1;
    Matrix_Ac_4ws(6,3) = +2*(Cf*Lf-Cr*Lr)/(Iz*Vx);
    Matrix_Ac_4ws(6,5) = -2*(Cf*Lf-Cr*Lr)/(Iz);
    Matrix_Ac_4ws(6,6) = +2*(Cf*Lf^2+Cr*Lr^2)/(Iz*Vx);
    Matrix_Bc_4ws(3,1) = -2*(Cf)/(Ms);
    Matrix_Bc_4ws(3,2) = -2*(Cr)/(Ms);
    Matrix_Bc_4ws(6,1) = -2*(Cf*Lf)/(Iz);
    Matrix_Bc_4ws(6,2) = +2*(Cr*Lr)/(Iz);
    Matrix_Gc_4ws(3,1) = +2*(Cf*Lf-Cr*Lr)/(Ms)-Vx^2;
    Matrix_Gc_4ws(6,1) = +2*(Cf*Lf^2+Cr*Lr^2)/(Iz);
%**** Calculate Matrix Ad, Bd & Gd ***************************************%
    Matrix_Ad_4ws = eye(6) + tS*Matrix_Ac_4ws;
    Matrix_Bd_4ws =          tS*Matrix_Bc_4ws;
    Matrix_Gd_4ws =          tS*Matrix_Gc_4ws;
%**** Calculate Matrix Ap, Bp & Gp ***************************************%
    Matrix_Ap_4ws(1:6,1:6) = Matrix_Ad_4ws;
    Matrix_Ap_4ws(1:6,  7) = Matrix_Gd_4ws;
    Matrix_Ap_4ws(7:end-1,8:end) = eye(tP-1);
    Matrix_Bp_4ws(1:6,1:2) = Matrix_Bd_4ws;
%**** Calculate Matrix Ap, Bp & Gp ***************************************%
    Matrix_Qp_4ws(1,1) = q1;
    Matrix_Qp_4ws(2,2) = q2;
    Matrix_Qp_4ws(3,3) = q3;
    Matrix_Qp_4ws(4,4) = q4;
    Matrix_Qp_4ws(5,5) = q5;
    Matrix_Qp_4ws(6,6) = q6;
    Matrix_Rp_4ws(1,1) = r1;
    Matrix_Rp_4ws(2,2) = r2;
%**** Solve LQR **********************************************************%
    Lqr_Ans = dlqr(Matrix_Ap_4ws,Matrix_Bp_4ws,Matrix_Qp_4ws,Matrix_Rp_4ws);
    Matrix_Kf_aug(i,:) = Lqr_Ans(1,:);
    Matrix_Kr_aug(i,:) = Lqr_Ans(2,:);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Post Process %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Matrix_Kf_aug = [Vx_Table',Matrix_Kf_aug];
Matrix_Kr_aug = [Vx_Table',Matrix_Kr_aug];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%**** Set Figure *********************************************************%
subplot(2,2,1);
set(gca,"ZDir","normal");
subplot(2,2,2);
surf(tS*[1:1:tP],Vx_Table,Matrix_Kf_aug(1:10,8:end),'FaceAlpha',0.3);
set(gca,"ZDir","reverse");
subplot(2,2,3);
set(gca,"ZDir","reverse");
subplot(2,2,4);
surf(tS*[1:1:tP],Vx_Table,Matrix_Kr_aug(1:10,8:end),'FaceAlpha',0.3);
set(gca,"ZDir","reverse");
%**** Plot Feedback Gains ************************************************%
subplot(2,2,1);hold on; grid on;
plot3(1*ones(1,10),Vx_Table,Matrix_Kf_aug(:,2),...
    'Color',[0.5,0,0.5],'LineWidth',2.5,'Marker','o','MarkerSize',1.5);
plot3(2*ones(1,10),Vx_Table,Matrix_Kf_aug(:,3),...
    'Color',[0.6,0,0.6],'LineWidth',2.5,'Marker','o','MarkerSize',1.5);
plot3(3*ones(1,10),Vx_Table,Matrix_Kf_aug(:,4),...
    'Color',[0.7,0,0.7],'LineWidth',2.5,'Marker','o','MarkerSize',1.5);
plot3(4*ones(1,10),Vx_Table,Matrix_Kf_aug(:,5),...
    'Color',[0.8,0,0.8],'LineWidth',2.5,'Marker','o','MarkerSize',1.5);
plot3(5*ones(1,10),Vx_Table,Matrix_Kf_aug(:,6),...
    'Color',[0.9,0,0.9],'LineWidth',2.5,'Marker','o','MarkerSize',1.5);
plot3(6*ones(1,10),Vx_Table,Matrix_Kf_aug(:,7),...
    'Color',[1.0,0,1.0],'LineWidth',2.5,'Marker','o','MarkerSize',1.5);
for j = 1:1:9
    x_coordinates = 1*[1,1,1,1];
    y_coordinates = [Vx_Table(j),Vx_Table(j),Vx_Table(j+1),Vx_Table(j+1)];
    z_coordinates = [0,Matrix_Kf_aug(j,2),Matrix_Kf_aug(j+1,2),0];
    fill3(x_coordinates,y_coordinates,z_coordinates,[1,0,0.5],"FaceAlpha",0.1);
    x_coordinates = 2*[1,1,1,1];
    y_coordinates = [Vx_Table(j),Vx_Table(j),Vx_Table(j+1),Vx_Table(j+1)];
    z_coordinates = [0,Matrix_Kf_aug(j,3),Matrix_Kf_aug(j+1,3),0];
    fill3(x_coordinates,y_coordinates,z_coordinates,[1,0,0.5],"FaceAlpha",0.1);
    x_coordinates = 3*[1,1,1,1];
    y_coordinates = [Vx_Table(j),Vx_Table(j),Vx_Table(j+1),Vx_Table(j+1)];
    z_coordinates = [0,Matrix_Kf_aug(j,4),Matrix_Kf_aug(j+1,4),0];
    fill3(x_coordinates,y_coordinates,z_coordinates,[1,0,0.5],"FaceAlpha",0.1);
    x_coordinates = 4*[1,1,1,1];
    y_coordinates = [Vx_Table(j),Vx_Table(j),Vx_Table(j+1),Vx_Table(j+1)];
    z_coordinates = [0,Matrix_Kf_aug(j,5),Matrix_Kf_aug(j+1,5),0];
    fill3(x_coordinates,y_coordinates,z_coordinates,[1,0,0.5],"FaceAlpha",0.1);
    x_coordinates = 5*[1,1,1,1];
    y_coordinates = [Vx_Table(j),Vx_Table(j),Vx_Table(j+1),Vx_Table(j+1)];
    z_coordinates = [0,Matrix_Kf_aug(j,6),Matrix_Kf_aug(j+1,6),0];
    fill3(x_coordinates,y_coordinates,z_coordinates,[1,0,0.5],"FaceAlpha",0.1);
    x_coordinates = 6*[1,1,1,1];
    y_coordinates = [Vx_Table(j),Vx_Table(j),Vx_Table(j+1),Vx_Table(j+1)];
    z_coordinates = [0,Matrix_Kf_aug(j,7),Matrix_Kf_aug(j+1,7),0];
    fill3(x_coordinates,y_coordinates,z_coordinates,[1,0,0.5],"FaceAlpha",0.1);
end
subplot(2,2,3);hold on; grid on;
plot3(1*ones(1,10),Vx_Table,Matrix_Kr_aug(:,2),...
    'Color',[0.5,0,0.5],'LineWidth',2.5,'Marker','o','MarkerSize',1.5);
plot3(2*ones(1,10),Vx_Table,Matrix_Kr_aug(:,3),...
    'Color',[0.6,0,0.6],'LineWidth',2.5,'Marker','o','MarkerSize',1.5);
plot3(3*ones(1,10),Vx_Table,Matrix_Kr_aug(:,4),...
    'Color',[0.7,0,0.7],'LineWidth',2.5,'Marker','o','MarkerSize',1.5);
plot3(4*ones(1,10),Vx_Table,Matrix_Kr_aug(:,5),...
    'Color',[0.8,0,0.8],'LineWidth',2.5,'Marker','o','MarkerSize',1.5);
plot3(5*ones(1,10),Vx_Table,Matrix_Kr_aug(:,6),...
    'Color',[0.9,0,0.9],'LineWidth',2.5,'Marker','o','MarkerSize',1.5);
plot3(6*ones(1,10),Vx_Table,Matrix_Kr_aug(:,7),...
    'Color',[1.0,0,1.0],'LineWidth',2.5,'Marker','o','MarkerSize',1.5);
for j = 1:1:9
    x_coordinates = 1*[1,1,1,1];
    y_coordinates = [Vx_Table(j),Vx_Table(j),Vx_Table(j+1),Vx_Table(j+1)];
    z_coordinates = [0,Matrix_Kr_aug(j,2),Matrix_Kr_aug(j+1,2),0];
    fill3(x_coordinates,y_coordinates,z_coordinates,[1,0,0.5],"FaceAlpha",0.1);
    x_coordinates = 2*[1,1,1,1];
    y_coordinates = [Vx_Table(j),Vx_Table(j),Vx_Table(j+1),Vx_Table(j+1)];
    z_coordinates = [0,Matrix_Kr_aug(j,3),Matrix_Kr_aug(j+1,3),0];
    fill3(x_coordinates,y_coordinates,z_coordinates,[1,0,0.5],"FaceAlpha",0.1);
    x_coordinates = 3*[1,1,1,1];
    y_coordinates = [Vx_Table(j),Vx_Table(j),Vx_Table(j+1),Vx_Table(j+1)];
    z_coordinates = [0,Matrix_Kr_aug(j,4),Matrix_Kr_aug(j+1,4),0];
    fill3(x_coordinates,y_coordinates,z_coordinates,[1,0,0.5],"FaceAlpha",0.1);
    x_coordinates = 4*[1,1,1,1];
    y_coordinates = [Vx_Table(j),Vx_Table(j),Vx_Table(j+1),Vx_Table(j+1)];
    z_coordinates = [0,Matrix_Kr_aug(j,5),Matrix_Kr_aug(j+1,5),0];
    fill3(x_coordinates,y_coordinates,z_coordinates,[1,0,0.5],"FaceAlpha",0.1);
    x_coordinates = 5*[1,1,1,1];
    y_coordinates = [Vx_Table(j),Vx_Table(j),Vx_Table(j+1),Vx_Table(j+1)];
    z_coordinates = [0,Matrix_Kr_aug(j,6),Matrix_Kr_aug(j+1,6),0];
    fill3(x_coordinates,y_coordinates,z_coordinates,[1,0,0.5],"FaceAlpha",0.1);
    x_coordinates = 6*[1,1,1,1];
    y_coordinates = [Vx_Table(j),Vx_Table(j),Vx_Table(j+1),Vx_Table(j+1)];
    z_coordinates = [0,Matrix_Kr_aug(j,7),Matrix_Kr_aug(j+1,7),0];
    fill3(x_coordinates,y_coordinates,z_coordinates,[1,0,0.5],"FaceAlpha",0.1);
end
%**** Plot Preview Gains *************************************************%
for i = 1:1:10
    subplot(2,2,2);hold on;
    plot3(tS*[1:1:tP],ones(tP)*Vx_Table(i),Matrix_Kf_aug(i,8:end),...
        'Color',[0.1*i,0,0.6],'LineWidth',2.5,'Marker','o','MarkerSize',2.5);
    subplot(2,2,4);hold on;
    plot3(tS*[1:1:tP],ones(tP)*Vx_Table(i),Matrix_Kr_aug(i,8:end),...
        'Color',[0.1*i,0,0.6],'LineWidth',2.5,'Marker','o','MarkerSize',2.5);
end
%**** Set Font Etc. ******************************************************%
subplot(2,2,1); view(90,0); set(gca,"FontSize",20);
set(gca,"XLim",[1,6]); set(gca,"YLim",[2.5,25]);
set(gca,"XTickLabel",["k_1_f","k_2_f","k_3_f","k_4_f","k_5_f","k_6_f"]);
xlabel('反馈/[-]',FontSize=20);
ylabel('纵向车速/[m/s]',FontSize=20);
zlabel('反馈增益/[-]',FontSize=20);
legend("k_1_f","k_2_f","k_3_f","k_4_f","k_5_f","k_6_f");
subplot(2,2,2); view(30,30); set(gca,"FontSize",20);
xlabel('预瞄时间/[s]',FontSize=20);
ylabel('纵向车速/[m/s]',FontSize=20);
zlabel('预瞄增益/[-]',FontSize=20);
subplot(2,2,3); view(90,0); set(gca,"FontSize",20);
set(gca,"XLim",[1,6]); set(gca,"YLim",[2.5,25]);
set(gca,"XTickLabel",["k_1_r","k_2_r","k_3_r","k_4_r","k_5_r","k_6_r"]);
xlabel('反馈/[-]',FontSize=20);
ylabel('纵向车速/[m/s]',FontSize=20);
zlabel('反馈增益/[-]',FontSize=20);
legend("k_1_r","k_2_r","k_3_r","k_4_r","k_5_r","k_6_r");
subplot(2,2,4);view(30,30); set(gca,"FontSize",20);
xlabel('预瞄时间/[s]',FontSize=20);
ylabel('纵向车速/[m/s]',FontSize=20);
zlabel('预瞄增益/[-]',FontSize=20);
toc;