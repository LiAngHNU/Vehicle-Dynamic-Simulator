%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Minimum Curvature Quadratic Programming Planner %%%%%%%%%%%%%%%%%%%%%%%%%
% Version: v2.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date: 2023/02/14 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Planner Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set Steps
Tp =   200;
Ts =     5;
% Set Weights
Qc = 1.00;
Qd = 1-Qc;
Rd = 1.00;
% Set RefLine
load("Scenarios\RaceTracks\RaceTrack_Budapest.mat");
% Set Constraints
SafetyMargin = 0.00;
dLmax = +0.50;
dLmin = -0.50;
% Set Initial Posture
Vx =  0.00; Ax = 0.00;
Vy =  0.00; Ay = 0.00;
StartID =   501;
StartEy = -0.00;
StartX  = RefLineInfo.X(StartID) - StartEy*sin(RefLineInfo.Tht(StartID));
StartY  = RefLineInfo.Y(StartID) + StartEy*cos(RefLineInfo.Tht(StartID));
StartTht= RefLineInfo.Tht(StartID);
StartVx = Vx*cos(StartTht) - Vy*sin(StartTht);
StartVy = Vx*sin(StartTht) + Vy*cos(StartTht);
StartAx = Ax*cos(StartTht) - Ay*sin(StartTht);
StartAy = Ax*sin(StartTht) + Ay*cos(StartTht);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Matrices & Vectors %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize RefLine Matrices
Vector_S_ref = zeros(Tp,1);
Vector_X_ref = zeros(Tp,1);
Vector_Y_ref = zeros(Tp,1);
Vector_Tht_ref = zeros(Tp,1);
Vector_dX_ref = zeros(Tp,1);
Vector_dY_ref = zeros(Tp,1);
Vector_Wl_ref = zeros(Tp,1);
Vector_Wr_ref = zeros(Tp,1);
Vector_X_traj = zeros(Tp,1);
Vector_Y_traj = zeros(Tp,1);
% Initialize Matrices
Matrix_X_refsp = zeros(4*Tp,1);
Matrix_Y_refsp = zeros(4*Tp,1);
Matrix_X_diff = zeros(Tp,1);
Matrix_Y_diff = zeros(Tp,1);
Matrix_A_spl = zeros(4*Tp,4*Tp);
Matrix_A_exb = zeros(Tp,4*Tp);
Matrix_A_trx = zeros(4*Tp,Tp);
Matrix_A_try = zeros(4*Tp,Tp);
Matrix_A_dsx = zeros(Tp,Tp);
Matrix_A_dsy = zeros(Tp,Tp);
% Initialize QP Matrices
Matrix_Pxx = zeros(Tp,Tp);
Matrix_Pxy = zeros(Tp,Tp);
Matrix_Pyy = zeros(Tp,Tp);
Matrix_Hcxx = zeros(Tp,Tp);
Matrix_Hcxy = zeros(Tp,Tp);
Matrix_Hcyy = zeros(Tp,Tp);
Matrix_Hd   = zeros(Tp,Tp);
Matrix_fcxx = zeros(Tp,1);
Matrix_fcxy = zeros(Tp,1);
Matrix_fcyy = zeros(Tp,1);
Matrix_fd   = zeros(Tp,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate Reference Line Information %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic;
for i = 1:1:Tp
    Vector_S_ref(i,1) = RefLineInfo.S(StartID) + Ts*i;
    Vector_X_ref(i,1) = interp1(RefLineInfo.S,RefLineInfo.X,Vector_S_ref(i,1));
    Vector_Y_ref(i,1) = interp1(RefLineInfo.S,RefLineInfo.Y,Vector_S_ref(i,1));
    Vector_Tht_ref(i,1) = interp1(RefLineInfo.S,RefLineInfo.Tht,Vector_S_ref(i,1));
    Vector_dX_ref(i,1) = interp1(RefLineInfo.S,RefLineInfo.dX,Vector_S_ref(i,1));
    Vector_dY_ref(i,1) = interp1(RefLineInfo.S,RefLineInfo.dY,Vector_S_ref(i,1));
    Vector_Wl_ref(i,1) = interp1(RefLineInfo.S,RefLineInfo.Wl,Vector_S_ref(i,1));
    Vector_Wr_ref(i,1) = interp1(RefLineInfo.S,RefLineInfo.Wr,Vector_S_ref(i,1));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate Matrices & Vectors %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate Xref & Yref
Matrix_X_refsp(1,1) = StartX;
Matrix_X_refsp(2,1) = Vector_X_ref(1,1);
Matrix_X_refsp(end-1,1) = StartAx;
Matrix_X_refsp(end,  1) = interp1(RefLineInfo.S,RefLineInfo.dX,Vector_S_ref(Tp,1));
Matrix_Y_refsp(1,1) = StartY;
Matrix_Y_refsp(2,1) = Vector_Y_ref(1,1);
Matrix_Y_refsp(end-1,1) = StartAy;
Matrix_Y_refsp(end,  1) = interp1(RefLineInfo.S,RefLineInfo.dY,Vector_S_ref(Tp,1));
for i = 2:1:Tp
    Matrix_X_refsp(4*i-3,1) = Vector_X_ref(i-1,1);
    Matrix_X_refsp(4*i-2,1) = Vector_X_ref(i,1);
    Matrix_Y_refsp(4*i-3,1) = Vector_Y_ref(i-1,1);
    Matrix_Y_refsp(4*i-2,1) = Vector_Y_ref(i,1);
end
% Calculate Xdiff & Ydiff
Matrix_X_diff(1,1) = Vector_X_ref(1,1) - StartX;
Matrix_Y_diff(1,1) = Vector_Y_ref(1,1) - StartY;
for i = 2:1:Tp
    Matrix_X_diff(i,1) = Vector_X_ref(i,1) - Vector_X_ref(i-1,1);
    Matrix_Y_diff(i,1) = Vector_Y_ref(i,1) - Vector_Y_ref(i-1,1);
end
% Calculate Aspl
Matrix_A_spl(end-3:end-2,end-3:end) = [0,0,0,1;1,1,1,1;];
Matrix_A_spl(end-1,1:4) = [0,2,0,0];
Matrix_A_spl(end,end-3:end) = [3,2,1,0];
for i = 1:1:Tp-1
    Matrix_A_spl(4*i-3:4*i,4*i-3:4*i+4) = [ 0, 0, 0, 1, 0, 0, 0, 0;...
                                            1, 1, 1, 1, 0, 0, 0, 0;...
                                            3, 2, 1, 0, 0, 0,-1, 0;...
                                            6, 2, 0, 0, 0,-2, 0, 0;];
end
% Calculate Aexb
for i = 1:1:Tp
    Matrix_A_exb(i,4*i-3:4*i) = [0,1,0,0];
end
% Calculate Atrx & Atry
for i = 2:1:Tp-1
    Matrix_A_trx(4*i-3,i) = -sin(Vector_Tht_ref(i-1,1));
    Matrix_A_trx(4*i-2,i+1) = -sin(Vector_Tht_ref(i,1));
    Matrix_A_try(4*i-3,i) = +cos(Vector_Tht_ref(i-1,1));
    Matrix_A_try(4*i-2,i+1) = +cos(Vector_Tht_ref(i,1));
end
Matrix_A_trx(1,1) = 0;
Matrix_A_trx(2,2) = -sin(Vector_Tht_ref(1,1));
Matrix_A_trx(end-2,end) = -sin(Vector_Tht_ref(Tp,1));
Matrix_A_try(1,1) = 0;
Matrix_A_try(2,2) = +cos(Vector_Tht_ref(1,1));
Matrix_A_try(end-2,end) = +cos(Vector_Tht_ref(Tp,1));
% Calculate Pxx, Pxy & Pyy
for i = 1:1:Tp
    Matrix_Pxx(i,i) = (Vector_dY_ref(i,1)^2)/((Vector_dX_ref(i,1)^2+Vector_dY_ref(i,1)^2)^3);
    Matrix_Pxy(i,i) = (-2*Vector_dX_ref(i,1)*Vector_dY_ref(i,1))/((Vector_dX_ref(i,1)^2+Vector_dY_ref(i,1)^2)^3);
    Matrix_Pyy(i,i) = (Vector_dX_ref(i,1)^2)/((Vector_dX_ref(i,1)^2+Vector_dY_ref(i,1)^2)^3);
end
% Calculate Adsx & Adsy
Matrix_A_dsx(Tp,Tp) = -sin(Vector_Tht_ref(Tp,1));
Matrix_A_dsy(Tp,Tp) = +cos(Vector_Tht_ref(Tp,1));
for i = 1:1:Tp-1
    Matrix_A_dsx(i,i) = -sin(Vector_Tht_ref(i,1));
    Matrix_A_dsx(i+1,i) = +sin(Vector_Tht_ref(i,1));
    Matrix_A_dsy(i,i) = +cos(Vector_Tht_ref(i,1));
    Matrix_A_dsy(i+1,i) = -cos(Vector_Tht_ref(i,1));    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% QP Construction %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate Matrix H
Matrix_Hcxx = 4*Matrix_A_trx'*transpose(inv(Matrix_A_spl))*Matrix_A_exb'*Matrix_Pxx*Matrix_A_exb*inv(Matrix_A_spl)*Matrix_A_trx;
Matrix_Hcxy = 4*Matrix_A_try'*transpose(inv(Matrix_A_spl))*Matrix_A_exb'*Matrix_Pxy*Matrix_A_exb*inv(Matrix_A_spl)*Matrix_A_trx;
Matrix_Hcyy = 4*Matrix_A_try'*transpose(inv(Matrix_A_spl))*Matrix_A_exb'*Matrix_Pyy*Matrix_A_exb*inv(Matrix_A_spl)*Matrix_A_try;
Matrix_Hd = Matrix_A_dsx'*Matrix_A_dsx + Matrix_A_dsy'*Matrix_A_dsy;
Matrix_H = Qc*(Matrix_Hcxx + Matrix_Hcxy + Matrix_Hcyy) + Qd*Matrix_Hd;
% Calculate Matrix f
Matrix_fcxx = 4*Matrix_A_trx'*transpose(inv(Matrix_A_spl))*Matrix_A_exb'*Matrix_Pxx*Matrix_A_exb*inv(Matrix_A_spl)*Matrix_X_refsp;
Matrix_fcxy = 4*Matrix_A_trx'*transpose(inv(Matrix_A_spl))*Matrix_A_exb'*Matrix_Pxy*Matrix_A_exb*inv(Matrix_A_spl)*Matrix_Y_refsp + ...
             4*Matrix_A_try'*transpose(inv(Matrix_A_spl))*Matrix_A_exb'*Matrix_Pxy*Matrix_A_exb*inv(Matrix_A_spl)*Matrix_X_refsp;
Matrix_fcyy = 4*Matrix_A_try'*transpose(inv(Matrix_A_spl))*Matrix_A_exb'*Matrix_Pyy*Matrix_A_exb*inv(Matrix_A_spl)*Matrix_Y_refsp;
Matrix_fd = Matrix_A_dsx'*Matrix_X_diff + Matrix_A_dsy'*Matrix_Y_diff;
Matrix_f = Qc*(Matrix_fcxx + Matrix_fcxy + Matrix_fcyy) + Qd*Matrix_fd;
% Calculate Matrix A
Matrix_Asat = [+eye(Tp);...
               -eye(Tp);];
Matrix_Ainc = [+eye(Tp);...
               -eye(Tp);];
for i = 2:1:Tp
    Matrix_Ainc(i,i-1) = -1;
    Matrix_Ainc(i+Tp,i-1) = +1;
end
Matrix_A = [Matrix_Asat;...
            Matrix_Ainc;];
% Calculate Matrix b
Matrix_bsat = [Vector_Wl_ref - SafetyMargin;...
               Vector_Wr_ref - SafetyMargin;];
Matrix_binc = [+dLmax*ones(Tp,1);...
               -dLmin*ones(Tp,1);];
Matrix_binc(1,1) = +dLmax + StartEy;
Matrix_binc(Tp+1,1) = -dLmin - StartEy;
Matrix_b = [Matrix_bsat;...
            Matrix_binc;];
% Solve QP
Vector_L_traj = quadprog(Matrix_H, Matrix_f, Matrix_A, Matrix_b);
Vector_L_traj(1,1) = 0.5*(StartEy + Vector_L_traj(2,1));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Post Process %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Vector_X_traj = [StartX; Vector_X_ref - Vector_L_traj.*sin(Vector_Tht_ref)];
Vector_Y_traj = [StartY; Vector_Y_ref + Vector_L_traj.*cos(Vector_Tht_ref)]; toc;
Traj = [Vector_X_traj,Vector_Y_traj];
Vector_Xl_ref = interp1(RefLineInfo.S,RefLineInfo.Xl,Vector_S_ref);
Vector_Yl_ref = interp1(RefLineInfo.S,RefLineInfo.Yl,Vector_S_ref);
Vector_Xr_ref = interp1(RefLineInfo.S,RefLineInfo.Xr,Vector_S_ref);
Vector_Yr_ref = interp1(RefLineInfo.S,RefLineInfo.Yr,Vector_S_ref);
hold on; axis equal;
plot(RefLineInfo.X,RefLineInfo.Y,'r--');
plot(RefLineInfo.Xl,RefLineInfo.Yl,'k');
plot(RefLineInfo.Xr,RefLineInfo.Yr,'k');
plot([RefLineInfo.Xl(1,1),RefLineInfo.Xr(1,1)],...
     [RefLineInfo.Yl(1,1),RefLineInfo.Yr(1,1)],'k','LineWidth',10);
plot(Vector_X_traj,Vector_Y_traj,'g','LineWidth',2);
% for i = 1:1:Tp
%     plot([Vector_Xl_ref(i,1),Vector_Xr_ref(i,1)],[Vector_Yl_ref(i,1),Vector_Yr_ref(i,1)],'k');
% end