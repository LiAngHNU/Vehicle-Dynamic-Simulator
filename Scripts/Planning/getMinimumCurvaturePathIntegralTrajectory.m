%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Minimum Curvature Path Integral Planner %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Version: v2.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date: 2023/02/14 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Planner Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set MPPI-Parameters
Exp = +0.00;
Var = +1.00;
Lam = +0.01;
Ita =  0.00;
% Set Steps
dS =     5;
nP =    20;
nS =   500;
% Set Weights
Qc =  1.00;
Qd =  1-Qc;
% Set Constraints
dLmax = +0.75;
dLmin = -0.75;
% Set RefLine
load("Scenarios\RaceTracks\RaceTrack_Budapest.mat");
% Set Constraints
SafetyMargin = +1.50;
% Set Initial Posture
Vx =  0.00; Ax = 0.00;
Vy =  0.00; Ay = 0.00;
StartID =   501;
StartL  = -0.00;
StartX  = RefLineInfo.X(StartID) - StartL*sin(RefLineInfo.Tht(StartID));
StartY  = RefLineInfo.Y(StartID) + StartL*cos(RefLineInfo.Tht(StartID));
StartTht= RefLineInfo.Tht(StartID);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Vectors & Matrices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Vector_S_ref = [RefLineInfo.S(StartID):dS:RefLineInfo.S(StartID)+(nP-1)*dS]';
Vector_X_ref = interp1(RefLineInfo.S,RefLineInfo.X,Vector_S_ref);
Vector_Y_ref = interp1(RefLineInfo.S,RefLineInfo.Y,Vector_S_ref);
Vector_Tht_ref = interp1(RefLineInfo.S,RefLineInfo.Tht,Vector_S_ref);
% Initialize MPPI Matrices & Vectors
Matrix_L_smp = zeros(nP,nS);
Matrix_dL_smp = zeros(nP,nS);
Vector_dX_tmp = zeros(nP,1);
Vector_ddX_tmp = zeros(nP,1);
Vector_dY_tmp = zeros(nP,1);
Vector_ddY_tmp = zeros(nP,1);
Vector_J_cos = zeros(1,nS);
Vector_L_wei = zeros(1,nS);
Vector_dL_opt_last = zeros(1+nP,1);
Vector_L_opt = zeros(1+nP,1);
Vector_X_opt = zeros(1+nP,1);
Vector_Y_opt = zeros(1+nP,1);
% Initialize State Matrices & Vectors
Matrix_X_smp = zeros(nP,nS);
Matrix_Y_smp = zeros(nP,nS);
Matrix_K_smp = zeros(nP,nS);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate State Sequences %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic;
% Generate Control Sequences
for i = 1:1:nS
    for j = 1:1:nP
        Matrix_dL_smp(j,i) = Vector_dL_opt_last(j+1,1) + normrnd(Exp,Var);
        if Matrix_dL_smp(j,i) > dLmax
            Matrix_dL_smp(j,i) = dLmax;
        end
        if Matrix_dL_smp(j,i) < dLmin
            Matrix_dL_smp(j,i) = dLmin;
        end
    end
end
% Calculate State Sequences
for i = 1:1:nS
    for j = 1:1:nP
        if j == 1
            Matrix_L_smp(j,i) = StartL + Matrix_dL_smp(j,i);
        else
            Matrix_L_smp(j,i) = Matrix_L_smp(j-1,i) + Matrix_dL_smp(j,i);
        end
    end
end
for i = 1:1:nS
    Matrix_X_smp(:,i) = Vector_X_ref - Matrix_L_smp(:,i).*sin(Vector_Tht_ref);
    coefs_X = spline(Vector_S_ref,Matrix_X_smp(:,i)).coefs;
    Vector_dX_tmp(1,1) = coefs_X(1,3);
    Vector_ddX_tmp(1,1) = 2*coefs_X(1,2);
    for j = 2:1:nP
        Vector_dX_tmp(j,1) = 3*coefs_X(j-1,1)*dS*dS + 2*coefs_X(j-1,2)*dS + coefs_X(j-1,3);
        Vector_ddX_tmp(j,1) = 6*coefs_X(j-1,1)*dS + 2*coefs_X(j-1,2);
    end

    Matrix_Y_smp(:,i) = Vector_Y_ref + Matrix_L_smp(:,i).*cos(Vector_Tht_ref);
    coefs_Y = spline(Vector_S_ref,Matrix_Y_smp(:,i)).coefs;
    Vector_dY_tmp(1,1) = coefs_Y(1,3);
    Vector_ddY_tmp(1,1) = 2*coefs_Y(1,2);
    for j = 2:1:nP
        Vector_dY_tmp(j,1) = 3*coefs_Y(j-1,1)*dS*dS + 2*coefs_Y(j-1,2)*dS + coefs_Y(j-1,3);
        Vector_ddY_tmp(j,1) = 6*coefs_Y(j-1,1)*dS + 2*coefs_Y(j-1,2);
    end
    for j = 1:1:nP
        Matrix_K_smp(j,i) = (Vector_dX_tmp(j,1)*Vector_ddY_tmp(j,1) - Vector_ddX_tmp(j,1)*Vector_dY_tmp(j,1))/...
                            ((Vector_dX_tmp(j,1)*Vector_dX_tmp(j,1) + Vector_dY_tmp(j,1)*Vector_dY_tmp(j,1))^(3/2));
    end
end
% Calculate Cost
for i = 1:1:nS
    for j = 1:1:nP
        Vector_J_cos(1,i) = Vector_J_cos(1,i) + Matrix_K_smp(j,i)^2;
    end
end
% MPPI
minID = find(Vector_J_cos == min(Vector_J_cos));
minJP = Vector_J_cos(minID);
for i = 1:1:nS
    Ita = Ita + exp((minJP-Vector_J_cos(1,i))/Lam);
end
for i = 1:1:nS
    Vector_L_wei(1,i) = (1/Ita)*exp((minJP-Vector_J_cos(1,i))/Lam);
end
Vector_L_opt(1,1) = StartL;
Vector_X_opt(1,1) = StartX;
Vector_Y_opt(1,1) = StartY;
for i = 1:1:nS
    Vector_L_opt(2:end,1) = Vector_L_opt(2:end,1) + Vector_L_wei(1,i)*Matrix_L_smp(:,i);
end
Vector_X_opt(2:end,1) = Vector_X_ref - Vector_L_opt(2:end,1).*sin(Vector_Tht_ref);
Vector_Y_opt(2:end,1) = Vector_Y_ref + Vector_L_opt(2:end,1).*cos(Vector_Tht_ref);
toc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate State Sequences %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hold on; axis equal;
plot(RefLineInfo.X,RefLineInfo.Y,'k-');
plot(RefLineInfo.Xl,RefLineInfo.Yl,'r-');
plot(RefLineInfo.Xr,RefLineInfo.Yr,'r-');
for i = 1:1:nS
    plot(Matrix_X_smp(:,i),Matrix_Y_smp(:,i),'k:');
end
plot(Vector_X_opt,Vector_Y_opt,'g-','LineWidth',2);