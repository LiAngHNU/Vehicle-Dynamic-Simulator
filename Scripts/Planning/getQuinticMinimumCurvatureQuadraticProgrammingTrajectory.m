%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Minimum Curvature Quadratic Programming Planner %%%%%%%%%%%%%%%%%%%%%%%%%
% Version: v2.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date: 2023/03/01 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc; tic;                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Planner Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%**** Set RefLine ********************************************************%
load('Scenarios\RaceTracks\RaceTrack_BrandsHatch.mat');                   %
%**** Set Steps **********************************************************%
dT =     1;     % Horizon Length of Terminal Heading                      %
dS =  5.00;     % Distance between Raw Path Points                        %
dC =    10;     % Horizon Length of Fixing Solution                       %
dP = floor(RefLineInfo.S(end)/dS);  % Horizon Length of Trajectory        %
%**** Set Weights ********************************************************%
MinKpa = 1e-20;                                                           %
%**** Set Constraints ****************************************************%
Lsafe = +0.00;  % Constraints of Safety Margin                            %
dLmax = +0.50;  % Constraints of Maximum Lateral Change Rate              %
dLmin = -0.50;  % Constraints of Minimum Lateral Change Rate              %
Vxmax = +50.0;
Aymax = +14.7;
% Set Initial & Final States                                              %
StartId =     1;                                                          %
StartS  = RefLineInfo.S(StartId);                                         %
StartL  = +2.00;                                                          %
StartX  = RefLineInfo.X(StartId) - StartL*sin(RefLineInfo.Tht(StartId));  %
StartY  = RefLineInfo.Y(StartId) + StartL*cos(RefLineInfo.Tht(StartId));  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Matrices & Vectors %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%**** Initialize RefLine Matrices ****************************************%
Vector_S_ref = zeros(dP,1);                                               %
Vector_X_ref = zeros(dP,1);                                               %
Vector_Y_ref = zeros(dP,1);                                               %
Vector_Tht_ref = zeros(dP,1);                                             %
Vector_Kpa_ref = zeros(dP,1);                                             %
Vector_dX_ref = zeros(dP,1);                                              %
Vector_dY_ref = zeros(dP,1);                                              %
Vector_Wl_ref = zeros(dP,1);                                              %
Vector_Wr_ref = zeros(dP,1);                                              %
Traj_S = zeros(dP+1,1);                                                   %
Traj_L = zeros(dP+1,1);                                                   %
Traj_X = zeros(dP+1,1);                                                   %
Traj_Y = zeros(dP+1,1);                                                   %
Traj_dL = zeros(dP,1);                                                    %
Traj_dX = zeros(dP,1);                                                    %
Traj_dY = zeros(dP,1);                                                    %
Traj_ddX = zeros(dP,1);                                                   %
Traj_ddY = zeros(dP,1);                                                   %
Traj_Tht = zeros(dP,1);                                                   %
Traj_Kpa = zeros(dP,1);                                                   %
Traj_Vxmax = zeros(dP+1,1);                                               %
%**** Initialize Median Matrices *****************************************%
Matrix_X_refsp = zeros(6*dP,1);     Matrix_Y_refsp = zeros(6*dP,1);       %
Matrix_A_spl = zeros(6*dP,6*dP);    Matrix_A_exb = zeros(dP,6*dP);        %
Matrix_A_trx = zeros(6*dP,dP);      Matrix_A_try = zeros(6*dP,dP);        %
Matrix_A_dsx = zeros(dP,dP);        Matrix_A_dsy = zeros(dP,dP);          %
Matrix_X_diff = zeros(dP,1);        Matrix_Y_diff = zeros(dP,1);          %
Matrix_A_rat = zeros(dP,dP);        Matrix_b_rat = zeros(dP,1);           %
%**** Initialize Cost Matrices (H & f) ***********************************%
Matrix_Pxx  = zeros(dP,dP);         Matrix_Qc   = zeros(dP,dP);           %
Matrix_Pxy  = zeros(dP,dP);         Matrix_Qd   = zeros(dP,dP);           %
Matrix_Pyy  = zeros(dP,dP);         Matrix_Qr   = zeros(dP,dP);           %
Matrix_Hcxx = zeros(dP,dP);         Matrix_fcxx = zeros(dP,1);            %
Matrix_Hcxy = zeros(dP,dP);         Matrix_fcxy = zeros(dP,1);            %
Matrix_Hcyy = zeros(dP,dP);         Matrix_fcyy = zeros(dP,1);            %
Matrix_Hc   = zeros(dP,dP);         Matrix_fc   = zeros(dP,1);            %
Matrix_Hd   = zeros(dP,dP);         Matrix_fd   = zeros(dP,1);            %
Matrix_Hr   = zeros(dP,dP);         Matrix_fr   = zeros(dP,1);            %
%**** Initialize Inequality Constraints Matrices (A & b) *****************%
Matrix_Asat = zeros(2*dP,dP);       Matrix_bsat = zeros(2*dP,1);          %
Matrix_Ainc = zeros(2*dP,dP);       Matrix_binc = zeros(2*dP,1);          %
Matrix_A    = zeros(4*dP,dP);       Matrix_b    = zeros(4*dP,1);          %
%**** Initialize Equality Constraints Matrices (Aeq & beq) ***************%
Matrix_Acon = zeros(dP,dP);         Matrix_bcon = zeros(dP,1);            %
Matrix_Aend = zeros(dP,dP);         Matrix_bend = zeros(dP,1);            %
Matrix_Adta = zeros(dP,dP);         Matrix_bdta = zeros(dP,1);            %
Matrix_Atht = zeros(dP,dP);         Matrix_btht = zeros(dP,1);            %
Matrix_Aeq  = zeros(dP,dP);         Matrix_beq  = zeros(dP,1);            %
%**** Initialize Bounds Matrices *****************************************%
Matrix_lb = [];                     Matrix_ub = [];                       %
%**** Initialize Initial Solution Matrices *******************************%
Matrix_x0 = StartL*ones(dP,1);                                            %
%**** Initialize Solution Matrices ***************************************%
Qp_ans = zeros(dP,1);                                                     %
Traj_S_Raw = zeros(dP+1,1);         Traj_L_Raw = zeros(dP+1,1);           %
Traj_X_Raw = zeros(dP+1,1);         Traj_Y_Raw = zeros(dP+1,1);           %
%**** Initialization Complete ********************************************%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate Reference Line Information %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:dP                                                            %
    Vector_S_ref(i,1) = RefLineInfo.S(StartId) + dS*i;                    %
    Vector_X_ref(i,1) = ...                                               %
        interp1(RefLineInfo.S,RefLineInfo.X,Vector_S_ref(i,1));           %
    Vector_Y_ref(i,1) = ...                                               %
        interp1(RefLineInfo.S,RefLineInfo.Y,Vector_S_ref(i,1));           %
    Vector_Tht_ref(i,1) = ...                                             %
        interp1(RefLineInfo.S,RefLineInfo.Tht,Vector_S_ref(i,1));         %
    Vector_Kpa_ref(i,1) = ...                                             %
        interp1(RefLineInfo.S,RefLineInfo.Kpa,Vector_S_ref(i,1));         %
    Vector_dX_ref(i,1) = ...                                              %
        interp1(RefLineInfo.S,RefLineInfo.dX,Vector_S_ref(i,1));          %
    Vector_dY_ref(i,1) = ...                                              %
        interp1(RefLineInfo.S,RefLineInfo.dY,Vector_S_ref(i,1));          %
    Vector_Wl_ref(i,1) = ...                                              %
        interp1(RefLineInfo.S,RefLineInfo.Wl,Vector_S_ref(i,1));          %
    Vector_Wr_ref(i,1) = ...                                              %
        interp1(RefLineInfo.S,RefLineInfo.Wr,Vector_S_ref(i,1));          %
end                                                                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate Matrices & Vectors %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%**** Calculate Xref & Yref **********************************************%
Matrix_X_refsp(1,1) = StartX;                                             %
Matrix_Y_refsp(1,1) = StartY;                                             %
Matrix_X_refsp(2,1) = Vector_X_ref(1,1);                                  %
Matrix_Y_refsp(2,1) = Vector_Y_ref(1,1);                                  %
Matrix_X_refsp(end-3,1) = 0;                                              %
Matrix_Y_refsp(end-3,1) = 0;                                              %
Matrix_X_refsp(end-2,1) = 0;                                              %
Matrix_Y_refsp(end-2,1) = 0;                                              %
Matrix_X_refsp(end-1,1) = 0;                                              %
Matrix_Y_refsp(end-1,1) = 0;                                              %
Matrix_X_refsp(end,  1) = 0;                                              %
Matrix_Y_refsp(end,  1) = 0;                                              %
for i = 2:1:dP                                                            %
    Matrix_X_refsp(6*i-5,1) = Vector_X_ref(i-1,1);                        %
    Matrix_X_refsp(6*i-4,1) = Vector_X_ref(i,1);                          %
    Matrix_Y_refsp(6*i-5,1) = Vector_Y_ref(i-1,1);                        %
    Matrix_Y_refsp(6*i-4,1) = Vector_Y_ref(i,1);                          %
end                                                                       %
%**** Calculate Aspl *****************************************************%
Matrix_A_spl(end-5:end-4,end-5:end) = [0,0,0,0,0,1;1,1,1,1,1,1;];         %
Matrix_A_spl(end-3,1:6) = [0,0,0,0,1,0];                                  %
Matrix_A_spl(end-2,1:6) = [0,0,0,2,0,0];                                  %
Matrix_A_spl(end-1,end-5:end) = [5,4,3,2,1,0];                            %
Matrix_A_spl(end  ,end-5:end) = [20,12,6,2,0,0];                          %
for i = 1:1:dP-1                                                          %
    Matrix_A_spl(6*i-5:6*i,6*i-5:6*i+6) = ...                             %
        [  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0;...              %
           1,  1,  1,  1,  1,  1,  0,  0,  0,  0,  0,  0;...              %
           5,  4,  3,  2,  1,  0,  0,  0,  0,  0, -1,  0;...              %
          20, 12,  6,  2,  0,  0,  0,  0,  0, -2,  0,  0;...              %
          60, 24,  6,  0,  0,  0,  0,  0, -6,  0,  0,  0;...              %
         120, 24,  0,  0,  0,  0,  0,-24,  0,  0,  0,  0;];               %
end                                                                       %
%**** Calculate Aexb *****************************************************%
for i = 1:1:dP                                                            %
    Matrix_A_exb(i,6*i-5:6*i) = [0,0,0,1,0,0];                            %
end                                                                       %
%**** Calculate Atrx & Atry **********************************************%
for i = 2:1:dP-1                                                          %
    Matrix_A_trx(6*i-5,i) = -sin(Vector_Tht_ref(i-1,1));                  %
    Matrix_A_trx(6*i-4,i+1) = -sin(Vector_Tht_ref(i,1));                  %
    Matrix_A_try(6*i-5,i) = +cos(Vector_Tht_ref(i-1,1));                  %
    Matrix_A_try(6*i-4,i+1) = +cos(Vector_Tht_ref(i,1));                  %
end                                                                       %
Matrix_A_trx(1,1) = 0;                                                    %
Matrix_A_trx(2,2) = -sin(Vector_Tht_ref(1,1));                            %
Matrix_A_trx(end-4,end) = -sin(Vector_Tht_ref(dP,1));                     %
Matrix_A_try(1,1) = 0;                                                    %
Matrix_A_try(2,2) = +cos(Vector_Tht_ref(1,1));                            %
Matrix_A_try(end-4,end) = +cos(Vector_Tht_ref(dP,1));                     %
% Calculate Adsx & Adsy                                                   %
Matrix_A_dsx(dP,dP) = -sin(Vector_Tht_ref(dP,1));                         %
Matrix_A_dsy(dP,dP) = +cos(Vector_Tht_ref(dP,1));                         %
for i = 1:1:dP-1                                                          %
    Matrix_A_dsx(i,i) = -sin(Vector_Tht_ref(i,1));                        %
    Matrix_A_dsx(i+1,i) = +sin(Vector_Tht_ref(i,1));                      %
    Matrix_A_dsy(i,i) = +cos(Vector_Tht_ref(i,1));                        %
    Matrix_A_dsy(i+1,i) = -cos(Vector_Tht_ref(i,1));                      %    
end                                                                       %
%**** Calculate Xdiff & Ydiff ********************************************%
Matrix_X_diff(1,1) = Vector_X_ref(1,1) - StartX;                          %
Matrix_Y_diff(1,1) = Vector_Y_ref(1,1) - StartY;                          %
for i = 2:1:dP                                                            %
    Matrix_X_diff(i,1) = Vector_X_ref(i,1) - Vector_X_ref(i-1,1);         %
    Matrix_Y_diff(i,1) = Vector_Y_ref(i,1) - Vector_Y_ref(i-1,1);         %
end                                                                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% QP Construction %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%**** Calculate Qc, Qd & Qr **********************************************%
for i = 1:1:dP                                                            %
    if abs(Vector_Kpa_ref(i,1)) > MinKpa                                  %
        Matrix_Qc(i,i) = 1.00;                                            %
        Matrix_Qd(i,i) = 0.00;                                            %
        Matrix_Qr(i,i) = 0.00;                                            %
    else                                                                  %
        Matrix_Qc(i,i) = 0.00;                                            %
        Matrix_Qd(i,i) = 1.00;                                            %
        Matrix_Qr(i,i) = 0.00;                                            %
    end                                                                   %
end                                                                       %
%**** Calculate Pxx, Pxy & Pyy *******************************************%
for i = 1:1:dP                                                            %
    Matrix_Pxx(i,i) = (Vector_dY_ref(i,1)^2) /...                         %
                      ((Vector_dX_ref(i,1)^2+Vector_dY_ref(i,1)^2)^3);    %
    Matrix_Pxy(i,i) = (-2*Vector_dX_ref(i,1)*Vector_dY_ref(i,1)) /...     %
                      ((Vector_dX_ref(i,1)^2+Vector_dY_ref(i,1)^2)^3);    %
    Matrix_Pyy(i,i) = (Vector_dX_ref(i,1)^2) /...                         %
                      ((Vector_dX_ref(i,1)^2+Vector_dY_ref(i,1)^2)^3);    %
end                                                                       %
Matrix_Pxx = Matrix_Pxx*Matrix_Qc;                                        %
Matrix_Pxy = Matrix_Pxy*Matrix_Qc;                                        %
Matrix_Pyy = Matrix_Pyy*Matrix_Qc;                                        %
%**** Calculate Matrix H *************************************************%
Matrix_Hcxx = ...                                                         %
    4*Matrix_A_trx'*transpose(inv(Matrix_A_spl))*Matrix_A_exb'...         %
     *Matrix_Pxx...                                                       %
     *Matrix_A_exb*inv(Matrix_A_spl)*Matrix_A_trx;                        %
Matrix_Hcxy = ...                                                         %
    4*Matrix_A_try'*transpose(inv(Matrix_A_spl))*Matrix_A_exb'...         %
     *Matrix_Pxy...                                                       %
     *Matrix_A_exb*inv(Matrix_A_spl)*Matrix_A_trx;                        %
Matrix_Hcyy = ...                                                         %
    4*Matrix_A_try'*transpose(inv(Matrix_A_spl))*Matrix_A_exb'...         %
     *Matrix_Pyy...                                                       %
     *Matrix_A_exb*inv(Matrix_A_spl)*Matrix_A_try;                        %
Matrix_Hc = Matrix_Hcxx + Matrix_Hcxy + Matrix_Hcyy;                      %
Matrix_Hd = Matrix_A_dsx'*Matrix_Qd*Matrix_A_dsx + ...                    %
            Matrix_A_dsy'*Matrix_Qd*Matrix_A_dsy;                         %
Matrix_Hr = Matrix_A_rat'*Matrix_Qr*Matrix_A_rat;                         %
Matrix_H = Matrix_Hc + Matrix_Hd + Matrix_Hr;                             %
%**** Calculate Matrix f *************************************************%
Matrix_fcxx = ...                                                         %
    4*Matrix_A_trx'*transpose(inv(Matrix_A_spl))*Matrix_A_exb'...         %
     *Matrix_Pxx*Matrix_A_exb*inv(Matrix_A_spl)*Matrix_X_refsp;           %
Matrix_fcxy = ...                                                         %
    4*Matrix_A_trx'*transpose(inv(Matrix_A_spl))*Matrix_A_exb'...         %
     *Matrix_Pxy*Matrix_A_exb*inv(Matrix_A_spl)*Matrix_Y_refsp+...        %
    4*Matrix_A_try'*transpose(inv(Matrix_A_spl))*Matrix_A_exb'...         %
     *Matrix_Pxy*Matrix_A_exb*inv(Matrix_A_spl)*Matrix_X_refsp;           %
Matrix_fcyy = ...                                                         %
    4*Matrix_A_try'*transpose(inv(Matrix_A_spl))*Matrix_A_exb'...         %
     *Matrix_Pyy*Matrix_A_exb*inv(Matrix_A_spl)*Matrix_Y_refsp;           %
Matrix_fc = Matrix_fcxx + Matrix_fcxy + Matrix_fcyy;                      %
Matrix_fd = Matrix_A_dsx'*Matrix_Qd*Matrix_X_diff + ...                   %
            Matrix_A_dsy'*Matrix_Qd*Matrix_Y_diff;                        %
Matrix_fr = Matrix_A_rat'*Matrix_Qr*Matrix_b_rat;                         %
Matrix_f = Matrix_fc + Matrix_fd + Matrix_fr;                             %
%**** Calculate Matrix A *************************************************%
Matrix_Asat = [+eye(dP);...                                               %
               -eye(dP);];                                                %
Matrix_Ainc = [+eye(dP);...                                               %
               -eye(dP);];                                                %
for i = 2:1:dP                                                            %
    Matrix_Ainc(i,i-1) = -1;                                              %
    Matrix_Ainc(i+dP,i-1) = +1;                                           %
end                                                                       %
Matrix_A = [Matrix_Asat;...                                               %
            Matrix_Ainc;];                                                %
%**** Calculate Matrix b *************************************************%
Matrix_bsat = [Vector_Wl_ref - Lsafe;...                                  %
               Vector_Wr_ref - Lsafe;];                                   %
Matrix_binc = [+dLmax*ones(dP,1);...                                      %
               -dLmin*ones(dP,1);];                                       %
Matrix_binc(1,1) = +dLmax + StartL;                                       %
Matrix_binc(dP+1,1) = -dLmin - StartL;                                    %
Matrix_b = [Matrix_bsat;...                                               %
            Matrix_binc;];                                                %
%**** Calculate Matrix Aeq ***********************************************%
Matrix_Acon(1:dC,1:dC) = eye(dC);                                         %
Matrix_Acon(end-dC+1:end,end-dC+1:end) = eye(dC);                         %
Matrix_Aend(end-dT+1:end,end-dT+1:end) = eye(dT);                         %
Matrix_Adta = eye(dP);                                                    %
for i = 2:1:dP                                                            %
    Matrix_Adta(i,i-1) = -1;                                              %
end                                                                       %
Matrix_Atht = Matrix_Aend*Matrix_Adta;                                    %
Matrix_Aeq = [Matrix_Acon;...                                             %
              Matrix_Atht;];                                              %
%**** Calculate Matrix beq ***********************************************%
Matrix_bcon(1:dC,1) = StartL;                                             %
Matrix_bcon(end-dC+1:end,1) = StartL;                                     %
Matrix_bdta(1,1) = -StartL;                                               %
Matrix_btht = Matrix_bend - Matrix_Aend*Matrix_bdta;                      %
Matrix_beq = [Matrix_bcon;...                                             %
              Matrix_btht;];                                              %
%**** Solve QP ***********************************************************%
options = optimoptions('quadprog','Algorithm','active-set');              %
Qp_ans = quadprog(Matrix_H,Matrix_f,...                                   %
                  Matrix_A,Matrix_b,...                                   %
                  Matrix_Aeq,Matrix_beq,...                               %
                  Matrix_ub,Matrix_lb,...                                 %
                  Matrix_x0,options);                                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Path Post Process %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%**** Calculate Path Coordinates *****************************************%
Traj_S = [StartS;Vector_S_ref;];                                          %
Traj_L = [StartL;Qp_ans];                                                 %
Traj_X = [StartX;Vector_X_ref-Qp_ans.*sin(Vector_Tht_ref)];               %
Traj_Y = [StartY;Vector_Y_ref+Qp_ans.*cos(Vector_Tht_ref)];               %
%**** Cubic Spline Interpolation *****************************************%
coefs_L = spline(Traj_S,Traj_L);
coefs_X = spline(Traj_S,Traj_X);
coefs_Y = spline(Traj_S,Traj_Y);
for i = 1:1:dP-1
    Traj_dL(i,1) = coefs_L.coefs(i,3);
    Traj_dX(i,1) = coefs_X.coefs(i,3);
    Traj_ddX(i,1) = 2*coefs_X.coefs(i,2);
    Traj_dY(i,1) = coefs_Y.coefs(i,3);
    Traj_ddY(i,1) = 2*coefs_Y.coefs(i,2);
end
Traj_dL(dP,1) = 3*coefs_L.coefs(dP-1,1)*(Traj_S(dP)-Traj_S(dP-1))^2 + ...
           2*coefs_L.coefs(dP-1,2)*(Traj_S(dP)-Traj_S(dP-1)) + ...
           1*coefs_L.coefs(dP-1,3);
Traj_dX(dP,1) = 3*coefs_X.coefs(dP-1,1)*(Traj_S(dP)-Traj_S(dP-1))^2 + ...
           2*coefs_X.coefs(dP-1,2)*(Traj_S(dP)-Traj_S(dP-1)) + ...
           1*coefs_X.coefs(dP-1,3);
Traj_ddX(dP,1) = 6*coefs_X.coefs(dP-1,1)*(Traj_S(dP)-Traj_S(dP-1)) + ...
            2*coefs_X.coefs(dP-1,2);
Traj_dY(dP,1) = 3*coefs_Y.coefs(dP-1,1)*(Traj_S(dP)-Traj_S(dP-1))^2 + ...
           2*coefs_Y.coefs(dP-1,2)*(Traj_S(dP)-Traj_S(dP-1)) + ...
           1*coefs_Y.coefs(dP-1,3);
Traj_ddY(dP,1) = 6*coefs_Y.coefs(dP-1,1)*(Traj_S(dP)-Traj_S(dP-1)) + ...
            2*coefs_Y.coefs(dP-1,2);
%**** Calculate Path Headings ********************************************%
Traj_Tht = Vector_Tht_ref + atan(Traj_dL/dS);
Traj_Tht = [RefLineInfo.Tht(StartId);Traj_Tht];
%**** Calculate Path Curvatures ******************************************%
for i = 1:1:dP
    Traj_Kpa(i,1) = (Traj_dX(i,1)*Traj_ddY(i,1)-Traj_ddX(i,1)*Traj_dY(i,1))/...
                    ((Traj_dX(i,1)^2+Traj_dY(i,1)^2)^(3/2));
end
Traj_Kpa = smooth([RefLineInfo.Kpa(StartId);Traj_Kpa],20);
%**** Exprot Trajectory Informations *************************************%
Trajectory = table(Traj_S,Traj_L,Traj_X,Traj_Y,Traj_Tht,Traj_Kpa);        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate Velocity Profile %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:dP+1
    if abs(Trajectory.Traj_Kpa(i)) > 0.01
        Traj_Vxmax(i,1) = sqrt(Aymax/abs(Trajectory.Traj_Kpa(i,1)));
    else
        Traj_Vxmax(i,1) = Vxmax;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
plot(Traj_X,Traj_Y,'b','LineWidth',2);
for i = 1:1:dP
    plot([Vector_Xl_ref(i,1),Vector_Xr_ref(i,1)],[Vector_Yl_ref(i,1),Vector_Yr_ref(i,1)],'k');
end
toc;