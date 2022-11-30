%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hunan University
% Published by: Ang Li
% Email: ang1997@hnu.edu.cn
% Copyright (c) 2022 . All rights reserved.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Numerical Model
% Lag/Delay/Preview Settings
% Ts = 0.01;      % Control Period:       0.01s
% Tl = 0.2;       % First Order Coeff:    0.20s
% Td = 20;        % Pure Delay Coeff:     0.20s
% Tp = 300;       % Preview Window:       1.00s

% Load Vehicle Infomation
load('Models\Vehicles\CiDi\Shanqi_E9\Shanqi_E9_Calib.mat');
% load('Models\Vehicles\VehicleSim\CommercialVehicles\2A_Micro_Truck\Micro_Truck_Calib.mat');
M   = Vehicle_Calib.Mass;
Iz  = Vehicle_Calib.Inertia;
Lf  = Vehicle_Calib.Lf;
Lr  = Vehicle_Calib.Lr;
Lp  = Vehicle_Calib.Lf;
Cyf = Vehicle_Calib.Cyf;    
Cyr = Vehicle_Calib.Cyr;    

% Calculate Numerical LQR-Based Vehicle Model
[Matrix_A, Matrix_B, Matrix_G] = get_idlopc_rear_axle_model(M, Iz, Lf, Lr, Lp, Cyf, Cyr, Vx, Ts, Tl, Td, Tp, Tr);

% Calculate Symbolic LQR_Based Vehicle Model

% Vehicle Models
function [Matrix_A, Matrix_B, Matrix_G] = ...
    get_idlopc_rear_axle_model(M, Iz, Lf, Lr, Lp, Cyf, Cyr, Vx, Ts, Tl, Td, Tp, Tr)

    a22 =  2*(Cyf+Cyr)/(M*Vx) + 2*(Cyf*Lf*Lp - Cyr*Lr*Lp)/(Iz*Vx);
    a23 = -2*(Cyf+Cyr)/(M)    - 2*(Cyf*Lf*Lp - Cyr*Lr*Lp)/(Iz);
    a24 =  2*(Cyf*(Lf - Lp)       - Cyr*(Lr + Lp))/(M*Vx) + ...
           2*(Cyf*Lf*Lp*(Lf - Lp) + Cyr*Lr*Lp*(Lr + Lp))/(Iz*Vx);
    a25 = -2*Cyf/M -2*Cyf*Lf*Lp/Iz;

    a42 =  2*(Cyf*Lf-Cyr*Lr)/(Iz*Vx);
    a43 = -2*(Cyf*Lf-Cyr*Lr)/Iz;
    a44 =  2*(Cyf*Lf*(Lf - Lp)+Cyr*Lr*(Lr + Lp))/(Iz*Vx);
    a45 = -2*Cyf*Lf/Iz;

    a55 = -1/Tl;        b51 = 1/Tl;

    g21 =  2*(Cyf*(Lf - Lp) - Cyr*(Lr + Lp))/M +...
           2*(Cyf*Lf*Lp*(Lf - Lp) + Cyr*Lr*Lp*(Lr + Lp))/Iz - Vx*Vx ;
    g41 =  2*(Cyf*Lf*(Lf - Lp)+Cyr*Lr*(Lr + Lp))/Iz;

    Matrix_Ac = [0,  1,  0,  0,  0,  0;...
                 0,  0,  1,  0,  0,  0;...
                 0,  0,a22,a23,a24,a25;...
                 0,  0,  0,  0,  1,  0;...
                 0,  0,a42,a43,a44,a45;...
                 0,  0,  0,  0,  0,a55;];

    Matrix_Bc = [  0;...
                   0;...
                   0;...
                   0;...
                   0;...
                 b51;];  

    Matrix_Gc = [  0;
                   0;...
                 g21;...
                   0;...
                 g41;...
                   0;]; 

    Matrix_Fc = [  0;
                   0;...
                   0;...
                   0;...
                 -Vx;...
                   0;]; 

    Matrix_Ad = eye(6) + Ts*Matrix_Ac;
    Matrix_Bd =          Ts*Matrix_Bc;
    Matrix_Gd =          Ts*Matrix_Gc;
    Matrix_Fd =          Ts*Matrix_Fc;

    Matrix_Delay = [zeros(Td-1, 1),      eye(Td-1);...
                                 0, zeros(1, Td-1);];
    Matrix_Prev  = [zeros(Tp-1, 1),      eye(Tp-1);...
                                 0, zeros(1, Tp-1);];
    Matrix_Rate  = [zeros(Tr-1, 1),      eye(Tr-1);...
                                 0, zeros(1, Tr-1);]; 

    Matrix_A = zeros(6+Td+Tp+Tr, 6+Td+Tp+Tr);
    Matrix_B = zeros(6+Td+Tp+Tr,          1);
    Matrix_G = zeros(6+Td+Tp+Tr,          1);

    Matrix_A(1:6, 1:6)                               = Matrix_Ad;
    Matrix_A(1:6, 7)                                 = Matrix_Bd;
    Matrix_A(1:6, 7+Td)                              = Matrix_Gd;
    Matrix_A(1:6, 7+Td+Tp)                           = Matrix_Fd;
    Matrix_A(7:6+Td, 7:6+Td)                         = Matrix_Delay;
    Matrix_A(7+Td:6+Td+Tp, 7+Td:6+Td+Tp)             = Matrix_Prev;
    Matrix_A(7+Td+Tp:6+Td+Tp+Tr, 7+Td+Tp:6+Td+Tp+Tr) = Matrix_Rate;

    Matrix_B(6+Td, 1) = 1;
    Matrix_G(6+Td+Tp, 1) = 1;
end

function [Matrix_A, Matrix_B, Matrix_G] = get_lqr_model(M, Iz, Lf, Lr, Cyf, Cyr, Vx)
    a22 =  2*(Cyf+Cyr)/(M*Vx);          a42 =  2*(Cyf*Lf-Cyr*Lr)/(Iz*Vx);
    a23 = -2*(Cyf+Cyr)/M;               a43 = -2*(Cyf*Lf-Cyr*Lr)/Iz;    
    a24 =  2*(Cyf*Lf-Cyr*Lr)/(M*Vx);    a44 =  2*(Cyf*Lf*Lf+Cyr*Lr*Lr)/(Iz*Vx);   

    b21 = -2*Cyf/M;                     b41 = -2*Cyf*Lf/Iz;

    g21 = 2*(Cyf*Lf-Cyr*Lr)/M - Vx^2;   g41 = 2*(Cyf*Lf*Lf+Cyr*Lr*Lr)/Iz;

    Matrix_A = [0,  1,  0,  0;...
                0,a22,a23,a24;...
                0,  0,  0,  1;...
                0,a42,a43,a44;];
    Matrix_B = [  0;...
                b21;...
                  0;...
                b41;]; 
    Matrix_G = [  0;...
                g21;...
                  0;...
                g41;]; 
end

function [Matrix_A, Matrix_B, Matrix_G] = get_llqr_model(M, Iz, Lf, Lr, Cyf, Cyr, Vx, Tl)
    a22 =  2*(Cyf+Cyr)/(M*Vx);          a42 =  2*(Cyf*Lf-Cyr*Lr)/(Iz*Vx);
    a23 = -2*(Cyf+Cyr)/M;               a43 = -2*(Cyf*Lf-Cyr*Lr)/Iz;    
    a24 =  2*(Cyf*Lf-Cyr*Lr)/(M*Vx);    a44 =  2*(Cyf*Lf*Lf+Cyr*Lr*Lr)/(Iz*Vx);   
    a25 = -2*Cyf/M;                     a45 = -2*Cyf*Lf/Iz;
                                        a55 = -1/Tl;

    b51 = 1/Tl;

    g21 = 2*(Cyf*Lf-Cyr*Lr)/M - Vx^2;   g41 = 2*(Cyf*Lf*Lf+Cyr*Lr*Lr)/Iz;

    Matrix_A = [0,  1,  0,  0,  0;...
                0,a22,a23,a24,a25;...
                0,  0,  0,  1,  0;...
                0,a42,a43,a44,a45;...
                0,  0,  0,  0,a55;];
    Matrix_B = [  0;...
                0;...
                0;...
                0;...
              b51;];  
    Matrix_G = [  0;...
                g21;...
                  0;...
                g41;...
                  0;]; 
end

function [Matrix_A, Matrix_B, Matrix_G] = get_illqr_model(M, Iz, Lf, Lr, Cyf, Cyr, Vx, Tl)
    a22 =  2*(Cyf+Cyr)/(M*Vx);          a42 =  2*(Cyf*Lf-Cyr*Lr)/(Iz*Vx);
    a23 = -2*(Cyf+Cyr)/M;               a43 = -2*(Cyf*Lf-Cyr*Lr)/Iz;    
    a24 =  2*(Cyf*Lf-Cyr*Lr)/(M*Vx);    a44 =  2*(Cyf*Lf*Lf+Cyr*Lr*Lr)/(Iz*Vx);   
    a25 = -2*Cyf/M;                     a45 = -2*Cyf*Lf/Iz;
                                        a55 = -1/Tl;

    b51 = 1/Tl;

    g21 = 2*(Cyf*Lf-Cyr*Lr)/M - Vx^2;   g41 = 2*(Cyf*Lf*Lf+Cyr*Lr*Lr)/Iz;

    Matrix_A = [0,  1,  0,  0,  0,  0;...
                0,  0,  1,  0,  0,  0;...
                0,  0,a22,a23,a24,a25;...
                0,  0,  0,  0,  1,  0;...
                0,  0,a42,a43,a44,a45;...
                0,  0,  0,  0,  0,a55;];
    Matrix_B = [  0;...
                  0;...
                  0;...
                  0;...
                  0;...
                b51;];  
    Matrix_G = [  0;
                  0;...
                g21;...
                  0;...
                g41;...
                  0;]; 
end

function [Matrix_A, Matrix_B, Matrix_G] = get_idllqr_model(M, Iz, Lf, Lr, Cyf, Cyr, Vx, Ts, Tl, Td)
    [Matrix_A, Matrix_B, Matrix_G] = get_illqr_model(M, Iz, Lf, Lr, Cyf, Cyr, Vx, Tl);
    
    Matrix_A = eye(6) + Ts*Matrix_A;
    Matrix_B =          Ts*Matrix_B;
    Matrix_G =          Ts*Matrix_G;

    Matrix_Delay = [zeros(Td-1,1),      eye(Td-1);...
                                0,  zeros(1,Td-1);];

    Matrix_A = [Matrix_A, Matrix_B, zeros(6,Td-1);...
                zeros(Td,6), Matrix_Delay;];
    Matrix_B = [zeros(5+Td,1);1];
    Matrix_G = [Matrix_G;zeros(Td,1)];
end

function [Matrix_A, Matrix_B, Matrix_G] = get_idlopc_model(M, Iz, Lf, Lr, Cyf, Cyr, Vx, Ts, Tl, Td, Tp)
    [Matrix_A, Matrix_B, Matrix_G] = get_idllqr_model(M, Iz, Lf, Lr, Cyf, Cyr, Vx, Ts, Tl, Td);
    
    Matrix_Preview = [zeros(Tp-1,1),      eye(Tp-1);...
                                  0,  zeros(1,Tp-1);];

    Matrix_A = [Matrix_A,[Matrix_G, zeros(6+Td,Tp-1)];...
                zeros(Tp,6+Td), Matrix_Preview];
    Matrix_B = [Matrix_B;zeros(Tp,1)];
    Matrix_G = [zeros(5+Td+Tp,1);1];
end