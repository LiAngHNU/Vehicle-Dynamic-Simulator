%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LQR K-Table Generator %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Version: v1.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Date: 2022/11/15 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Controller Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ts = 0.01;              % Control Period:       0.01s
Tl = 0.2;               % First Order Coeff:    0.20s
Td = 20;                % Pure Delay Coeff:     0.20s
Tp = 200;               % Preview Window:       2.00s
Tr = 100;               % Preview Window:       1.00s
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Weight Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Vx_tab = [1.38, 2.78, 5.56, 8.33,11.11,13.88,16.67,19.44,22.22,25.00,27.78]';

q1_tab = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]';
q2_tab = [   1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1]';
q3_tab = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
q4_tab = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
q5_tab = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
q6_tab = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
r1_tab = [  24,   32,   56,   72,   88,  150,  240,  300,  500,  900, 1000]';

Gm_tab = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
Pm_tab = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
Wc_tab = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
Os_tab = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
Ts_tab = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';

coeff_ki = 1.250;    % Lat Error Integral Coeff
coeff_k1 = 1.000;    % Lat Error Proportional Coeff
coeff_k2 = 1.000;    % Lat Error Derivative Coeff
coeff_k3 = 1.000;    % Heading Error Proportional Coeff
coeff_k4 = 1.000;    % Heading Error Derivative Coeff
coeff_kl = 1.350;    % Lag Coeff
coeff_kd = 1.350;    % Delay Coeff
coeff_kp = 0.700;    % Preview Coeff
coeff_kr = 400.0;    % Preview Coeff

gains_matrix_front = zeros(length(Vx_tab), 8 + Td + Tp + Tr);
gains_matrix_front(:,1) = 4580;
gains_matrix_front(:,2) = Vx_tab;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Generate K Table %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:length(Vx_tab)
	Vx = Vx_tab(i,:);
	
	q1 = q1_tab(i,:);
	q2 = q2_tab(i,:);
	q3 = q3_tab(i,:);
	q4 = q4_tab(i,:);
    q5 = q5_tab(i,:);
    q6 = q6_tab(i,:);
	r1 = r1_tab(i,:);
	
	getLqrBasedModel;
	
	Matrix_Q = diag([q1, q2, q3, q4, q5, q6, zeros(1,Td + Tp + Tr)]);
	Matrix_R = diag([r1]);        

    Matrix_K = dlqr(Matrix_A, Matrix_B, Matrix_Q, Matrix_R);
    Matrix_K(1, 1) = coeff_ki*Matrix_K(1, 1);
    Matrix_K(1, 2) = coeff_k1*Matrix_K(1, 2);
    Matrix_K(1, 3) = coeff_k2*Matrix_K(1, 3);
    Matrix_K(1, 4) = coeff_k3*Matrix_K(1, 4);
    Matrix_K(1, 5) = coeff_k4*Matrix_K(1, 5);
    Matrix_K(1, 6) = coeff_kl*Matrix_K(1, 6);
    Matrix_K(1, 7:6+Td) = coeff_kd*Matrix_K(1, 7:6 + Td);
    Matrix_K(1, 7+Td:6+Td+Tp) = coeff_kp*Matrix_K(1, 7+Td:6+Td+Tp);
    Matrix_K(1, 7+Td+Tp:6+Td+Tp+Tr) = coeff_kr*Matrix_K(1, 7+Td+Tp:6+Td+Tp+Tr);
    
    gains_matrix_front(i, 3:8+Td+Tp+Tr) = Matrix_K(1,:);
    
    Matrix_K_prev = Matrix_K(1, 7+Td:6+Td+Tp);
    Matrix_K(1, 1) = 0;
    Matrix_K(1, 7+Td:6+Td+Tp) = 0;
    Matrix_A_op = Matrix_A - Matrix_B*Matrix_K;
    Matrix_B_op = Matrix_A(:,7+Td:6+Td+Tp) - Matrix_B*Matrix_K_prev;
    Matrix_C_op = diag([1 1 1 1 1 zeros(1, 1+Td+Tp+Tr)]);
    Matrix_D_op = zeros(6+Td+Tp+Tr, Tp);
    
    sys = ss(Matrix_A_op, Matrix_B_op, Matrix_C_op, Matrix_D_op, Ts);
    
    hold on;
    margin(sys(2,1), {0.01,100});

    [Gm_tab(i,1), Pm_tab(i,1)] = margin(sys(2,1));
    [Gm_tab(i,2), Pm_tab(i,2)] = margin(sys(4,1));
end

figure;
pzmap(sys(2,1));

figure;
for i = 1:1:length(Vx_tab)
    subplot(2, 2, 1);
    plot(gains_matrix_front(i, 3:8));
    hold on;

    subplot(2, 2, 2);
    plot(gains_matrix_front(i, 9:8+Td));
    hold on;

    subplot(2, 2, 3);
    plot(gains_matrix_front(i, 9+Td:8+Td+Tp));
    hold on;

    subplot(2, 2, 4);
    plot(gains_matrix_front(i, 9+Td+Tp:8+Td+Tp+Tr));
    hold on;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Save K Table %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% save('Models\Vehicles\VehicleSim\CommercialVehicles\2A_Micro_Truck\Conf\LQR\idlopc_k_table.mat', "gains_matrix_front");
% writetable(array2table(gains_matrix_front), 'Models\Vehicles\VehicleSim\CommercialVehicles\2A_Micro_Truck\Conf\LQR\idlopc_k_table.txt');
save('Models\Vehicles\CiDi\JMC_N800\Conf\LQR\idlopc_k_table_rear.mat', "gains_matrix_front");
writetable(array2table(gains_matrix_front), 'Models\Vehicles\CiDi\JMC_N800\Conf\LQR\idlopc_k_table_rear.txt');