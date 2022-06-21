%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control System Analysis                                                 %
% Hunan University                                                        %
% Author: Ang Li                                                          %
% Email: ang1997@hnu.edu.cn                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control System Settings                                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ts = 0.01;      % Control Period:       0.01s
Tl = 0.2;       % First Order Coeff:    0.20s
Td = 20;        % Pure Delay Coeff:     0.20s
Tp = 100;       % Preview Window:       1.00s
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Basic Settings                                                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Vx_table = [1.38, 2.78, 5.56, 8.33,11.11,13.88,16.67,19.44,22.22,25.00,27.78]';

q1_table = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]';
q2_table = [   1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1]';
q3_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
q4_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
q5_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
q6_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
r1_table = [  24,   32,   56,   72,   88,  150,  240,  300,  500,  900, 1000]';

Gm_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
Pm_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
Wc_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
Os_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
Ts_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';

                     %                                      vx =        5.0;  10.0;  15.0;  20.0;  25.0;
coeff_ki = 2.500;    % Lat Error Integral Coeff             coeff_k1 =       0.250; 0.200; 0.200; 0.100;
coeff_k1 = 1.000;    % Lat Error Proportional Coeff         coeff_k2 =       0.175; 0.150; 0.150; 0.150;
coeff_k2 = 1.000;    % Lat Error Derivative Coeff           coeff_k3 =       0.000; 0.000; 0.000; 0.000;
coeff_k3 = 1.000;    % Heading Error Proportional Coeff     coeff_k4 =       0.700; 0.600; 0.600; 0.600;
coeff_k4 = 1.000;    % Heading Error Derivative Coeff       coeff_k5 =       0.000; 0.000; 0.000; 0.000;
coeff_kl = 1.250;    % Lag Coeff                            coeff_k6 =       0.850; 0.800; 0.800; 0.800;
coeff_kd = 1.250;    % Delay Coeff                          coeff_kd =       0.000; 0.000; 0.000; 0.000;
coeff_kp = 0.500;    % Preview Coeff                        coeff_kp =       0.140; 0.335; 0.400; 0.400;

gains_matrix_front = zeros(length(Vx_table), 8 + Td + Tp);
gains_matrix_front(:,1) = 10000;
gains_matrix_front(:,2) = Vx_table;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% K-Table Generation                                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:length(Vx_table)
	Vx = Vx_table(i,:);
	
	q1 = q1_table(i,:);
	q2 = q2_table(i,:);
	q3 = q3_table(i,:);
	q4 = q4_table(i,:);
    q5 = q5_table(i,:);
    q6 = q6_table(i,:);
	r1 = r1_table(i,:);
	
	get_lqr_based_vehicle_model;
	
	Matrix_Q = diag([q1, q2, q3, q4, q5, q6, zeros(1,Td + Tp)]);
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
    
    gains_matrix_front(i, 3:8+Td+Tp) = Matrix_K(1,:);
    
    Matrix_K_prev = Matrix_K(1, 7+Td:6+Td+Tp);
    Matrix_K(1, 1) = 0;
    Matrix_K(1, 7+Td:6+Td+Tp) = 0;
    Matrix_A_op = Matrix_A - Matrix_B*Matrix_K;
    Matrix_B_op = Matrix_A(:,7+Td:6+Td+Tp) - Matrix_B*Matrix_K_prev;
    Matrix_C_op = diag([1 1 1 1 1 zeros(1, 1+Td+Tp)]);
    Matrix_D_op = zeros(6+Td+Tp, Tp);
    
    sys = ss(Matrix_A_op, Matrix_B_op, Matrix_C_op, Matrix_D_op, Ts);
    
    hold on;
    margin(sys(2,1), {0.01,100});

    [Gm_table(i,1), Pm_table(i,1)] = margin(sys(2,1));
    [Gm_table(i,2), Pm_table(i,2)] = margin(sys(4,1));
end

% gains_matrix_front(:, 3) = coeff_ki*gains_matrix_front(:, 3);
% gains_matrix_front(:, 4) = coeff_k1*gains_matrix_front(:, 4);
% gains_matrix_front(:, 5) = coeff_k2*gains_matrix_front(:, 5);
% gains_matrix_front(:, 6) = coeff_k3*gains_matrix_front(:, 6);
% gains_matrix_front(:, 7) = coeff_k4*gains_matrix_front(:, 7);
% gains_matrix_front(:, 8) = coeff_kl*gains_matrix_front(:, 8);
% 
% gains_matrix_front(:,9:8+Td)       = coeff_kd*gains_matrix_front(:,9:8+Td);
% gains_matrix_front(:,9+Td:8+Td+Tp) = coeff_kp*gains_matrix_front(:,9+Td:8+Td+Tp);
figure;
pzmap(sys(2,1));

figure;
for i = 1:1:length(Vx_table)
    subplot(2, 2, 1);
    plot(gains_matrix_front(i, 3:8));
    hold on;

    subplot(2, 2, 2);
    plot(gains_matrix_front(i, 9:8+Td));
    hold on;

    subplot(2, 2, 3);
    plot(gains_matrix_front(i, 9+Td:8+Td+Tp));
    hold on;
end
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Save Calibration File                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
save('Models\Vehicles\CiDi\Shanqi_E9\Conf\LQR\LQR_Based_K_table_front.mat', "gains_matrix_front");
writetable(array2table(gains_matrix_front), 'Models\Vehicles\CiDi\Shanqi_E9\Conf\LQR\LQR_Based_K_table_front.txt');