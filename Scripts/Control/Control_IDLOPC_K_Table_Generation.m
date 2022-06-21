%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hunan University
% Published by: Ang Li
% Email: ang1997@hnu.edu.cn
% Copyright (c) 2022 . All rights reserved.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; close all; clc;

% >>>>>>>>>>>>>>>>>>>> Controller Selection <<<<<<<<<<<<<<<<<<<< %

% >>>>>>>>>>>>>>>>>>>> Basic Settings <<<<<<<<<<<<<<<<<<<< %
Vx_table = [1.25,  2.5,  5.0,  7.5, 10.0, 12.5, 15.0, 17.5, 20.0, 22.5, 25.0, 27.5, 30.0, 32.5]';
Kv_table = [ 1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0]';
q1_table = [   1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1]';
q2_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
q3_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
q4_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
q5_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
q6_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
r1_table = [   1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1]';

coeff_k1 = 0.10;    % ErrY_Acc
coeff_k2 = 0.30;    % ErrY
coeff_k3 = 0.10;    % ErrVy
coeff_k4 = 0.25;    % ErrYz
coeff_k5 = 0.50;    % ErrAVz
coeff_k6 = 0.50;    % Lag Coeff

coeff_kd = 1.00;    % Delay Coeff
coeff_kp = 1.00;    % Preview Coeff

gains_matrix_front = zeros(length(Vx_table), 8);
gains_matrix_front(:,1) = 7870;
gains_matrix_front(:,2) = Vx_table;

% >>>>>>>>>>>>>>>>>>>> K-Table Generation <<<<<<<<<<<<<<<<<<<< %
for i = 1:1:length(Vx_table)
	Vx = Vx_table(i,:);
    Kv = Kv_table(i,:);
	
	q1 = q1_table(i,:);
	q2 = q2_table(i,:);
	q3 = q3_table(i,:);
	q4 = q4_table(i,:);
    q5 = q5_table(i,:);
    q6 = q6_table(i,:);
	r1 = r1_table(i,:);
	
	get_vehicle_2dof_preview_integer_delay_lag_model;
	
	Matrix_Q = diag([q1, q2, q3, q4, q5, q6, zeros(1,Td+Tp)]);
	Matrix_R = diag([r1]);        

    K = dlqr(Matrix_A, Matrix_B, Matrix_Q, Matrix_R);
    K_front = Kv*K(1,:);

	gains_matrix_front(i, 3:8+Td+Tp) = K_front;
    
%     gains_matrix(i, 3:Prev_Horizon+6) = -K_total;
%     gains_matrix(i, 3:6) = -gains_matrix(i, 3:6);
end

gains_matrix_front(:, 3) = coeff_k1*gains_matrix_front(:, 3);
gains_matrix_front(:, 4) = coeff_k2*gains_matrix_front(:, 4);
gains_matrix_front(:, 5) = coeff_k3*gains_matrix_front(:, 5);
gains_matrix_front(:, 6) = coeff_k4*gains_matrix_front(:, 6);
gains_matrix_front(:, 7) = coeff_k5*gains_matrix_front(:, 7);
gains_matrix_front(:, 8) = coeff_k6*gains_matrix_front(:, 8);

gains_matrix_front(:,9:8+Td)       = coeff_kd*gains_matrix_front(:,9:8+Td);
gains_matrix_front(:,9+Td:8+Td+Tp) = coeff_kp*gains_matrix_front(:,9+Td:8+Td+Tp);

for i = 1:1:length(Vx_table)
    subplot(1, 2, 1);
    plot(gains_matrix_front(i, 9:8+Td));
    hold on;

    subplot(1, 2, 2);
    plot(gains_matrix_front(i, 9+Td:8+Td+Tp));
    hold on;
end

%% Save Calibration File
save('Models\Vehicles\CiDi\Shanqi_E9\Conf\LQR\IDLOPC_K_table_front.mat', "gains_matrix_front");
writetable(array2table(gains_matrix_front), 'Models\Vehicles\CiDi\Shanqi_E9\Conf\LQR\IDLOPC_K_table_front.txt'); 