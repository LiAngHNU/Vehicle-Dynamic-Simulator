%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hunan University
% Published by: Ang Li
% Email: ang1997@hnu.edu.cn
% Copyright (c) 2022 . All rights reserved.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; close all; clc;

% >>>>>>>>>>>>>>>>>>>> Controller Selection <<<<<<<<<<<<<<<<<<<< %

% >>>>>>>>>>>>>>>>>>>> Basic Settings <<<<<<<<<<<<<<<<<<<< %
TS = 0.01;

Vx_table = [1.25,  2.5,  5.0,  7.5, 10.0, 12.5, 15.0, 17.5, 20.0, 22.5, 25.0, 27.5, 30.0, 32.5]';
q1_table = [   1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1]';
q2_table = [   1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1]';
q3_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
q4_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
q5_table = [   2,    5,   10,   15,   20,   30,   50,   80,  120,  200,    0,    0,    0,    0]';
q6_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
r1_table = [   1,    2,    5,   10,   16,   24,   36,   50,   80,  120,   50,   50,   50,   50]';

coeff_k1 = 1;
coeff_k2 = 1;
coeff_k3 = 1.5;
coeff_k4 = 0.3;
coeff_k5 = 3.0;
coeff_k6 = 1;

gains_matrix_front = zeros(length(Vx_table), 8);
gains_matrix_front(:,1) = 9070;
gains_matrix_front(:,2) = Vx_table;

% >>>>>>>>>>>>>>>>>>>> K-Table Generation <<<<<<<<<<<<<<<<<<<< %
for i = 1:1:length(Vx_table)
	Vx = Vx_table(i,:);
	
	q1 = q1_table(i,:);
	q2 = q2_table(i,:);
	q3 = q3_table(i,:);
	q4 = q4_table(i,:);
    q5 = q5_table(i,:);
    q6 = q6_table(i,:);
	r1 = r1_table(i,:);
	
	get_vehicle_2dof_integer_lag_model;
	
	Matrix_Q = diag([q1, q2, q3, q4, q5, q6]);
	Matrix_R = diag([r1]);        

    K = lqr(Matrix_A, Matrix_B, Matrix_Q, Matrix_R);
    K_front = K(1,:);

	gains_matrix_front(i, 3:8) = K_front;
    
%     gains_matrix(i, 3:Prev_Horizon+6) = -K_total;
%     gains_matrix(i, 3:6) = -gains_matrix(i, 3:6);
end

gains_matrix_front(:, 3) = coeff_k1*gains_matrix_front(:, 3);
gains_matrix_front(:, 4) = coeff_k2*gains_matrix_front(:, 4);
gains_matrix_front(:, 5) = coeff_k3*gains_matrix_front(:, 5);
gains_matrix_front(:, 6) = coeff_k4*gains_matrix_front(:, 6);
gains_matrix_front(:, 7) = coeff_k5*gains_matrix_front(:, 7);
gains_matrix_front(:, 8) = coeff_k6*gains_matrix_front(:, 8);

%% Save Calibration File
save('Models\Vehicles\C_3A_Cab_Over_6x4\Conf\LQR\ILCLQR_K_table_front.mat', "gains_matrix_front");
writetable(array2table(gains_matrix_front), 'Models\Vehicles\C_3A_Cab_Over_6x4\Conf\LQR\ILCLQR_K_table_front.txt'); 

