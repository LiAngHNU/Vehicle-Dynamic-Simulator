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

Vx_table = [1.25,  2.5,  5.0,  7.5, 10.0, 12.5, 15.0, 17.5, 20.0, 22.5, 25.0, 27.5, 30.0  32.5]';
q1_table = [   1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1]';
q2_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
q3_table = [   5,    5,    5,    5,    5,    5,    5,    5,    5,    5,    5,    5,    5,    5]';
q4_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
q5_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
r1_table = [   2,    5,   10,   15,   75,  150,  300,  600,  750, 1000, 1500, 2250, 3500, 5000]';

gains_matrix_front = zeros(length(Vx_table), 7);
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
	r1 = r1_table(i,:);
	
	get_vehicle_2dof_lag_model;
	
	Matrix_Q = diag([q1, q2, q3, q4, q5]);
	Matrix_R = diag([r1]);        

    K = lqr(Matrix_A, Matrix_B, Matrix_Q, Matrix_R);
    K_front = K(1,:);

	gains_matrix_front(i, 3:7) = K_front;
    
%     gains_matrix(i, 3:Prev_Horizon+6) = -K_total;
%     gains_matrix(i, 3:6) = -gains_matrix(i, 3:6);

    save('Models\Vehicles\C_3A_Cab_Over_6x4\Conf\LQR\LCLQR_K_table_front.mat', "gains_matrix_front");
    writetable(array2table(gains_matrix_front), 'Models\Vehicles\C_3A_Cab_Over_6x4\Conf\LQR\LCLQR_K_table_front.txt'); 
end
