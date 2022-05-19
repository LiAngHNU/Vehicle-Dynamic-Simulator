%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hunan University
% Published by: Ang Li
% Email: ang1997@hnu.edu.cn
% Copyright (c) 2022 . All rights reserved.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; close all; clc;

% >>>>>>>>>>>>>>>>>>>> Controller Selection <<<<<<<<<<<<<<<<<<<< %
FLAGS_use_Normal_LQR      = true;
FLAGS_use_Integral_LQR    = false;

% >>>>>>>>>>>>>>>>>>>> Basic Settings <<<<<<<<<<<<<<<<<<<< %
TS = 0.01;

Vx_table = [1.25,  2.5,  5.0,  7.5, 10.0, 12.5, 15.0, 17.5, 20.0, 22.5, 25.0, 27.5, 30.0  32.5]';
q1_table = [   1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1]';
q2_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
q3_table = [  20,   20,   15,   10,   10,   10,   10,   10,   10,   10,   10,   10,   10,   10]';
q4_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]';
r1_table = [  15,   20,   30,   45,   65,   90,  120,  155,  195,  240,  290,  345,  405,  500]';
r2_table = [  15,   20,   25,   30,   35,   50,  100,  400,  800, 1200, 1000, 1500, 2500, 3500]';

gains_matrix_front = zeros(length(Vx_table), 6);
gains_matrix_rear  = zeros(length(Vx_table), 6);
gains_matrix_front(:,1) = 9070;
gains_matrix_rear(:,1)  = 9070;
gains_matrix_front(:,2) = Vx_table;
gains_matrix_rear(:,2)  = Vx_table;

% >>>>>>>>>>>>>>>>>>>> K-Table Generation <<<<<<<<<<<<<<<<<<<< %
for i = 1:1:length(Vx_table)
	Vx = Vx_table(i,:);
	
	q1 = q1_table(i,:);
	q2 = q2_table(i,:);
	q3 = q3_table(i,:);
	q4 = q4_table(i,:);
	r1 = r1_table(i,:);
    r2 = r2_table(i,:);
	
	get_vehicle_2dof_model;
	
	Matrix_Q = diag([q1, q2, q3, q4]);
	Matrix_R = diag([r1, r2]);        

    K = lqr(Matrix_A, Matrix_B, Matrix_Q, Matrix_R);
    K_front = K(1,:);
    K_rear  = K(2,:);

	gains_matrix_front(i, 3:6) = K_front;
    gains_matrix_rear(i, 3:6)  = K_rear;
    
%     gains_matrix(i, 3:Prev_Horizon+6) = -K_total;
%     gains_matrix(i, 3:6) = -gains_matrix(i, 3:6);
end

save("Models\Vehicles\C_3A_Cab_Over_6x4\Conf\LQR\LQR_K_table_front.mat", "gains_matrix_front");
save("Models\Vehicles\C_3A_Cab_Over_6x4\Conf\LQR\LQR_K_table_rear.mat", "gains_matrix_rear");
writetable(array2table(gains_matrix_front), 'Models\Vehicles\C_3A_Cab_Over_6x4\Conf\LQR\LQR_K_table_front.txt');
writetable(array2table(gains_matrix_rear),  'Models\Vehicles\C_3A_Cab_Over_6x4\Conf\LQR\LQR_K_table_rear.txt');     
