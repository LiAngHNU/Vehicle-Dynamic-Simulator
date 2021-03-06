%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Product Department 1
% Published by: Ang Li
% Email: li.ang@cidi.ai
% Copyright (c) 2022 CIDI. All rights reserved.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; close all; clc;

% >>>>>>>>>>>>>>>>>>>> Basic Settings <<<<<<<<<<<<<<<<<<<< %

TS = 0.01;

VX_table =([   5,   10,   20,   30,   40,   50,   60,   70,   80]/3.6)';
q1_table = [   1,    1,    1,    1,    1,    1,    1,    1,    1]';
q2_table = [ 1.1,  1.4,  1.9,  2.8,  3.8,  3.9,  4.5,  5.2,  6.0]';
q3_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0]';
q4_table = [ 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0]';
 r_table = [  13,   14,   23,   23,   75,   95,  300,  300,  300]';

Prev_Window_table = ...
		   [ 5.0,  5.0,  5.0,  5.0,  5.0,  5.0,  5.0,  5.0,  5.0]';
	 
Prev_Horizon_table = Prev_Window_table/TS + 1;

gains_matrix = zeros(length(VX_table), max(Prev_Horizon_table)+2);
gains_matrix(:,1) = 10000;
gains_matrix(:,2) = VX_table;
	 
% >>>>>>>>>>>>>>>>>>>> Vehicle Information <<<<<<<<<<<<<<<<<<<< %
MF = 6000.0; MR = 4000.0; MT = MF + MR; 
LF =  2.317; LR =  2.575;  L = LF + LR;

IZ = 6000*LF*LF + 4000*LR*LR;

KF = -155494; KR = -310989;

% >>>>>>>>>>>>>>>>>>>> K-Table Generation <<<<<<<<<<<<<<<<<<<< %
for i = 1:1:length(VX_table)
	VX = VX_table(i,:);
	
	q1 = q1_table(i,:);
	q2 = q2_table(i,:);
	q3 = q3_table(i,:);
	q4 = q4_table(i,:);
	 r =  r_table(i,:);
	
	Prev_Horizon = Prev_Horizon_table(i,:);
	
	[A_prev, B_prev] = get_prev_model(VX, MT, IZ, LF, LR, KF, KR, TS, Prev_Horizon);
	
	Q_errd	 = diag([q1, q2, q3, q4]);
	R_errd   = r_table(i,:);
	
	Q_prev = [Q_errd, zeros(4,Prev_Horizon); zeros(Prev_Horizon,4), zeros(Prev_Horizon)];
	R_prev = R_errd;
	
	P_total = dare(A_prev, B_prev, Q_prev, R_prev);
	K_total = inv(R_prev + B_prev'*P_total*B_prev)*B_prev'*P_total*A_prev;

	gains_matrix(i, 3:Prev_Horizon+6) = K_total;
    
%     gains_matrix(i, 3:Prev_Horizon+6) = -K_total;
%     gains_matrix(i, 3:6) = -gains_matrix(i, 3:6);

    plot(gains_matrix(i,7:Prev_Horizon+2));
    hold on;
end
plot(zeros(1,max(Prev_Horizon_table)), 'k--');

writetable(array2table(gains_matrix), 'K_table_a.txt');

function [A_prev, B_prev] = get_prev_model(VX, MT, IZ, LF, LR, KF, KR, TS, Prev_Horizon)
	a22 =  2*(KF+KR)/(MT*VX);					b21 = -2*KF/MT;
	a23 = -2*(KF+KR)/MT;						b41 = -2*KF*LF/IZ;
	a24 =  2*(KF*LF-KR*LR)/(MT*VX);

	a42 =  2*(KF*LF-KR*LR)/(IZ*VX);				d21 = 2*(KF*LF-KR*LR)/MT - VX*VX;		
	a43 = -2*(KF*LF-KR*LR)/IZ;					d41 = 2*(KF*LF*LF+KR*LR*LR)/IZ;
	a44 =  2*(KF*LF*LF+KR*LR*LR)/(IZ*VX);
	
	A_errc = [0, 1,   0,   0;	...
			  0, a22, a23, a24;	...
			  0, 0,   0,   1;	...
			  0, a42, a43, a44];
		 
	B_errc = [0; 	...
			  b21; 	...
			  0; 	...
			  b41];

	D_errc = [0; 	...
			  d21; 	...
			  0; 	...					
			  d41];
    
    A_errd = (eye(4) + 0.5*TS*A_errc)/(eye(4) - 0.5*TS*A_errc);
    B_errd = TS * B_errc;
    D_errd = TS * D_errc;
	
	A_road = [zeros(Prev_Horizon - 1, 1), eye(Prev_Horizon - 1); 0, zeros(1, Prev_Horizon - 1)];

	D_aug = [D_errd, zeros(4, Prev_Horizon - 1)];

	A_prev = [A_errd, D_aug; zeros(Prev_Horizon, 4), A_road];
	B_prev = [B_errd; zeros(Prev_Horizon, 1)];
end