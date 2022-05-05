%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Product Department 1
% Published by: Ang Li
% Email: li.ang@cidi.ai
% Copyright (c) 2022 CIDI. All rights reserved.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; close all; clc;

% >>>>>>>>>>>>>>>>>>>> Basic Settings <<<<<<<<<<<<<<<<<<<< %
FLAGS_Vehicle_Code = 2;
FLAGS_Controller_Code = 8;

FLAGS_use_lag_compensation = false;
FLAGS_use_delay_compensation = false;
FLAGS_use_feedforward = false;

TS = 0.01;
TL = 0.20;
TD =   20;

VX_table =([   5,   10,   20,   30,   40,   50,   60,   70,   80]/3.6)';
q1_table = [   1,    1,    1,    1,    1,    1,    1,    1,    1]';
q2_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0]';
q3_table = [  15,   25,   35,   45,   55,   65,   75,   85,   95]';
q4_table = [   0,    0,    0,    0,    0,    0,    0,    0,    0]';
 r_table = [  10,   20,   50,  100,  250,  400,  600,  800, 1200]';
 k_table = [ 1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0]';

Prev_Window_table = ...
		   [ 2.0,  2.0,  2.0,  2.0,  2.0,  2.0,  2.0,  2.0,  2.0]';
	 
Prev_Horizon_table = Prev_Window_table/TS + 1;

OPC_K_Table      = zeros(length(VX_table), 528);
OPC_K_Table(:,1) = 10000;
OPC_K_Table(:,2) = VX_table;
	 
% >>>>>>>>>>>>>>>>>>>> Vehicle Information <<<<<<<<<<<<<<<<<<<< %
switch FLAGS_Vehicle_Code
    case 1
        MF = 6000.0; MR = 4000.0; MT = MF + MR; 
        LF =  2.317; LR =  2.575;  L = LF + LR;
        IZ = 6000*LF*LF + 4000*LR*LR;
        KF = -155494; KR = -310989;
    
    case 2
        MT = 2020.0;
        LF =  1.265; LR =  1.895; L = LF + LR;
        IZ = 4095.0;
        KF = -70000; KR = -70000;
end

% >>>>>>>>>>>>>>>>>>>> K-Table Generation <<<<<<<<<<<<<<<<<<<< %
for i = 1:1:length(VX_table)
	VX = VX_table(i,:);
	
	q1 = q1_table(i,:);
	q2 = q2_table(i,:);
	q3 = q3_table(i,:);
	q4 = q4_table(i,:);
	 r =  r_table(i,:);
	
	Prev_Horizon = Prev_Horizon_table(i,1);
	
	[Matrix_A, Matrix_B] = ...
        get_models(FLAGS_Controller_Code, VX, MT, IZ, LF, LR, KF, KR, TS, TL, TD, Prev_Horizon);
	Matrix_Q = zeros(size(Matrix_A));
    Matrix_Q(1,1) = q1;
    Matrix_Q(2,2) = q2;
    Matrix_Q(3,3) = q3;
    Matrix_Q(4,4) = q4;
    Matrix_R(1,1) =  r;
	
	P_total = dare(Matrix_A, Matrix_B, Matrix_Q, Matrix_R);
	K_total = inv(Matrix_R + Matrix_B'*P_total*Matrix_B)*Matrix_B'*P_total*Matrix_A;

	OPC_K_Table(i, 3:228) = K_total;

    plot(OPC_K_Table(i,7:228));
    hold on;
end
plot(zeros(1,228), 'k--');

writetable(array2table(OPC_K_Table), 'optimal_preview_K_table.txt');

function [Matrix_A, Matrix_B] = ...
    get_models(FLAGS_Model_Type, VX, MT, IZ, LF, LR, KF, KR, TS, TL, TD, Prev_Horizon)
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
    
    A_road = [zeros(Prev_Horizon-1, 1), eye(Prev_Horizon-1); zeros(1, Prev_Horizon)];
    B_road = [zeros(Prev_Horizon-1, 1); 1];

    A_delay = [eye(TD-1); zeros(1, TD-1)];
    B_delay = [zeros(Prev_Horizon-1, 1); 1];

    A_feed = eye(4) + TS * A_errc;
    B_feed =          TS * B_errc;
    D_feed =          TS * D_errc;

    A_feed_L = [A_errc, B_errc; zeros(1, 4), -1/TL];
    B_feed_L = [zeros(4, 1); 1/TL];
    D_feed_L = [D_errc; 0];
    A_feed_L = eye(5) + TS * A_feed_L;
    B_feed_L =          TS * B_feed_L;
    D_feed_L =          TS * D_feed_L;

    A_feed_D = blkdiag([A_feed, B_feed], A_delay);
    B_feed_D = [zeros(3+TD, 1); 1];
    D_feed_D = [D_feed; zeros(TD, 1)];

    A_feed_DL = [A_feed_L, B_feed_L, zeros(5, TD - 1); zeros(TD, 6), [eye(TD - 1); zeros(1, TD - 1)]];
    B_feed_DL = [zeros(4+TD, 1); 1];
    D_feed_DL = [D_feed_L; zeros(TD, 1)];

    A_prev = blkdiag(A_feed, A_road);	A_prev(1:4, 5) = D_feed;
    B_prev = [B_feed; zeros(Prev_Horizon, 1)];
    D_prev = [zeros(3+Prev_Horizon, 1); 1];

    A_prev_L = blkdiag(A_feed_L, A_road);	A_prev_L(1:5, 6) = D_feed_L;
    B_prev_L = [B_feed_L; zeros(Prev_Horizon, 1)];
    D_prev_L = [zeros(4+Prev_Horizon, 1); 1];

    A_prev_D = blkdiag([A_feed, B_feed], A_delay, A_road);	A_prev_D(1:4, 5+TD) = D_feed;
    B_prev_D = [B_feed_D; zeros(Prev_Horizon, 1)];
    D_prev_D = [zeros(3+TD+Prev_Horizon, 1); 1];

    A_prev_DL = blkdiag([A_feed_L, B_feed_L], A_delay, A_road);		A_prev_DL(1:5, 6+TD) = D_feed_L;
    B_prev_DL = [zeros(4+TD, 1); 1; zeros(Prev_Horizon, 1)];
    D_prev_DL = [zeros(4+TD+Prev_Horizon, 1); 1];

    A_feed_I  = [0, zeros(1,4); zeros(4,1), A_errc];
    B_feed_I  = [0; B_errc];
    A_feed_I  = eye(5) + TS * A_feed_I;
    B_feed_I  =          TS * B_feed_I;

    switch FLAGS_Model_Type
        case 1
            Matrix_A = A_feed;
            Matrix_B = B_feed;
        case 2
            Matrix_A = A_feed_L;
            Matrix_B = B_feed_L;
        case 3
            Matrix_A = A_feed_D;
            Matrix_B = B_feed_D;
        case 4
            Matrix_A = A_feed_DL;
            Matrix_B = B_feed_DL;
        case 5
            Matrix_A = A_prev;
            Matrix_B = B_prev;
        case 6
            Matrix_A = A_prev_L;
            Matrix_B = B_prev_L;
        case 7
            Matrix_A = A_prev_D;
            Matrix_B = B_prev_D;
        case 8
            Matrix_A = A_prev_DL;
            Matrix_B = B_prev_DL;   
        case 9
            Matrix_A = A_feed_I;
            Matrix_B = B_feed_I;
    end
end

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
    
    A_errd = eye(4) + TS*A_errc;
    B_errd = TS * B_errc;
    D_errd = TS * D_errc;
	
	A_road = [zeros(Prev_Horizon - 1, 1), eye(Prev_Horizon - 1); 0, zeros(1, Prev_Horizon - 1)];

	D_aug = [D_errd, zeros(4, Prev_Horizon - 1)];

	A_prev = [A_errd, D_aug; zeros(Prev_Horizon, 4), A_road];
	B_prev = [B_errd; zeros(Prev_Horizon, 1)];
end