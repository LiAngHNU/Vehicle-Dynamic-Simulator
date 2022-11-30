%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control System Analysis                                                 %
% Hunan University                                                        %
% Author: Ang Li                                                          %
% Email: ang1997@hnu.edu.cn                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% System Parameters Definition                                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
syms delta_swf delta_swr;

syms  x_veh  y_veh  z_veh rol_veh pit_veh yaw_veh;
syms vx_veh vy_veh vz_veh avx_veh avy_veh avz_veh;
syms ax_veh ay_veh az_veh aax_veh aay_veh aaz_veh;
 
syms delta_F delta_R;
syms trq_FL trq_FR trq_RL trq_RR;

delta_whl = [delta_F; delta_F; delta_R; delta_R];
  trq_whl = [trq_FL; trq_FR; trq_RL; trq_RR];

syms avy_FL avy_FR avy_RL avy_RR;

syms M Iz Cx Cy;

syms fx_total fy_total mz_total

syms W_FL W_FR W_RL W_RR;
syms L_FL L_FR L_RL L_RR;
syms R_whl;

  W_whl = [W_FL; W_FR; W_RL; W_RR];
  L_whl = [L_FL; L_FR; L_RL; L_RR];
avy_whl = [avy_FL; avy_FR; avy_RL; avy_RR];

% //Calculate Wheel Velocity//
for i = 1:1:4
    vx_whl(i,1) = vx_veh - W_whl(i,1)*avz_veh;
    vy_whl(i,1) = vy_veh + L_whl(i,1)*avz_veh;
     v_whl(i,1) = cos(delta_whl(i,1))*sqrt(vx_whl(i,1)^2 + vy_whl(i,1)^2);
end

% //Calculate Wheel Kinematics//
for i = 1:1:4
    kappa_whl(i,1) = (avy_whl(i,1)*R_whl - v_whl(i,1))/v_whl(i,1);
    alpha_whl(i,1) = atan(vy_whl(i,1)/vx_whl(i,1)) - delta_whl(i,1);
end

% //Calculate Wheel Dynamics//
for i = 1:1:4
    fx_whl(i,1) = Cx*kappa_whl(i,1);
    fy_whl(i,1) = Cy*alpha_whl(i,1);
    
    fx_single(i,1) = fx_whl(i,1)*cos(delta_whl(i,1)) - fy_whl(i,1)*sin(delta_whl(i,1));
    fy_single(i,1) = fx_whl(i,1)*sin(delta_whl(i,1)) + fy_whl(i,1)*cos(delta_whl(i,1));
    mz_single(i,1) = -W_whl(i,1)*fx_single(i,1) + L_whl(i,1)*fy_single(i,1);
end

fx_total = sum(fx_single);
fy_total = sum(fy_single);
mz_total = sum(mz_single);

% //Calculate Vehicle 7DOF Model//
% fx_total = + fx_whl*cos(delta_whl) - fy_whl*sin(delta_whl);
% fy_total = + fx_whl*sin(delta_whl) + fy_whl*cos(delta_whl);
% mz_total = -W*fx_total + L*fy_total;
% aay_whl  = t_whl 

% M*ax_veh - M*vy_veh*avz_veh = fx_total;
% M*ay_veh + M*vx_veh*avz_veh = fy_total;
%                  Iz*aaz_veh = mz_total;
%              Iy_whl*aay_whl = t_whl - fx_whl*R_whl;

% //Calculate Jacobian Matrix
 ax_veh =  (1/M)*fx_total + vy_veh*avz_veh;
 ay_veh =  (1/M)*fy_total - vx_veh*avz_veh;
aaz_veh = (1/Iz)*mz_total;
aay_FL  = trq_FL - fx_whl(1,1)*R_whl;
aay_FR  = trq_FR - fx_whl(2,1)*R_whl;
aay_RL  = trq_RL - fx_whl(3,1)*R_whl;
aay_RR  = trq_RR - fx_whl(4,1)*R_whl;

Matrix_A_7DOF_SYM = jacobian([ax_veh; ay_veh; aaz_veh; aay_FL; aay_FR; aay_RL; aay_RR], ...
                             [vx_veh; vy_veh; avz_veh; avy_FL; avy_FR; avy_RL; avy_RR]);
Matrix_B_7DOF_SYM = jacobian([ax_veh; ay_veh; aaz_veh; aay_FL; aay_FR; aay_RL; aay_RR], ...
                             [delta_F; delta_R; trq_FL; trq_FR; trq_RL; trq_RR]);

% Matrix_A_7DOF = subs(Matrix_A_7DOF_SYM);
% Matrix_B_7DOF = subs(Matrix_B_7DOF_SYM);