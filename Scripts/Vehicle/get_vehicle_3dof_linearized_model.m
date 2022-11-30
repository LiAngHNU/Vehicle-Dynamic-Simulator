% // Initialization //
close all; clear; clc;
load("Vehicles\Truck_106_Daimler_Auman.mat");

% // Configuration //
FLAGS_use_Small_Angle_Hypothesis        = true;
FLAGS_use_Small_Acceleration_Hypothesis = true;

% // Symbolic Variable Declaration //
syms delta_swf delta_swr;
syms delta_FL delta_FR delta_RL delta_RR delta_F delta_R;
syms   trq_FL   trq_FR   trq_RL   trq_RR   trq_F   trq_R;

syms  x_veh  y_veh  z_veh rho_veh pch_veh yaw_veh;
syms vx_veh vy_veh vz_veh avx_veh avy_veh avz_veh;
syms ax_veh ay_veh az_veh aax_veh aay_veh aaz_veh;

syms kappa_whl_FL kappa_whl_FR kappa_whl_RL kappa_whl_RR;
syms alpha_whl_FL alpha_whl_FR alpha_whl_RL alpha_whl_RR;
syms    vx_whl_FL    vx_whl_FR    vx_whl_RL    vx_whl_RR;
syms    vy_whl_FL    vy_whl_FR    vy_whl_RL    vy_whl_RR; 
syms     v_whl_FL     v_whl_FR     v_whl_RL     v_whl_RR;
syms       avy_FL       avy_FR       avy_RL       avy_RR;

syms fx_total fy_total mz_total;

syms M Iz Cx Cy;
syms W_FL W_FR W_RL W_RR W_F W_R W;
syms L_FL L_FR L_RL L_RR L_F L_R L;
syms R_whl;

syms bank_road slope_road theta_road;
syms                      kappa_road;
syms                     dkappa_road;

syms  error_s  error_l error_yaw;
syms error_vs error_vl error_avz;
syms error_as error_al error_aaz;

    W_whl = [W_FL; W_FR; W_RL; W_RR];
    L_whl = [L_FL; L_FR; L_RL; L_RR];
  avy_whl = [avy_FL; avy_FR; avy_RL; avy_RR];
  trq_whl = [trq_FL; trq_FR; trq_RL; trq_RR];
delta_whl = [delta_F; delta_F; delta_R; delta_R];

% // Calculate Wheel Velocity //
if(FLAGS_use_Small_Angle_Hypothesis)
    for i = 1:1:4
        vx_whl(i,1) = vx_veh - W_whl(i,1)*avz_veh;
        vy_whl(i,1) = vy_veh + L_whl(i,1)*avz_veh; 
        v_whl(i,1) = sqrt(vx_whl(i,1)^2 + vy_whl(i,1)^2);
    end
else
    for i = 1:1:4
        vx_whl(i,1) = vx_veh - W_whl(i,1)*avz_veh;
        vy_whl(i,1) = vy_veh + L_whl(i,1)*avz_veh; 
        v_whl(i,1) = cos(delta_whl(i,1))*sqrt(vx_whl(i,1)^2 + vy_whl(i,1)^2);
    end
end

% // Calculate Wheel Kinematics //
if(FLAGS_use_Small_Angle_Hypothesis)
    for i = 1:1:4
        kappa_whl(i,1) = (avy_whl(i,1)*R_whl - v_whl(i,1))/v_whl(i,1);
        alpha_whl(i,1) = vy_whl(i,1)/vx_whl(i,1) - delta_whl(i,1);
    end
else
    for i = 1:1:4
        kappa_whl(i,1) = (avy_whl(i,1)*R_whl - v_whl(i,1))/v_whl(i,1);
        alpha_whl(i,1) = atan(vy_whl(i,1)/vx_whl(i,1)) - delta_whl(i,1);
    end
end

% // Calculate Wheel Dynamics //
if(FLAGS_use_Small_Angle_Hypothesis)
    for i = 1:1:4
        fx_whl(i,1) = Cx*kappa_whl(i,1);
        fy_whl(i,1) = Cy*alpha_whl(i,1);
    
        fx_single(i,1) = fx_whl(i,1) - fy_whl(i,1)*delta_whl(i,1);
        fy_single(i,1) = fx_whl(i,1)*delta_whl(i,1) + fy_whl(i,1);
        mz_single(i,1) = -W_whl(i,1)*fx_single(i,1) + L_whl(i,1)*fy_single(i,1);
    end
else
    for i = 1:1:4
        fx_whl(i,1) = Cx*kappa_whl(i,1);
        fy_whl(i,1) = Cy*alpha_whl(i,1);
    
        fx_single(i,1) = fx_whl(i,1)*cos(delta_whl(i,1)) - fy_whl(i,1)*sin(delta_whl(i,1));
        fy_single(i,1) = fx_whl(i,1)*sin(delta_whl(i,1)) + fy_whl(i,1)*cos(delta_whl(i,1));
        mz_single(i,1) = -W_whl(i,1)*fx_single(i,1) + L_whl(i,1)*fy_single(i,1);
    end
end

fx_total = sum(fx_single);
fy_total = sum(fy_single);
mz_total = sum(mz_single);

% // Calculate Vehicle 7DOF Model //
% fx_total = + fx_whl*cos(delta_whl) - fy_whl*sin(delta_whl);
% fy_total = + fx_whl*sin(delta_whl) + fy_whl*cos(delta_whl);
% mz_total = -W*fx_total + L*fy_total;
% aay_whl  = t_whl 

% M*ax_veh - M*vy_veh*avz_veh = fx_total;
% M*ay_veh + M*vx_veh*avz_veh = fy_total;
%                  Iz*aaz_veh = mz_total;
%              Iy_whl*aay_whl = t_whl - fx_whl*R_whl;

% // Calculate Jacobian Matrix
 ax_veh =  (1/M)*fx_total + vy_veh*avz_veh;
 ay_veh =  (1/M)*fy_total - vx_veh*avz_veh;
aaz_veh = (1/Iz)*mz_total;

Matrix_A_3DOF_SYM = jacobian([ax_veh; ay_veh; aaz_veh], ...
                             [vx_veh; vy_veh; avz_veh]);
Matrix_B_3DOF_SYM = jacobian([ax_veh; ay_veh; aaz_veh], ...
                             [delta_F; delta_R]);

% // Post Processing //
Matrix_A_3DOF_SYM = subs(Matrix_A_3DOF_SYM, vx_whl(1,1), vx_whl_FL);
Matrix_A_3DOF_SYM = subs(Matrix_A_3DOF_SYM, vx_whl(2,1), vx_whl_FR);
Matrix_A_3DOF_SYM = subs(Matrix_A_3DOF_SYM, vx_whl(3,1), vx_whl_RL);
Matrix_A_3DOF_SYM = subs(Matrix_A_3DOF_SYM, vx_whl(4,1), vx_whl_RR);

Matrix_A_3DOF_SYM = subs(Matrix_A_3DOF_SYM, vy_whl(1,1), vx_whl_FL);
Matrix_A_3DOF_SYM = subs(Matrix_A_3DOF_SYM, vy_whl(2,1), vx_whl_FR);
Matrix_A_3DOF_SYM = subs(Matrix_A_3DOF_SYM, vy_whl(3,1), vx_whl_RL);
Matrix_A_3DOF_SYM = subs(Matrix_A_3DOF_SYM, vy_whl(4,1), vx_whl_RR);

Matrix_B_3DOF_SYM = subs(Matrix_B_3DOF_SYM, alpha_whl(1,1), alpha_whl_FL);
Matrix_B_3DOF_SYM = subs(Matrix_B_3DOF_SYM, alpha_whl(2,1), alpha_whl_FR);
Matrix_B_3DOF_SYM = subs(Matrix_B_3DOF_SYM, alpha_whl(3,1), alpha_whl_RL);
Matrix_B_3DOF_SYM = subs(Matrix_B_3DOF_SYM, alpha_whl(4,1), alpha_whl_RR);

Matrix_B_3DOF_SYM = subs(Matrix_B_3DOF_SYM, kappa_whl(1,1), kappa_whl_FL);
Matrix_B_3DOF_SYM = subs(Matrix_B_3DOF_SYM, kappa_whl(2,1), kappa_whl_FR);
Matrix_B_3DOF_SYM = subs(Matrix_B_3DOF_SYM, kappa_whl(3,1), kappa_whl_RL);
Matrix_B_3DOF_SYM = subs(Matrix_B_3DOF_SYM, kappa_whl(4,1), kappa_whl_RR);

a11 = ;
a12 = ;
a13 = ;

% // Error Model //
 error_vl = vx_veh*error_yaw + vy_veh;                                      vy_veh = error_vl - vx_veh*error_yaw;
 error_al = ax_veh*error_yaw + vx_veh*error_avz + ay_veh;
error_avz = avz_veh - vx_veh*kappa_road;                                   avz_veh = error_avz + vx_veh*kappa_road;
error_aaz = aaz_veh - ax_veh*kappa_road - vx_veh*dkappa_road;

% Matrix_A_7DOF = subs(Matrix_A_7DOF_SYM);
% Matrix_B_7DOF = subs(Matrix_B_7DOF_SYM);