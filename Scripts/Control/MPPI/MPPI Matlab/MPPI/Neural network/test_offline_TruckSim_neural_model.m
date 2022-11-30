clc;
clear;
close all;

load("TruckSimData1.mat");

num = length(Ax);
cur_pred_posi = [X_cg(1) Y_cg(1) Yaw(1)];
plot_comp = figure;
scatter(X_cg(1),Y_cg(1),500,'g.')
hold on;
scatter(cur_pred_posi(1),cur_pred_posi(2),100,'r.');
for i=1:num
    cur_pred_posi = cur_pred_posi + myNeuralNetworkFunctionTruckSim([steer(i) cur_pred_posi(3) V_x(i)])*0.001;
    plot_comp;
    hold on;
    scatter(X_cg(i),Y_cg(i),500,'g.');
    hold on;
    scatter(cur_pred_posi(1),cur_pred_posi(2),100,'r.');
    drawnow;
end