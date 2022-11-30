param.dt = 0.01;
param.v = 10;              %5m/s
param.m = 1400;
param.g = 9.81;
param.L = 4;

u = U_sys(1:458);
t = length(u);

x_kinemic(:,1) = [0;0;0];
x_neural(:,1) = [0;0;0];
plot_comp = figure;
scatter(x_kinemic(1,1),x_kinemic(2,1),500,'g.')
hold on;
scatter(x_neural(1,1),x_neural(2,1),400,'r.')
for i = 1:t
    x_kinemic(:,i+1) = vehicle_kinetics(x_kinemic(:,i) ,u(i) ,param);
    x_neural(:,i+1) = x_neural(:,i) + myNeuralNetworkFunctionS([x_neural(3,i);u(i);param.v]).*param.dt;
    plot_comp;
    hold on;
    scatter(x_kinemic(1,i+1),x_kinemic(2,i+1),500,'g.');
    hold on;
    scatter(x_neural(1,i+1),x_neural(2,i+1),100,'r.');
    drawnow;
end