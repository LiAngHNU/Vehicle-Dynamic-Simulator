function xk1 = vehicle_dynamic(xk ,uk ,v ,param)
dt = param.dt;
Caf = 50000;
Car = Caf;
lf = 1.65;
lr = 1.35;
Iz = 15000;
m = param.m;

A = [0 1 0 0;
    0 -(2*Caf+2*Car)/(m*v) 0 -(2*lf*Caf-2*lr*Car)/(m*v)-v;
    0 0 0 1;
    0 (2*Car*lr-2*Caf*lf)/(Iz*v) 0 -(2*lf^2*Caf+2*lr^2*Car)/(Iz*v)];
B  = [0;2*Caf/m;0;2*lf*Caf/Iz];
At=(eye(4,4)+(dt/2)*A)/(eye(4,4)-(dt/2)*A);
Bt=B*dt;

xk1 = At*xk + Bt*uk;
end