function X = vehicle_kinetics(X ,delta ,param)
v = param.v;
dt = param.dt;
L = param.L;
xo = X(1);
yo = X(2);
yaw = X(3);
x = xo + v * cos(yaw) * dt;
y = yo + v * sin(yaw) * dt;
yaw = yaw + v / L * tan(delta) * dt;
X = [x;y;yaw];
end