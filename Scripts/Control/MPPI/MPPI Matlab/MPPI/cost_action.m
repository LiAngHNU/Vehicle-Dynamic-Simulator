function cost = cost_action(a,a0,param)

omega = param.omega;

cost = (a(1)-a0)*omega*(a(1)-a0);
num = length(a);
for i = 2:num
    cost = cost + (a(i)-a(i-1))*omega*(a(i)-a(i-1));
end
end