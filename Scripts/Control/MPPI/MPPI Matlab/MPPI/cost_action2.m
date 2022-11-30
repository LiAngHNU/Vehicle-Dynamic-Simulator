function cost = cost_action2(a, param)

num = length(a);
cost = 0;
for i=2:num
    cost = cost + param.omega*(a(i)-a(i-1))^2;
end
end