function cost = cost_obs(x,x_obs,r)

dist = pdist2(x, x_obs);

if dist > r
    cost = 0;
else
    cost = 100000;
end
end