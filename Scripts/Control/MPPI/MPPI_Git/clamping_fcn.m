function clamped_u = clamping_fcn(u,vd_max,vd_min,wd_max,wd_min)
v = u(1);
w = u(2);
if v > vd_max
    v = vd_max;
end 
if v < vd_min
    v = vd_min;
end
if w > wd_max
    w = wd_max;
end
if w < wd_min
    w = wd_min;
end
clamped_u = [v, w];
end