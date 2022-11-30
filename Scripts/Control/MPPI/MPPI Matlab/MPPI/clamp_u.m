function result = clamp_u(u, u_max, u_min)
result = u;
if result>u_max
    result = u_max;
end
if result<u_min
    result = u_min;
end
end