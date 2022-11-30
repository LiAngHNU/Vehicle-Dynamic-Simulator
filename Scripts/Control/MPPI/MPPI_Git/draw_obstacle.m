function draw_obstacle(pose_and_size)
[n_obstacles, col_] = size(pose_and_size);
for nth_obstacle = 1:1:n_obstacles
    px    = pose_and_size(nth_obstacle,1);
    py    = pose_and_size(nth_obstacle,2);
    psize = pose_and_size(nth_obstacle,3);
    draw_circle(px,py,psize,'b','k',.3);
    draw_circle(px,py,.1,'b','k',1);
end
end