function draw_circle(x,y,r,color,edge_color,opacity)
theta=0:0.01:2*pi; 
cx = x + r*cos(theta);
cy = y + r*sin(theta);
plot(polyshape(cx,cy),'FaceColor',color,'FaceAlpha',opacity, 'EdgeColor',edge_color,'LineWidth',0.5);
end