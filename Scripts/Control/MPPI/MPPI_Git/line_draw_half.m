function line_draw_half(center,length,phi,color,width)
 c1 = center(1);
 c2 = center(2);
 L = length;
 alpha = phi;
 x2=c1+(L*cos(alpha));
 y2=c2+(L*sin(alpha));
 plot([c1 x2],[c2 y2], color,'LineWidth', width);
 hold off;
end