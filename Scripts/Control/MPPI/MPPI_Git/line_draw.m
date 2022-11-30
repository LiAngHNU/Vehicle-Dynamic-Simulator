function line_draw(X,Y,phi,L_f,L_r,color,width)
 c1 = X;
 c2 = Y;
 alpha = phi;
 x1=c1-(L_r*cos(alpha));
 y1=c2-(L_r*sin(alpha)); 
 x2=c1+(L_f*cos(alpha));
 y2=c2+(L_f*sin(alpha));
 plot([c1 x2],[c2 y2], color,'LineWidth', width);
 hold on;
 plot([c1 x1],[c2 y1], color,'LineWidth', width);
 hold on;
end