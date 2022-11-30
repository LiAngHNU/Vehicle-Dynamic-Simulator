function draw_wheel(x,y,phi,radius,width, color, opacity)
% create wheel along x-axis 
wheel = [0+radius,0+width/2;
         0+radius,0-width/2;
         0-radius,0-width/2;
         0-radius,0+width/2];
% rotate by phi using rotation matrix
r_matrix = [ cos(phi), sin(phi);
            -sin(phi), cos(phi)];
wheel = wheel*r_matrix;
% translate to correct position
wheel = wheel + [x,y;
                 x,y;
                 x,y;
                 x,y];
plot(polyshape(wheel),'FaceColor',color,'FaceAlpha',opacity);
end