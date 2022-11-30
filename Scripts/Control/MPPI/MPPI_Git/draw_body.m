function [front_wheel_pos, rear_wheel_pos] = draw_body(x,y,phi,L_r,L_f,width)
% create body along x-axis at (0,0)
body = [0+L_f,0+width/2;
        0+L_f,0-width/2;
        0-L_r,0-width/2;
        0-L_r,0+width/2];
    
% rotate by phi using rotation matrix
r_matrix = [ cos(phi), sin(phi);
            -sin(phi), cos(phi)];
body = body*r_matrix;

% translate to correct position
body = body + [x,y;
               x,y;
               x,y;
               x,y];
           
plot(polyshape(body),'FaceColor','r','FaceAlpha',1);

% calculate front/rear wheel position
rear_x  = x - (L_r*cos(phi));
rear_y  = y - (L_r*sin(phi)); 
front_x = x + (L_f*cos(phi));
front_y = y + (L_f*sin(phi));

front_wheel_pos = [front_x, front_y];
rear_wheel_pos  = [rear_x,   rear_y];
end