function cost = stateCost(state, state_dot, control, inn_fcn, out_fcn,road_width,obstacles)
%myFun - Description
%
% Syntax: output = myFun(input)
%
% Long description

x       = state(1);
y       = state(2);
phi     = state(3);
v0      = state(4);
x_dot   = state_dot(1);
y_dot   = state_dot(2);
phi_dot = state_dot(3);
v0_dot  = state_dot(4);
vd      = control(1);
wd      = control(2);

cost = 0;

off_road_penalty  = 500;
collision_penalty = 800;
track_err_penalty = 5;
v0_dot_penalty    = 0;
phi_dot_penalty   = 0;
vd_penalty        = 10;
wd_penalty        = 10;
vd_change_penalty = 0;
wd_change_penalty = 0;
% Phisiacal limitations
vd_max =  4.0;
vd_min =  1.5;
wd_max =  1.0;
wd_min = -1.0;


out_f = out_fcn(1,1)*y+out_fcn(1,2)*x+out_fcn(1,3);
out_g = out_fcn(2,1)*y+out_fcn(2,2)*x+out_fcn(2,3);
out_h = out_fcn(3,1)*y+out_fcn(3,2)*x+out_fcn(3,3);
out_i = out_fcn(4,1)*y+out_fcn(4,2)*x+out_fcn(4,3);

inn_f = inn_fcn(1,1)*y+inn_fcn(1,2)*x+inn_fcn(1,3);
inn_g = inn_fcn(2,1)*y+inn_fcn(2,2)*x+inn_fcn(2,3);
inn_h = inn_fcn(3,1)*y+inn_fcn(3,2)*x+inn_fcn(3,3);
inn_i = inn_fcn(4,1)*y+inn_fcn(4,2)*x+inn_fcn(4,3);



% % Designated path
% mid_f = y - 1.33200*x + 82.62978;  % > 0
% mid_g = y + 0.75640*x - 240.86623; % < 0
% mid_h = y - 1.36070*x - 33.13473;  % < 0
% mid_i = y + 0.47203*x - 35.00739;  % > 0
% % outer bound of road (position should be inside the surrounded area)
% out_f = mid_f + (road_width/2)/cos(atan( 1.33200)); % > 0
% out_g = mid_g - (road_width/2)/cos(atan(-0.75640)); % < 0 
% out_h = mid_h - (road_width/2)/cos(atan( 1.36070)); % < 0
% out_i = mid_i + (road_width/2)/cos(atan(-0.47203)); % > 0
% 
% % inner bound of road (position should be outside the surrounded area)
% inn_f = mid_f - (road_width/2)/cos(atan( 1.33200)); % < 0
% inn_g = mid_g + (road_width/2)/cos(atan(-0.75640)); % > 0 
% inn_h = mid_h + (road_width/2)/cos(atan( 1.36070)); % > 0
% inn_i = mid_i - (road_width/2)/cos(atan(-0.47203)); % < 0

% Cost of position
if (out_f>0 && out_g<0 && out_h<0 && out_i>0) && ...
   ~(inn_f>0 && inn_g<0 && inn_h<0 && inn_i>0)
    cost = cost + 0;
else
    cost = cost + off_road_penalty;
end

% Distance from center
distance = distance_from_track(inn_f, inn_g, inn_h, inn_i, road_width, inn_fcn);
cost = cost + distance/(road_width/2)*track_err_penalty;
% cost = cost + distance*track_err_penalty;

% index of the path names
%           g
%        ______ 
%       /     /
%    h /     / f
%     /     /
%     ------
%       i

% % outer bound of road (position should be inside the surrounded area)
% out_f = -119.12*x + 89.48*y + 7681.02;  % > 0
% out_g =   42.96*x + 56.80*y - 13822.83; % < 0 
% out_h = -132.48*x + 97.31*y - 3547.03;  % < 0
% out_i =   30.29*x + 64.17*y - 2107.13;  % > 0
% 
% % inner bound of road (position should be outside the surrounded area)
% inn_f = -119.82*x + 89.91*y + 7139.93;  % < 0
% inn_g =   42.94*x + 56.76*y - 13530.23; % > 0 
% inn_h = -131.77*x + 96.89*y - 2889.29;  % > 0
% inn_i =   30.31*x + 64.20*y - 2386.94;  % < 0

% % Cost of position
% if (out_f>0 && out_g<0 && out_h<0 && out_i>0) && ...
%    (inn_f<0 || inn_g>0 || inn_h>0 || inn_i<0)
%     cost = cost + 0;
% else
%     cost = cost + off_road_penalty;
%     disp('OFF_ROAD');
% end

% Cost of control
% cost = cost + v0_dot_penalty*v0_dot + phi_dot_penalty*phi_dot + ...
%        vd_change_penalty*abs(vd-v0) + wd_change_penalty*abs(wd-phi_dot);    
   
collision = obstacle_collide(state, obstacles);
obs_shape = size(collision);
for obs = 1:1:obs_shape(1)
    if collision(obs,1) == 1
        cost= cost + abs(collision(obs,2))*collision_penalty;
    end
end
if vd>vd_max || vd<vd_min
    cost = cost+vd_penalty;
end
if wd>wd_max || wd<wd_min
    cost = cost+wd_penalty;
end

end