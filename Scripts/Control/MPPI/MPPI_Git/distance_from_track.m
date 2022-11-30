function distance = distance_from_track(inn_f, inn_g, inn_h, inn_i, road_width, inn_fcn)
% Outputs the distance between the vehicle and track 
% Uses inner bound as calculation to prevent +/- errors
% When vehicle is in the area bounded by two lines, use circle distance as
% measurement.
%
% Arguments:
%   inn_      : the output value of substituding (x,y) into the 4 line eqs
%   road_width: width of valid track
%   inn_fcn   : by + ax +c = 0 (4*3 matrix recording [b a c])
%
% index of the path names
%           g
%        ______ 
%       /     /
%    h /     / f
%     /     /
%     ------
%       i

% Get the slope of the 4 inner-lines 
slope = -inn_fcn(:,2);

% Initialize distance
distance = 0;

% Determine which line equation the vehicle should be following
if    (inn_f<0&&inn_g<0&&inn_h<0&&inn_i>0)
    distance = abs(abs(inn_f) - 0.5*road_width/cos(atan(slope(1))));
    
elseif(inn_f<0&&inn_g>0&&inn_h<0&&inn_i>0)
    distance = abs(sqrt(inn_f^2+inn_g^2) - 0.5*road_width);
    
elseif(inn_f>0&&inn_g>0&&inn_h<0&&inn_i>0)
    distance = abs(abs(inn_g) - 0.5*road_width/cos(atan(slope(2))));
    
elseif(inn_f>0&&inn_g>0&&inn_h>0&&inn_i>0)
    distance = abs(sqrt(inn_g^2+inn_h^2) - 0.5*road_width);
    
elseif(inn_f>0&&inn_g<0&&inn_h>0&&inn_i>0)
    distance = abs(abs(inn_h) - 0.5*road_width/cos(atan(slope(3))));
    
elseif(inn_f>0&&inn_g<0&&inn_h>0&&inn_i<0)
    distance = abs(sqrt(inn_h^2+inn_i^2) - 0.5*road_width);
    
elseif(inn_f>0&&inn_g<0&&inn_h<0&&inn_i<0)
    distance = abs(abs(inn_i) - 0.5*road_width/cos(atan(slope(4))));
    
elseif(inn_f>0&&inn_g<0&&inn_h<0&&inn_i<0)
    distance = abs(sqrt(inn_i^2+inn_f^2) - 0.5*road_width);
    
elseif(inn_f==0||inn_g==0||inn_h==0||inn_i==0)
    inn_dis = [inn_f, inn_g, inn_h, inn_i];
    min_d = abs(inn_dis);
    distance = min(min_d(min_d>0)) - 0.5*road_width; 
else
%     disp('ERROR');
    
%     fprintf('ERROR %d %d %d %d.\n', inn_f, inn_g, inn_h, inn_i);
end

% Only gives penalties when vehicle is in valid roads 
if distance > road_width/2
    distance = 2;
%     distance = road_width/2;
end
% disp(distance);
end