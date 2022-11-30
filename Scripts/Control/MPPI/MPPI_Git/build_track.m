function [outer_fcn, inner_fcn, mid_fcn, outer_intersec, ...
          inner_intersec, mid_intersec] = build_track(mid_fcn, road_width)
% fcn in the form of by + ax + c = 0
%
%       [b1 a1 c1]               [x1,y1]
%       [b2 a2 c2]               [x2,y2]
% fcn = [b3 a3 c3];   intersec = [x3,y3];
%       [b4 a4 c4]               [x4,y4]


mid_m = -mid_fcn(:,2); % Slope of the functions
outer_fcn = mid_fcn;
outer_fcn(:,3) = outer_fcn(:,3) + [ (road_width/2)/cos(atan(mid_m(1)));
                                   -(road_width/2)/cos(atan(mid_m(2)));
                                   -(road_width/2)/cos(atan(mid_m(3)));
                                    (road_width/2)/cos(atan(mid_m(4)))];

inner_fcn = mid_fcn;
inner_fcn(:,3) = inner_fcn(:,3) + [-(road_width/2)/cos(atan(mid_m(1)));
                                    (road_width/2)/cos(atan(mid_m(2)));
                                    (road_width/2)/cos(atan(mid_m(3)));
                                   -(road_width/2)/cos(atan(mid_m(4)))];
                               
% index of the path names
%           g
%        ______ 
%       /     /
%    h /     / f
%     /     /
%     ------
%       i
% f,g intersection: [x1,y1]
% g,h intersection: [x2,y2]
% h,i intersection: [x3,y3]
% i,f intersection: [x4,y4]

mid_intersec   = [get_intersection(mid_fcn(1,:),mid_fcn(2,:));
                  get_intersection(mid_fcn(2,:),mid_fcn(3,:));
                  get_intersection(mid_fcn(3,:),mid_fcn(4,:));
                  get_intersection(mid_fcn(4,:),mid_fcn(1,:))];
            
inner_intersec = [get_intersection(inner_fcn(1,:),inner_fcn(2,:));
                  get_intersection(inner_fcn(2,:),inner_fcn(3,:));
                  get_intersection(inner_fcn(3,:),inner_fcn(4,:));
                  get_intersection(inner_fcn(4,:),inner_fcn(1,:))];
            
outer_intersec = [get_intersection(outer_fcn(1,:),outer_fcn(2,:));
                  get_intersection(outer_fcn(2,:),outer_fcn(3,:));
                  get_intersection(outer_fcn(3,:),outer_fcn(4,:));
                  get_intersection(outer_fcn(4,:),outer_fcn(1,:))];          
end
