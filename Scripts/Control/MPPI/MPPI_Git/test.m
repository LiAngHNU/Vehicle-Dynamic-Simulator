% for i=1:150
%     x = 0+0.5*i;
%     y = 0;
%     if i < 50
%         phi = 0;
%     else
%         phi = 0.01*(i-51);
%     end
%     v0 = 2;
%     vd = 2;
%     wd = 0.3*exp(0.01*i);
%     
%     % Generate state and control
%     state = [x,y,phi,v0];
%     control = [vd,wd];
%     
%     % Create single figure
%     figure(1);
%     % Clear the plot to elimanate old image
%     clf;
%     % Call function to draw the bicycle
%     DrawBicycle(state, control);
% %     axis([0,100,-2,2]);
%     axis equal;
%     
% end

x = 136.03;
y = 137.93;
road_width = 2;

% % outer bound of road (position should be inside the surrounded area) Road width = 4
% out_f = -119.12*x + 89.48*y + 7681.02;  % > 0
% out_g =   42.96*x + 56.80*y - 13822.83; % < 0 
% out_h = -132.48*x + 97.31*y - 3547.03;  % < 0
% out_i =   30.29*x + 64.17*y - 2107.13;  % > 0
% 
% % inner bound of road (position should be outside the surrounded area) Road width = 4
% inn_f = -119.82*x + 89.91*y + 7139.93;  % < 0
% inn_g =   42.94*x + 56.76*y - 13530.23; % > 0 
% inn_h = -131.77*x + 96.89*y - 2889.29;  % > 0
% inn_i =   30.31*x + 64.20*y - 2386.94;  % < 0

% Designated path
mid_f = y - 1.33200*x + 82.62978;  % > 0
mid_g = y + 0.75640*x - 240.86623; % < 0
mid_h = y - 1.36070*x - 33.13473;  % < 0
mid_i = y + 0.47203*x - 35.00739;  % > 0
% outer bound of road (position should be inside the surrounded area)
out_f = mid_f + (road_width/2)/cos(atan( 1.33200)); % > 0
out_g = mid_g - (road_width/2)/cos(atan(-0.75640)); % < 0 
out_h = mid_h - (road_width/2)/cos(atan( 1.36070)); % < 0
out_i = mid_i + (road_width/2)/cos(atan(-0.47203)); % > 0

% inner bound of road (position should be outside the surrounded area)
inn_f = mid_f - (road_width/2)/cos(atan( 1.33200)); % < 0
inn_g = mid_g + (road_width/2)/cos(atan(-0.75640)); % > 0 
inn_h = mid_h + (road_width/2)/cos(atan( 1.36070)); % > 0
inn_i = mid_i - (road_width/2)/cos(atan(-0.47203)); % < 0

% % Cost of position
% if (out_f>0 && out_g<0 && out_h<0 && out_i>0) && ...
%    (inn_f<0 || inn_g>0 || inn_h>0 || inn_i<0)
% Cost of position
if (out_f>0 && out_g<0 && out_h<0 && out_i>0) && ...
   ~(inn_f>0 && inn_g<0 && inn_h<0 && inn_i>0)
%     cost = cost + 0;
    disp('GOOD!!!');
else
    fprintf('WRONG %.2f %.2f %.2f %.2f .\n', inn_f, inn_g, inn_h, inn_i);
    if (out_f>0 && out_g<0 && out_h<0 && out_i>0)
        disp('inside outer');
    end
    if (inn_f>0 && inn_g<0 && inn_h<0 && inn_i>0)
        disp('outside inner');
        fprintf('WRONG %.2f %.2f %.2f %.2f .\n', inn_f, inn_g, inn_h, inn_i);
    end
%     cost = cost + off_road_penalty;
    disp('OFF_ROAD');
end

