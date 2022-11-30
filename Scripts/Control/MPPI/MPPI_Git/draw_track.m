function draw_track(inner_intersec, outer_intersec, mid_intersec)
hold on;
% path = [154.90096, 123.69824;
%          98.12052, 166.64730;
%           1.02179,  34.52508;
%          65.20817,   4.22747];
plot(polyshape(mid_intersec),'FaceColor','r','FaceAlpha',0.0, 'EdgeColor','r','LineWidth',2);
% axis equal;
% outer = [157.7, 124.09;
%          97.71, 169.47;
%          -1.97,  33.77;
%          65.82,   1.77];
plot(polyshape(outer_intersec),'FaceColor','r','FaceAlpha',0.1, 'EdgeColor','k','LineWidth',0.5);

% inner = [152.11, 123.31;
%           98.53, 163.83;
%            4.02,  35.28;
%            64.6,   6.68];
plot(polyshape(inner_intersec),'FaceColor','w','FaceAlpha',1.0, 'EdgeColor','k','LineWidth',0.5);
axis equal;
end