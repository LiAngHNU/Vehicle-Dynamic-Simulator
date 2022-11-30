function DrawBicycle(state, control)
% Set parameters
b             = 1;           % distance between front and rear wheel, unit: meter 
wheel_radius  = 0.65/2;      % radius of bicycle wheel,               unit: meter
epsilon       = deg2rad(70); % fork angle of bicycle,                 unit: radius 
wheel_width   = 0.045;       % width of wheel,                        unit: meter
body_width    = 0.045;       % width of bicycle,                      unit: meter
a             = 0.2;         % distance from COG to rear wheel        unit: meter
handle_length = 0.62;        % length of handle                       unit: meter

% x   = 1;
% y   = 1;
% phi = 0;
% v0  = 2;
% vd  = 2;
% wd  = 0.5;
% delta = atan(wd*b/v0)/sin(epsilon);

x   = state(1);
y   = state(2);
phi = state(3);
v0  = state(4);
vd  = control(1);
wd  = control(2);
delta = atan(wd*b/v0)/sin(epsilon);

hold on;
% Draw body
[pos_f,pos_r] = draw_body(x,y,phi,a,b-a,body_width);

% Draw rear wheel
draw_wheel(pos_r(1),pos_r(2),phi,wheel_radius,2*wheel_width,'#0072BD',.2);

% Draw front wheel
draw_wheel(pos_f(1),pos_f(2),delta+phi,wheel_radius,2*wheel_width,'#0072BD',.2);

% Draw handle
draw_wheel(pos_f(1),pos_f(2),delta+phi+pi/2,handle_length/2,body_width,'r',1);

% Draw COG of bicycle
COG_size = 40;
scatter(x,y,COG_size,'MarkerEdgeColor','k',...
                     'MarkerFaceColor',[0.9290 0.6940 0.1250]);
end