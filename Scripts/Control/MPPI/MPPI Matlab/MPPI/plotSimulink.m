data = out.simout.Data;
x = [];
y = [];
for i = 1:length(data)
    x(i) = data(1,1,i);
    y(i) = data(2,1,i);
end
pathfig = figure(1);
set(pathfig, 'position', [240 90 1440 900]);
plot(refPos_x,refPos_y,'r')
axis equal;
hold on
plot(x,y,'b');