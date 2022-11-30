function result = obstacle_collide(state,pos_and_size)
% Arguments: 
%   pos_and_size: a list to record the position and dimentions(radius) of the obstacle
%   state:        state of the vehicle [x,y,phi,v0]
%
%                 obstacle_n: (x-px)^2+(y-py)^2=r^2
%
%                 [px1, py1, r1]  --> obstacle 1
%         ex.     [px2, py2, r2]  --> obstacle 2
%                 [px3, py3, r3]  --> obstacle 3 ...
% Output:
%   result: gives 1 for collision and 0 for no-collision, also pairs the
%           obstacle id with the corresponding result
%
%                 [1, val1, 1]  --> collide with obstacle 1
%         ex.     [0, val2, 2]  --> non-collision with obstacle 2
%                 [0, val3, 3]  --> non-collision with obstacle 3 ...

% Import parameters
x   = state(1);
y   = state(2);
phi = state(3);

% initialize result matrix 
[n_obstacles, col_] = size(pos_and_size);
result = zeros(n_obstacles,3);

for nth_obstacle=1:1:n_obstacles
    obstacle_fcn = (x-pos_and_size(nth_obstacle, 1))^2+(y-pos_and_size(nth_obstacle, 2))^2 ...
                   - pos_and_size(nth_obstacle, 3)^2;
    if obstacle_fcn < 0
        result(nth_obstacle, 1) = 1;
    end
    result(nth_obstacle, 2) = -obstacle_fcn/pos_and_size(nth_obstacle, 3)^2;
    result(nth_obstacle, 3) = nth_obstacle;
end
    
end