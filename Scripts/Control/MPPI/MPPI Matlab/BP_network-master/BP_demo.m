% A demo to BP nerual network
% the demo data is watermelon_3.0_alpha.csv

% Liu Jingchang     2016/3/9
% log:

% get data, input x and its size, output y
data = csvread('.\watermelon_3.0_alpha.csv',1,1);
x = data(:,1:end-1)
y = data(:,end)
temp = size(x)
m = temp(1)  % number of rows
n = temp(2)  % number of cols

% define the sigmoid function
f = @(x) 1/(1+exp(-x))

% initialize weights and thresholds of network stochasticly
    % enter the learning rate
delta = input('Please enter the learning rate: ')
    % the weights between input and hidden layer 
V = rand(2,2)
    % the weights between hidden and output layer
W = rand(2,1)
    % the threshold of hidden layer
gamma = rand(2,1)
    % the threshold of output layer
theta = rand(1,1)
    % the output of neurons in hidden layer
b = [1;1]
    % gradient of hidden layer
e = [1;1]
    
% initialize the condition to terminate loop
diff = 2


while(diff > 0.0001)
    for k = 1:m  % for all samples
        
        % the obtained output
        b(1) = f(V(:,1)'*x(k,:)' -gamma(1))
        b(2) = f(V(:,2)'*x(k,:)' -gamma(2))
        y_bar = f(W'*b -theta)
        
        % gradient of output layer
        g = y_bar*(1-y_bar)*(y(k)-y_bar)
        
        % gradient of hidden layer
        e(1) = b(1)*(1-b(1))*W(1)*g
        e(2) = b(2)*(1-b(2))*W(2)*g
        
        % update weights and threshold
            % update weights 
        W(1) = W(1) + delta*g*b(1)
        W(2) = W(2) + delta*g*b(2)
        V(1,1) = V(1,1) + delta*e(1)*x(k,1)
        V(2,1) = V(2,1) + delta*e(1)*x(k,2)
        V(1,2) = V(1,2) + delta*e(2)*x(k,1)
        V(2,2) = V(2,2) + delta*e(2)*x(k,2)
            % update threshold
        theta = theta - delta*g
        gamma(1) = gamma(1) - delta*e(1)
        gamma(2) = gamma(2) - delta*e(2)
    end
    
    % update diff
    diff = max(max(e,g))
end