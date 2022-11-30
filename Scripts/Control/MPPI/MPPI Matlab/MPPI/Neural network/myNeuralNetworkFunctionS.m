function [y1] = myNeuralNetworkFunctionS(x1)
%MYNEURALNETWORKFUNCTION neural network simulation function.
%
% Auto-generated by MATLAB, 09-Sep-2022 09:50:53.
%
% [y1] = myNeuralNetworkFunction(x1) takes these arguments:
%   x = 3xQ matrix, input #1
% and returns:
%   y = 3xQ matrix, output #1
% where Q is the number of samples.

%#ok<*RPMT0>

% ===== NEURAL NETWORK CONSTANTS =====

% Input 1
x1_step1.keep = [1 2];
x1_step2.xoffset = [-0.0355410933003255;-0.462770104591699];
x1_step2.gain = [0.624115168690869;2.17690137965392];
x1_step2.ymin = -1;

% Layer 1
b1 = [0.31441697235947346778;-1.3390556433806708458;-1.3456753422971223788;-0.061479757134840641586;0.87059871940409361279;0.85614912551757293979;-0.51284477028243824481;1.2724621948557277573;-0.68760674797308174444;1.1511186858937749733];
IW1_1 = [-0.035350709149698150702 0.13123703441492004518;-0.01290874553556978048 0.53443880014390043875;-0.0078425991276096281707 -0.55255540549661474881;0.7683248895209807694 -0.00055913672264707185624;-0.27922375557543793745 -0.11126163749729073293;-1.0142082734289534773 1.531344255082473245e-05;-0.22186162291654898393 -0.089140007217475891732;-0.75236595097818270705 0.0056698057594723365663;-0.85170094058441847551 -7.347298870648245276e-05;0.74587904452137987477 0.0037447389990982495045];

% Layer 2
b2 = [0.62808751882863700366;-4.6618181086209036224;-0.073552890525698078061];
LW2_1 = [-1.347157659594278023 -0.095011590904448384176 -0.044711606957582306077 -3.2099161152634625438 -1.3730263510694642459 0.28701520599805474543 -1.2631847577411539074 -2.3633744343178486602 1.163580122622089652 2.8703530900768505774;0.56086394516170356983 0.036308632494211193209 0.015089047742114742451 -1.2885011383944500185 0.51622997434749340329 2.5865539616948693968 0.61313193525609299694 0.72199600728166357655 -4.8938242120985151473 0.063944293208405236784;1.1047351403607428288 2.4218314606928919019 -2.2528097687888442024 -0.25263190032669058382 -1.354831614385145544 0.039704604723601634453 -1.5782173934872016918 0.43166602210578991494 0.069502692895094836389 -0.34493486240813509003];

% Output 1
y1_step1.ymin = -1;
y1_step1.gain = [0.100000000090987;0.193137676807376;0.808620427233298];
y1_step1.xoffset = [-9.99999998180261;-0.355336113698002;-1.24725900063751];

% ===== SIMULATION ========

% Dimensions
Q = size(x1,2); % samples

% Input 1
xp1 = removeconstantrows_apply(x1,x1_step1);
xp1 = mapminmax_apply(xp1,x1_step2);

% Layer 1
a1 = tansig_apply(repmat(b1,1,Q) + IW1_1*xp1);

% Layer 2
a2 = repmat(b2,1,Q) + LW2_1*a1;

% Output 1
y1 = mapminmax_reverse(a2,y1_step1);
end

% ===== MODULE FUNCTIONS ========

% Map Minimum and Maximum Input Processing Function
function y = mapminmax_apply(x,settings)
y = bsxfun(@minus,x,settings.xoffset);
y = bsxfun(@times,y,settings.gain);
y = bsxfun(@plus,y,settings.ymin);
end

% Remove Constants Input Processing Function
function y = removeconstantrows_apply(x,settings)
y = x(settings.keep,:);
end

% Sigmoid Symmetric Transfer Function
function a = tansig_apply(n,~)
a = 2 ./ (1 + exp(-2*n)) - 1;
end

% Map Minimum and Maximum Output Reverse-Processing Function
function x = mapminmax_reverse(y,settings)
x = bsxfun(@minus,y,settings.ymin);
x = bsxfun(@rdivide,x,settings.gain);
x = bsxfun(@plus,x,settings.xoffset);
end
