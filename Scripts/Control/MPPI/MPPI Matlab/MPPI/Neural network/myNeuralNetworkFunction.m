function [y1] = myNeuralNetworkFunction(x1)
%MYNEURALNETWORKFUNCTION neural network simulation function.
%
% Auto-generated by MATLAB, 02-Sep-2022 15:57:08.
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
x1_step2.xoffset = [-0.411707077609202;-1.31251109668235];
x1_step2.gain = [3.15307056445931;0.794562585520539];
x1_step2.ymin = -1;

% Layer 1
b1 = [-3.0151057009093773509;2.1474063739787849769;-0.048494688529111130981;-2.0397105878042225413;-0.046253572111700898273;0.81520295658458119537;-0.19728216124233799444;0.24243923997811320237;1.3202483077760613916;-0.39985635411271486239;7.3186785873262758528;-0.47168052786436187596;-0.95931730512429458368;0.3616084341659291379;3.4139470125549058466;-1.1074423221975850939;-1.0471527406448617459;1.2550741531345621649;0.64342890137560060371;-1.0852086909118883185;-0.2968635868956334134;0.32901376704046125798;0.36076011312189332214;0.41289300319676380813;-0.39305078095674295113;0.77447252122128373841;4.1645263757423869322;0.30731832889302357348;-2.6744271136744925954;0.67139486483300025998];
IW1_1 = [0.071278450889915273558 2.119798634061040854;0.043748531793495346043 -1.0487988256351183924;-0.028775095456221574941 0.15414736268547155063;0.066726454938436638442 0.55260416296562597793;-0.67567815260720409665 -0.0062292083443295743692;0.025629979228782660133 0.06967537694564052142;0.10930221368637717261 0.24476558108166743266;-0.064757352781300264732 0.21296057855625369348;-0.75973157920645406627 0.0016461455826638313627;0.033894272866759403429 -0.070761853160953500086;0.0016848183090138208612 5.7367759980844308387;-0.022984350080014478296 -0.04388743535697718634;0.60635640754069575209 0.0045196816966248621814;-0.10097756787949724311 -0.042509453166354668752;-0.080034968087668204029 -2.8795948934149446785;-0.043874727616560901811 0.087273545697042639868;-0.65717431478130405864 -0.0039380469945284142172;0.691677382172843358 -0.0037332846397720945847;0.0019152429662602661869 0.070970550428152534561;-0.66047065470901300532 -0.0054607992154872527293;0.08709336445772541957 0.0041184255749056406654;-0.047848738931750116332 0.22940472742101589154;-0.024388032153520582124 0.0057067077162903101506;-0.017481470174688793634 0.67092124081036419447;0.032599030289935861104 -0.022375542279976491644;0.004650815138946651911 0.11486081089625892193;-0.048527497580331931115 -3.0685469396504703177;-0.067399092816536179673 0.056422361098120821232;-9.9252177110415273193e-06 -2.0668349109290273802;0.020369233098574818924 0.04893946634449820865];

% Layer 2
b2 = [-2.7651678885795152318;0.49044168784379382764;0.55135421284898644689];
LW2_1 = [0.34354565367390782482 -0.16886366290198032147 0.17392526768636309975 0.60269383392140696287 -0.3859244914660533432 -0.84562144608528611034 0.066095175887047677432 -0.062585420808053696118 2.8314765258692062844 0.27416292566298539057 0.019678466701217308538 0.53322961099689047959 -1.288688914423066656 -0.33278012771499299038 0.36743807246812670853 0.45093468564711941537 -1.2587405222258951643 2.4313177602186728521 -0.62965317827554323493 -1.8782935850270077172 0.54993787040153307988 -0.33678567125564862206 -0.43258631721158352645 0.28499649021290734963 0.43175370891359549086 -0.76492329707438777575 -0.44679078636164415439 -0.20250004864348042366 -0.021860237087225356523 -0.66054859754644923431;0.013428444170570121713 0.0033497432631078472194 0.080936010509381622891 -0.046224737097549782605 -0.58997797270841823014 0.14836124243573620141 0.0083889658177007074558 -0.008200413623927805401 0.18017537338502540023 0.1484369371561651274 -0.00099131691436682087271 -0.032558022797472878362 1.2792882389113187092 -0.27701311576680986981 0.0053097826017958075009 -0.22978386558243571458 -0.38258002131368840093 -0.41392194884194821736 -0.15812406627811778637 -0.81978837216965505519 0.46102487197279856668 0.059056995930089242752 -0.19263080813714728023 -0.0017242093143327215116 0.21849751311851117741 -0.21138629151505219927 -0.0016818368379752687402 -0.17372116525826095113 -0.00094185470473745925169 0.049547148990227343757;1.6620405530398809635 -0.98587971951731501186 0.37909920979729094803 0.78045818413095413568 -0.011597291952147139801 -0.30913166315385093297 0.44136902961345425567 0.44244335053000416469 -0.0027453142864964997019 -0.58403041762642726731 4.2881114235369164334 0.31405065694665806886 0.016798006759668408783 0.39693283322236089239 1.4218865140694179061 0.56471262565000701628 0.43195385792333579333 -0.10678998631432284894 0.48718057920506879777 -0.55982179549218469283 -0.25225669746757034551 -2.0579942846077012852 -0.3215423025140917157 0.58083445593651217287 0.41149868944503403823 0.58467078754945334129 -3.4245838764427904977 0.53260639331953518738 -1.1468401849956293415 -0.33383885819621078594];

% Output 1
y1_step1.ymin = -1;
y1_step1.gain = [2.39346191769883;0.322094424618157;0.125140970431458];
y1_step1.xoffset = [9.16439029791505;-4.00174342849233;-9.46302061784023];

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
