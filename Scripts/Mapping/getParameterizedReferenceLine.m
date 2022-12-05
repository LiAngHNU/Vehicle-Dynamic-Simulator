%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reference Line Parameterizer %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Version: v2.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date: 2022/11/09 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Parameterizer Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set Steps
nI = 5;
% Set Dimensions
nD = length(out.T.Data);
% Set Fit Methods               [diff, spline, pchip, makima]
Method = 'diff';                     
% Set Track Width
Wl = 2.0;
Wr = 2.0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Matrices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Id = zeros(nD,1);
T = zeros(nD,1);
S = zeros(nD,1);
X = zeros(nD,1);
Y = zeros(nD,1);
Sl = zeros(nD,1);
Xl = zeros(nD,1);
Yl = zeros(nD,1);
Sr = zeros(nD,1);
Xr = zeros(nD,1);
Yr = zeros(nD,1);
Bnk = zeros(nD,1);
Slp = zeros(nD,1);
Tht = zeros(nD,1);
Kpa = zeros(nD,1);
dX = zeros(nD,1);
dY = zeros(nD,1);
dBnk = zeros(nD,1);
dSlp = zeros(nD,1);
dTht = zeros(nD,1);
dKpa = zeros(nD,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Load Sampled Data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T = out.T.Data;
S = out.S.Data;
X = out.X.Data;
Y = out.Y.Data;
Bnk = out.Bnk.Data;
Slp = out.Slp.Data;
Kpa = out.Kpa.Data;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Parameterize Reference Line %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% First: Calculate dX, dY, ddX, ddY
% Second: Calculate Bank, Slope, Theta, Kappa
% Third: Calculate dBank, dSlope, dTheta, dKappa
% Update ID
for i = 1:1:nD
Id(i,1) = i;
end
% 
switch Method
% Method: diff
case 'diff'
dX = [0;diff(X)];
dY = [0;diff(Y)];
for i = 1:1:nD
Tht(i,1) = atan(dY(i)/dX(i));
Tht(1,1) = Tht(2,1);
end
dBnk = [0;diff(Bnk)];
dSlp = [0;diff(Slp)];
dKpa = [0;diff(Kpa)];
% Method: spline
% Method: pchip
% Method: makima
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Data Legality Check %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Theta Legality Check
for j = 1:1:nI
    for i = 2:1:nD
        if     Tht(i) - Tht(i-1) < -0.9*pi
            Tht(i) = Tht(i) + pi;
        elseif Tht(i) - Tht(i-1) > +0.9*pi
         Tht(i) = Tht(i) - pi;
        end
    end
    j = j + 1;
end
dTht = [0;diff(Tht)];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate Boundaries %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Xl = X - Wl*sin(Tht);
Yl = Y + Wl*cos(Tht);
Xr = X + Wr*sin(Tht);
Yr = Y - Wr*cos(Tht);
for i = 2:1:nD
    Sl(i) = sqrt((Xl(i)-Xl(i-1))^2+(Yl(i)-Yl(i-1))^2) + Sl(i-1);
    Sr(i) = sqrt((Xr(i)-Xr(i-1))^2+(Yr(i)-Yr(i-1))^2) + Sr(i-1);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Save RefLine %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
RefLineInfo = table(Id, T, S, X, Y, Bnk, Slp, Tht, Kpa, dX, dY,...
                    dBnk, dSlp, dTht, dKpa, Sl, Xl, Yl, Sr, Xr, Yr);
RefLineInfo.Id = Id;
RefLineInfo.T = T;
RefLineInfo.S = S;
RefLineInfo.X = X;
RefLineInfo.Y = Y;
RefLineInfo.Bnk = Bnk;
RefLineInfo.Slp = Slp;
RefLineInfo.Tht = Tht;
RefLineInfo.Kpa = Kpa;
RefLineInfo.dX = dX;
RefLineInfo.dY = dY;
RefLineInfo.dBnk = dBnk;
RefLineInfo.dSlp = dSlp;
RefLineInfo.dTht = dTht;
RefLineInfo.dKpa = dKpa;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize RefLine Info %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,3,1); hold on; axis equal;
plot(RefLineInfo.X,RefLineInfo.Y,'k');
plot(RefLineInfo.Xl,RefLineInfo.Yl,'r');
plot(RefLineInfo.Xr,RefLineInfo.Yr,'r');
legend('center', 'left', 'right');
subplot(2,3,2); hold on;
plot(RefLineInfo.S,RefLineInfo.Bnk);
plot(RefLineInfo.S,RefLineInfo.dBnk);
legend('bank', 'dbank');
subplot(2,3,3); hold on;
plot(RefLineInfo.S,RefLineInfo.Slp);
plot(RefLineInfo.S,RefLineInfo.dSlp);
legend('slope', 'dslope');
subplot(2,3,4); hold on;
plot(RefLineInfo.S,RefLineInfo.Tht);
plot(RefLineInfo.S,RefLineInfo.dTht);
legend('theta', 'dtheta');
subplot(2,3,5); hold on;
plot(RefLineInfo.S,RefLineInfo.Kpa);
plot(RefLineInfo.S,RefLineInfo.dKpa);
legend('kappa', 'dkappa');
subplot(2,3,6); hold on; grid on; view(45,45);
plot3(RefLineInfo.X,RefLineInfo.Y,zeros(nD,1),'k');
plot3(RefLineInfo.Xl,RefLineInfo.Yl,zeros(nD,1),'r');
plot3(RefLineInfo.Xr,RefLineInfo.Yr,zeros(nD,1),'r');
plot3(RefLineInfo.X,RefLineInfo.Y,RefLineInfo.S,'k');