%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reference Line Parameterizer (Coordinates Only) %%%%%%%%%%%%%%%%%%%%%%%%%
% Version: v1.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date: 2023/02/15 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Parameterizer Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set Legality
nI = 5;
% Set Resolution
dS = 0.50;
% Set Scale
ScaleFactor = 0.50;
% Set Track
Track = "BrandsHatch";
% Load Track
RefLineRaw = readtable(strcat("Scenarios\Racetracks\",Track,".csv"));
X_Raw = ScaleFactor*[RefLineRaw.x_X_m; RefLineRaw.x_X_m(1,1);];
Y_Raw = ScaleFactor*[RefLineRaw.y_m; RefLineRaw.y_m(1,1);];
Wl_Raw = ScaleFactor*[RefLineRaw.w_tr_left_m; RefLineRaw.w_tr_left_m(1,1);];
Wr_Raw = ScaleFactor*[RefLineRaw.w_tr_right_m; RefLineRaw.w_tr_right_m(1,1);];
nD_Raw = length(X_Raw);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Resample %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate Raw Station
S_Raw = zeros(nD_Raw,1);
for i = 2:1:nD_Raw
    S_Raw(i,1) = S_Raw(i-1) + sqrt((X_Raw(i)-X_Raw(i-1))^2 + (Y_Raw(i)-Y_Raw(i-1))^2);
end
% Resample X, Y, Wl & Wr
S = [0:dS:S_Raw(nD_Raw,1)]';
X = interp1(S_Raw,X_Raw,S,'spline');
Y = interp1(S_Raw,Y_Raw,S,'spline');
Wl = interp1(S_Raw,Wl_Raw,S,'spline');
Wr = interp1(S_Raw,Wr_Raw,S,'spline');
nD = length(S);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Matrices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Id = zeros(nD,1);
Tht = zeros(nD,1);
Kpa = zeros(nD,1);
dX = zeros(nD,1);
ddX = zeros(nD,1);
dY = zeros(nD,1);
ddY = zeros(nD,1);
dTht = zeros(nD,1);
dKpa = zeros(nD,1);
Sl = zeros(nD,1);
Xl = zeros(nD,1);
Yl = zeros(nD,1);
Sr = zeros(nD,1);
Xr = zeros(nD,1);
Yr = zeros(nD,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Parameterize Reference Line %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% First: Calculate dX, dY, ddX, ddY
% Second: Calculate Bank, Slope, Theta, Kappa
% Third: Calculate dBank, dSlope, dTheta, dKappa
% Calculate ID
for i = 1:1:nD
Id(i,1) = i;
end
% Calculate S, X, Y
% Calculate dX & dY
coefs_X = spline(S,X);
coefs_Y = spline(S,Y);
for i = 1:1:nD-1
    dX(i,1) = coefs_X.coefs(i,3);
    ddX(i,1) = 2*coefs_X.coefs(i,2);
    dY(i,1) = coefs_Y.coefs(i,3);
    ddY(i,1) = 2*coefs_Y.coefs(i,2);
end
dX(nD,1) = 3*coefs_X.coefs(nD-1,1)*(S(nD)-S(nD-1))^2 + ...
           2*coefs_X.coefs(nD-1,2)*(S(nD)-S(nD-1)) + ...
           1*coefs_X.coefs(nD-1,3);
ddX(nD,1) = 6*coefs_X.coefs(nD-1,1)*(S(nD)-S(nD-1)) + ...
            2*coefs_X.coefs(nD-1,2);
dY(nD,1) = 3*coefs_Y.coefs(nD-1,1)*(S(nD)-S(nD-1))^2 + ...
           2*coefs_Y.coefs(nD-1,2)*(S(nD)-S(nD-1)) + ...
           1*coefs_Y.coefs(nD-1,3);
ddY(nD,1) = 6*coefs_Y.coefs(nD-1,1)*(S(nD)-S(nD-1)) + ...
            2*coefs_Y.coefs(nD-1,2);
% Calculate Tht
for i = 1:1:nD
    Tht(i,1) = atan(dY(i,1)/dX(i,1));
end
% Calculate Kpa
for i = 1:1:nD
    Kpa(i,1) = (dX(i,1)*ddY(i,1) - ddX(i,1)*dY(i,1))/(dX(i,1)^2 + dY(i,1)^2);
end
dKpa = [0;diff(Kpa)];
% Kpa = smooth(Kpa,50);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Data Legality Check %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Heading Angle Correction
for j = 1:1:nI
    for i = 2:1:nD
        if     Tht(i) - Tht(i-1) < -0.95*pi
            Tht(i) = Tht(i) + pi;
        elseif Tht(i) - Tht(i-1) > +0.95*pi
         Tht(i) = Tht(i) - pi;
        end
    end
    j = j + 1;
end
% Initial Heading Angle Correction
if Tht(1) < 0
    Tht = Tht + pi;
end
dTht = [0;diff(Tht)];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate Boundaries %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:nD
    Xl(i) = X(i) - Wl(i)*sin(Tht(i));
    Yl(i) = Y(i) + Wl(i)*cos(Tht(i));
    Xr(i) = X(i) + Wr(i)*sin(Tht(i));
    Yr(i) = Y(i) - Wr(i)*cos(Tht(i));
end
for i = 2:1:nD
    Sl(i) = sqrt((Xl(i)-Xl(i-1))^2+(Yl(i)-Yl(i-1))^2) + Sl(i-1);
    Sr(i) = sqrt((Xr(i)-Xr(i-1))^2+(Yr(i)-Yr(i-1))^2) + Sr(i-1);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Save RefLine %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
RefLineInfo = table(Id, S, X, Y, Tht, Kpa, ...
                    dX, ddX, dY, ddY, dTht, dKpa, ...
                    Sl, Wl, Xl, Yl, Sr, Wr, Xr, Yr);
RefLineInfo.Id = Id;
RefLineInfo.S = S;
RefLineInfo.X = X;
RefLineInfo.Y = Y;
RefLineInfo.Tht = Tht;
RefLineInfo.Kpa = Kpa;
RefLineInfo.dX = dX;
RefLineInfo.dY = dY;
RefLineInfo.dTht = dTht;
RefLineInfo.dKpa = dKpa;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize RefLine Info %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,2,1); hold on; axis equal;
plot(RefLineInfo.X,RefLineInfo.Y,'k');
plot(RefLineInfo.Xl,RefLineInfo.Yl,'r');
plot(RefLineInfo.Xr,RefLineInfo.Yr,'r');
legend('center', 'left', 'right');
subplot(2,2,2); hold on;
plot(RefLineInfo.S,RefLineInfo.Tht);
plot(RefLineInfo.S,RefLineInfo.dTht);
legend('theta', 'dtheta');
subplot(2,2,3); hold on;
plot(RefLineInfo.S,RefLineInfo.Kpa);
plot(RefLineInfo.S,RefLineInfo.dKpa);
legend('kappa', 'dkappa');
subplot(2,2,4); hold on; grid on; view(45,45);
plot3(RefLineInfo.X,RefLineInfo.Y,zeros(nD,1),'k');
plot3(RefLineInfo.Xl,RefLineInfo.Yl,zeros(nD,1),'r');
plot3(RefLineInfo.Xr,RefLineInfo.Yr,zeros(nD,1),'r');
plot3(RefLineInfo.X,RefLineInfo.Y,RefLineInfo.S,'k');
save(strcat("Scenarios\RaceTracks\RaceTrack_",Track,".mat"), "RefLineInfo");