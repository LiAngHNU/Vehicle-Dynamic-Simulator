%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hunan University
% Published by: Ang Li
% Email: ang1997@hnu.edu.cn
% Copyright (c) 2022 . All rights reserved.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load Data
load('Data\ILCLQR_vs_ILQR\ILQR_at_9.mat');
load('Data\ILCLQR_vs_ILQR\ILCLQR_at_9.mat');

% RMS Error & Max Error
ILQR_ErrY_RMS = rms(ILQR.ErrY.Data);
ILCLQR_ErrY_RMS = rms(ILCLQR.ErrY.Data);
ILQR_ErrVy_RMS = rms(ILQR.ErrVy.Data);
ILCLQR_ErrVy_RMS = rms(ILCLQR.ErrVy.Data);
ILQR_ErrYz_RMS = rms(ILQR.ErrYz.Data);
ILCLQR_ErrYz_RMS = rms(ILCLQR.ErrYz.Data);
ILQR_ErrAVz_RMS = rms(ILQR.ErrAVz.Data);
ILCLQR_ErrAVz_RMS = rms(ILCLQR.ErrAVz.Data);

% Data Visualization
subplot(3,2,1);
hold on;
plot(ILQR.Station.Data, ILQR.SteerSW.Data);
plot(ILCLQR.Station.Data, ILCLQR.SteerSW.Data);
title('Delta Steer SW');

subplot(3,2,2);
hold on;
plot(ILQR.Station.Data, ILQR.ErrY_Acc.Data);
plot(ILCLQR.Station.Data, ILCLQR.ErrY_Acc.Data);
title('Veh ErrY Acc');

subplot(3,2,3);
hold on;
plot(ILQR.Station.Data, ILQR.ErrY.Data);
plot(ILCLQR.Station.Data, ILCLQR.ErrY.Data);
title('Veh ErrY');

subplot(3,2,4);
hold on;
plot(ILQR.Station.Data, ILQR.ErrVy.Data);
plot(ILCLQR.Station.Data, ILCLQR.ErrVy.Data);
title('Veh ErrVy');

subplot(3,2,5);
hold on;
plot(ILQR.Station.Data, ILQR.ErrYz.Data);
plot(ILCLQR.Station.Data, ILCLQR.ErrYz.Data);
title('Veh ErrYz');

subplot(3,2,6);
hold on;
plot(ILQR.Station.Data, ILQR.ErrAVz.Data);
plot(ILCLQR.Station.Data, ILCLQR.ErrAVz.Data);
title('Veh ErrAVz');