%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hunan University
% Published by: Ang Li
% Email: ang1997@hnu.edu.cn
% Copyright (c) 2022 . All rights reserved.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  TS   = 0.02; 
Index  = zeros(length(out.S.Data), 1);
     S = out.S.Data;
     X = out.X.Data;
     Y = out.Y.Data;
 Theta = zeros(length(out.S.Data), 1);
 Kappa = out.Kappa.Data;
dKappa = zeros(length(out.S.Data), 1);

for i = 1:1:length(out.S.Data)
    Index(i,1) = i;
    if i == 1
        Theta(i,1) = 0;
        dKappa(i,1) = 0;
    else
        dx = X(i,1) - X(i-1,1);
        dy = Y(i,1) - Y(i-1,1);
        Theta(i,1) = atan(dy/dx);
        dKappa(i,1) = Kappa(i,1) - Kappa(i-1,1);
    end
end

subplot(2,2,1); plot(X,Y);
subplot(2,2,2); plot(Index,Theta);
subplot(2,2,3); plot(Index,Kappa);
subplot(2,2,4); plot(Index,dKappa);

%%

Path = table(Index, S, X, Y, Theta, Kappa, dKappa);
k = 0;
for i = 2:1:length(Index)
    dTheta = Theta(i,1) - Theta(i-1,1);
    if abs(dTheta) > 1.57
    k = k + 1;
    Flip_Flag(k,1) = i;
    end
end

%%
for i = 534:1:1437
    Path.Theta(i,1) = Path.Theta(i,1) + pi;
end

%%
subplot(2,2,1); plot(Path.X,Path.Y);
subplot(2,2,2); plot(Path.Index,Path.Theta);
subplot(2,2,3); plot(Path.Index,Path.Kappa);
subplot(2,2,4); plot(Path.Index,Path.dKappa);