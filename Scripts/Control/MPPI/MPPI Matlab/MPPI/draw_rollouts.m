function a = draw_rollouts(x_hrz,K,S)


% K: number of rollouts
hold on;
max_s = max(S);
min_s = min(S);
for kth_rollout=1:K
%     plot(x_hrz(1,:,kth_rollout),x_hrz(2,:,kth_rollout),'Color',[0.3010, 0.7450, 0.9330, 0.05]);
    col = (S(kth_rollout)-min_s)/(max_s-min_s);
    a(kth_rollout) = plot(x_hrz(1,:,kth_rollout),x_hrz(2,:,kth_rollout),'Color',[col, 1-col, 0.9330, 0.1]);
%     disp(col);
end
end