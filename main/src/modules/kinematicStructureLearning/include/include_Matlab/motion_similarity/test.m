

dist_P = movingsticks(90, 120, -270, -300, 0, 0, 100);

norm_buf = [];
for theta_step = -90:90:540
    120-theta_step
dist_Q = movingsticks(95, 120-theta_step, -270, -300, 0, 0, 100);
% dist_S = movingsticks(-90, -110, -275, -305, 5, 30, 100);

% %%
% d = 3;
% figure(1)
% clf
% 
% for d=1:3
%     subplot(3,1,1+(d-1))
%     plot(dist_P(:,d),'b-')
%     hold on
%     plot(dist_Q(:,d),'r-')    
% %     subplot(3,1,2+(d-1))
% %     plot(dist_Q(:,d))    
%     p(d) = ranksum(dist_P(:,d), dist_Q(:,d));
% end

%%
min_P_buf = min(dist_P,[],1) ./ cal_logm(rotz(0), rotz(pi));
max_P_buf = max(dist_P,[],1) ./ cal_logm(rotz(0), rotz(pi));
min_Q_buf = min(dist_Q,[],1) ./ cal_logm(rotz(0), rotz(pi));
max_Q_buf = max(dist_Q,[],1) ./ cal_logm(rotz(0), rotz(pi));
min_S_buf = min(dist_S,[],1) ./ cal_logm(rotz(0), rotz(pi));
max_S_buf = max(dist_S,[],1) ./ cal_logm(rotz(0), rotz(pi));

minmax_P = reshape([min_P_buf;max_P_buf],[1,6]);
minmax_Q = reshape([min_Q_buf;max_Q_buf],[1,6]);
minmax_S = reshape([min_S_buf;max_S_buf],[1,6]);

norm_compare     = [norm((minmax_P - minmax_Q)),  norm((minmax_Q - minmax_S)),  norm((minmax_S - minmax_P))] ./ norm([1 1 1 1 1 1]);
ranksum_compare  = [ranksum(minmax_P, minmax_Q),  ranksum(minmax_Q, minmax_S),  ranksum(minmax_S, minmax_P)] ./ norm([1 1 1 1 1 1]);
signrank_compare = [signrank(minmax_P, minmax_Q), signrank(minmax_Q, minmax_S), signrank(minmax_S, minmax_P)] ./ norm([1 1 1 1 1 1]);

norm_buf = [norm_buf;sum(norm_compare)];
end

figure
plot(norm_buf)