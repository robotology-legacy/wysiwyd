function [G, h] = my_randomized_voting(y, G, h, alpha, lambda)

[n, c] = size(h);
f = size(y,3);

rfn = randperm(f, 2);
k = rfn(1);
l = rfn(2);

T_p = 0.99;
m = 8;

for g = 1:c
    p = rand(1);    % uniform random variable
    
    G_g_size = size(G{g},1);
    
    if G_g_size >= m
        if p < T_p
            % select m points in G_g with the highest votes
            buf= h(G{g},g);
            [sort_value, sort_idx] = sort(buf,'descend');
            m_points_idx_buf = G{g}(sort_idx);
            m_points_idx = m_points_idx_buf(1:m)';
        elseif p>=T_p
            % randomly select m points from G_g
            m_points_idx = G{g}(randperm(G_g_size, m));
        end
    else
%         m_points_idx = G{g};
%         buf = setdiff([1:n],m_points_idx);
%         m_points_idx = [m_points_idx ; buf(randperm(size(buf,2),m-G_g_size))'];
        m_points_idx = randperm(n,m);
    end
    
    
    x_k = y(:,[m_points_idx],k);
    x_l = y(:,[m_points_idx],l);
    
    % estimate F_g using selected m points
    [F_g,e1,e2] = fundmatrix(x_k,x_l);
    
%     for i = 1:n
%         x_k_i = y(:,i,k);
%         x_l_i = y(:,i,l);
%         
%         di = SampSonDist(F_g, x_k_i, x_l_i);
%         
%         for g_idx = 1:c
%             if g_idx == g
%                 h(i,g_idx) = h(i,g_idx) + exp(-lambda * di);
%             else
%                 h(i,g_idx) = h(i,g_idx) - exp(-lambda * di) + 1;
%             end
%         end
%     end

    x_k = y(:,:,k);
    x_l = y(:,:,l);
    
    d = SampSonDist(F_g, x_k, x_l);    
    exp_buf = exp(-lambda*d)';
    
    for g_idx = 1:c
        if g_idx == g
            h(:,g_idx) = h(:,g_idx) + exp_buf;
        else
            h(:,g_idx) = h(:,g_idx) - exp_buf + 1;
        end
    end

end

%update
[max_h_value,max_g_idx] = max(h,[],2);

for g = 1:c
    G{g} = find(max_g_idx==g);
end

% %%
% color_idx = 'rgbcmykrbgcmyk';
% figure(100)
% % for f=1:size(y,3)
% for f=1
%     for p=1:size(y,2)
%         plot(y(1,p,f), y(2,p,f),'.','Color',color_idx(max_g_idx(p)));
% %         axis([0, width, 0, height]);
%         hold on
%     end
%     pause(0.01);
%     clf
% end
%%

h = alpha*h;


%%
% color_idx = 'rgbcmykrbgcmyk';
% figure(100)
% clf
% hold on
% for g = 1:c
%     h_buf = [];
%     for i = 1:n
%         h_buf = [h_buf;[i,h(i,g)]];
%     end    
%     plot(h_buf(:,1),h_buf(:,2),'Color',color_idx(g));    
%     axis([0 n 0 100]);
% end
% pause(0.01)