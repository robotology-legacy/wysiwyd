function [G, h] = my_randomized_voting(y, G, h, alpha, lambda)

[n, c] = size(h);
f = size(y,3);

rfn = randperm(f, 2);
k = rfn(1);
l = rfn(2);

T_p = 0.7;
m = 8;

for g = 1:c
    
    iter = -1;
    
    while(iter<0)
        p = rand(1)    % uniform random variable
        
        G_g_size = size(G{g},1);
        
        if G_g_size >= m
            if p < T_p
                disp('select m points in G_g with the highest votes');                
                % select m points in G_g with the highest votes
                buf= h(G{g},g);
                [sort_value, sort_idx] = sort(buf,'descend');
                m_points_idx_buf = G{g}(sort_idx);
                m_points_idx = m_points_idx_buf(1:m)';
            elseif p>=T_p
                disp('randomly select m points from G_g');
                % randomly select m points from G_g
                m_points_idx = G{g}(randperm(G_g_size, m));
            end
        else
            disp('random permutation');
            m_points_idx = randperm(n,m);
        end       
        
        [m_points_idx]
        [k,l]
        x_k = y(:,[m_points_idx],k)
        x_l = y(:,[m_points_idx],l)
        
        figure(22)
        clf
        subplot(1,2,1)
        plot(y(1,:,k),y(2,:,k),'b.');
        hold on
        plot(y(1,[m_points_idx],k),y(2,[m_points_idx],k),'ro');
        
        subplot(1,2,2)
        plot(y(1,:,l),y(2,:,l),'b.');
        hold on        
        plot(y(1,[m_points_idx],l),y(2,[m_points_idx],l),'ro');        
        
        x_k_unique_value = unique(x_k','rows');
        x_l_unique_value = unique(x_l','rows');
        
        if size(x_k_unique_value,1) ~= 1 && size(x_l_unique_value,1) ~= 1
            iter = 1;
        end
    end
    
    % estimate F_g using selected m points
    [F_g,e1,e2] = fundmatrix(x_k,x_l);
    
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

%%

h = alpha*h;


%%
color_idx = 'rgbcmykrbgcmyk';
figure(100)
clf
hold on
for g = 1:c
    h_buf = [];
    for i = 1:n
        h_buf = [h_buf;[i,h(i,g)]];
    end
    plot(h_buf(:,1),h_buf(:,2),'Color',color_idx(g));
    axis([0 n 0 100]);
end
pause(0.01)