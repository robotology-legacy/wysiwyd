function final = my_motion_segmentation(y, T, T_i, T_c, T_r, alpha, lambda, c)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% T_i : the number of iteration     % 150
% T : the number of trial
% T_c : convergence test, noise free : 15, noise : 5
% T_r : reinitialization at T_r
% alpha : decay parameter
% lambda : voting strength
% c : the number of group
% s : ground truth


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% To do : modify G and h including t index
% converge

n = size(y,2);
f = size(y,3);

Gt = cell(c,T);
ht = zeros(n,c,T);

idx_set = cell(T,1);
g = ones(T,1);
ht_buf = cell(T,1);
k_buf = cell(T,1);
k = zeros(T,1);
Gt_not_changed_counter = cell(T,1);
Gt_buf = cell(T,1);
Gt_buf_2 = cell(T,1);
Gt_changed_flag = cell(T,1);
Gt_diff = cell(T,1);

n_c_buf = round(n/c);
 

% t = 0; % initialize the trial number

parfor t=1:T
    fprintf('trial #%d\n',t);
    
    % randomly initialize G^t_g and h
    idx_set{t} = randperm(n)';
%     g(t) = 1;
    while(1)
        Gt_buf{t}{g(t)} = idx_set{t}(1:n_c_buf);
        idx_set{t} = idx_set{t}(n_c_buf+1:end);
        if size(idx_set{t},1) <= n_c_buf
            Gt_buf{t}{g(t)+1} = idx_set{t};
            break;
        else
            g(t) = g(t) + 1;
        end
    end
    
    ht_buf{t} = zeros(n,c);
    
    % initialize the voting number
%     k(t) = 0;
%     Gt_not_changed_counter{t} = 0;
%     
%     k_buf{t} = 1;
    
    while(k(t) < T_i)
        % randomized voting
        [Gt_buf{t}, ht_buf{t}] = my_randomized_voting(y, Gt_buf{t}, ht_buf{t}, alpha, lambda);
        %         [Gt_buf_2, ht_buf] = my_randomized_voting_mod(y, Gt_buf, ht_buf, alpha, lambda);
        
        % reinitialization at T_r
        if k(t) == T_r
            ht_buf{t} = zeros(n,c);
        end
        
        % check whether Gt is not changed over T_c
%         Gt_changed_flag{t} = 0;
%         
%         if k{t} > 1
%             for g{t} = 1:c
%                 if size(Gt_buf_2{g{t}},1) ~= size(Gt_buf{t}{g{t}},1)
%                     Gt_changed_flag{t} = 1;
%                 else
%                     Gt_diff{t} = abs(Gt_buf_2{g{t}} - Gt_buf{t}{g{t}});
%                     if sum(Gt_diff{t}) ~= 0
%                         Gt_changed_flag{t} = 1;
%                     end
%                 end
%             end
%             
%             if Gt_changed_flag{t} == 0
%                                 [k k_buf]
%                 if k{t}-k_buf{t} < 3 % counting margin for non continuous convergence
%                     Gt_not_changed_counter{t} = Gt_not_changed_counter{t} + 1;
%                     if Gt_not_changed_counter{t} >= T_c
%                         Gt_converge_flag{t} = true;
%                         fprintf('.....converged!\n')
%                         Gt_buf{t} = Gt_buf_2{t};
%                         break;
%                     end
%                 else
%                     Gt_not_changed_counter{t} = 0;
%                 end
%                                 [t k Gt_not_changed_counter]
%                 k_buf{t} = k{t};
%             end
%         end
        
%         Gt_buf{t} = Gt_buf_2{t};
        
        k(t) = k(t) + 1;
    end
    
    for g2 = 1:c
        Gt{g2,t} = Gt_buf{t}{g2};
    end
    ht(:,:,t) = ht_buf{t};
    
%     if t == T || Gt_converge_flag == true
%         repeat_flag = false;
%     end
end

final = ones(n,1);

% if Gt_converge_flag == true
%     for g = 1:c
%         final(Gt{g,t}) = g;
%     end
% else
    A = affinityMatrix(Gt,ht);
    [final,qual]=spectral_clustering(A,c);
% end

