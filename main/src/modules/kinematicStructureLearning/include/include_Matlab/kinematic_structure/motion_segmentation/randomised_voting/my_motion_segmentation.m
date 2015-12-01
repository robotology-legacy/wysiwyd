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

repeat_flag = true;
Gt_converge_flag = false;

Gt = cell(c,T);
ht = zeros(n,c,T);

t = 0; % initialize the trial number

while(repeat_flag)
    t = t + 1;
    fprintf('trial #%d\n',t);
    
    % randomly initialize G^t_g and h
    idx_set = randperm(n)';
    g = 1;
    while(1)
        Gt_buf{g} = idx_set(1:round(n/c));
        idx_set = idx_set(round(n/c)+1:end);
        if size(idx_set,1) <= round(n/c)
            Gt_buf{g+1} = idx_set;
            break;
        else
            g = g + 1;
        end
    end
    
    ht_buf = zeros(n,c);
    
    % initialize the voting number
    k = 0;
    Gt_not_changed_counter = 0;
    cont_check = 0;
    
    k_buf = 1;  
    
    while(k < T_i)
        % randomized voting
        [Gt_buf_2, ht_buf] = my_randomized_voting(y, Gt_buf, ht_buf, alpha, lambda);
%         [Gt_buf_2, ht_buf] = my_randomized_voting_mod(y, Gt_buf, ht_buf, alpha, lambda);
        
        % reinitialization at T_r
        if k == T_r
            ht_buf = zeros(n,c);
        end
        
        % check whether Gt is not changed over T_c
        Gt_changed_flag = 0;

        if k > 1
            for g = 1:c
                if size(Gt_buf_2{g},1) ~= size(Gt_buf{g},1)
                    Gt_changed_flag = 1;
                else
                    Gt_diff = abs(Gt_buf_2{g} - Gt_buf{g});
                    if sum(Gt_diff) ~= 0
                        Gt_changed_flag = 1;
                    end
                end
            end
            
            if Gt_changed_flag == 0
%                 [k k_buf]
                if k-k_buf < 3 % counting margin for non continuous convergion
                    Gt_not_changed_counter = Gt_not_changed_counter + 1;        
                    if Gt_not_changed_counter >= T_c
                        Gt_converge_flag = true;
                        fprintf('.....converged!\n')
                        Gt_buf = Gt_buf_2;
                        break;
                    end
                else
                    Gt_not_changed_counter = 0;
                end
%                 [t k Gt_not_changed_counter]
                k_buf = k;                
            end
        end
        
        Gt_buf = Gt_buf_2;
        
        k = k + 1;
    end
    
    for g = 1:c
        Gt{g,t} = Gt_buf{g};
    end
    ht(:,:,t) = ht_buf;
    
    if t == T || Gt_converge_flag == true
        repeat_flag = false;
    end
end

final = ones(n,1);

if Gt_converge_flag == true
    for g = 1:c
        final(Gt{g,t}) = g;
    end
else
    A = affinityMatrix(Gt,ht);
    [final,qual]=spectral_clustering(A,c);
end

