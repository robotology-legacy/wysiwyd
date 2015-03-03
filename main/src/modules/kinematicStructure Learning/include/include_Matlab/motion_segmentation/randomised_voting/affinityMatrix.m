function A = affinityMatrix(Gt,ht)

[n,c,t] = size(ht);
% Ht = zeros(n,c,t);
% 
% for t_idx = 1:t
%     for n_idx = 1:n
%         for c_idx = 1:c
%             Ht(n_idx, c_idx, t_idx) = ht(n_idx,c_idx,t_idx);
%         end
%     end
% end

V = [];
for t_idx = 1:t
    V = [V,ht(:,:,t_idx)];
end
for t_idx = 1:t
    G_colvec = [];
    for g = 1:c
%         G_colvec = [G_colvec; Gt{g,t_idx}];
        G_colvec = [G_colvec; ones(size(Gt{g,t_idx},1),1)*g];
    end
    V = [V,G_colvec];
end

A = V*V';
end