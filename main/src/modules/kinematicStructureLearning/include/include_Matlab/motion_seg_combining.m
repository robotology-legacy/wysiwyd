rank_W_ms = zeros(c,c);
rank_tol = 10;

% conversion_check = 1;
%
% while(conversion_check)
%
%     for i=1:c
%         for j=i+1:c
%             W_ms_buf = W(:,[find(final==i);find(final==j)]);
%             rank_W_ms(i,j) = rank(W_ms_buf,rank_tol);
%         end
%     end
%
%     rank_W_values = nonzeros(rank_W_ms);
%
%     min_value = min(rank_W_values);
%     max_value = max(rank_W_values);
%
%     [rank_tol, min_value, max_value]
%
%     if min_value >= 2 && max_value <= 10
%         conversion_check = 0;
%     else
%         if max_value > 10
%             rank_tol = rank_tol + 1;
%         else min_value < 2
%             rank_tol = rank_tol - 1;
%         end
%     end
% end

for i=1:c
    for j=i+1:c
        W_ms_buf = W(:,[find(final==i);find(final==j)]);
        rank_W_ms(i,j) = rank(W_ms_buf,rank_tol);
    end
end

full_ST = full(ST);
full_ST(find(full_ST ~= 0)) = 1;
full_ST = full_ST' + full_ST;

rank_connect_mtx = rank_W_ms .* full_ST;

rank_connect_mtx(find(rank_connect_mtx > 5)) = 0;

rank_connect_mtx(rank_connect_mtx~=0) = 1;

g = sparse(rank_connect_mtx);
G = g + g';
[new_cluster_num new_cluster_idx] = graphconncomp( G ); % find connected components
new_cluster_num




final_buf = final;

for i=1:c
    final_buf(find(final == i)) = new_cluster_idx(i);
end

final = final_buf;