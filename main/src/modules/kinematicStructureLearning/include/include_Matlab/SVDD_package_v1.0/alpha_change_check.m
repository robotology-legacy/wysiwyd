function dist_mtx_idx = alpha_change_check(dist_mtx)

n = size(dist_mtx,1);
dist_mtx_idx = zeros(n,n);

for i=1:n
    for j=2:n
        check = dist_mtx(i,j)-dist_mtx(i,j-1);
        if check > 0
            dist_mtx_idx(i,j) = 1;
        elseif check < 0
            dist_mtx_idx(i,j) = -1;
        end
    end
end