%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% adative rank detection
function K = rank_detection(N,kappa)

    [U_org, S_org, V_org] = svd(N);
%     kappa = 10^(-5);
    lambda = diag(S_org);

    r_buf = [];
    for r=1:rank(N)-1
        r_buf = [r_buf,lambda(r+1)^2/(sum(lambda(1:r).^2))+kappa*r];
    end

    [min_value,K] = min(r_buf);