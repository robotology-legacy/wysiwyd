function y = generate_noise(y, sigma)
    f = size(y,3);
    m = size(y,2);

    if sigma == 0
        return;
    end

    % % % gaussian noise
    for i=1:m
        for j=1:f
            mu = [y(1, i, j) y(2, i, j)];
            SIGMA = [sigma*sigma 0; 0 sigma*sigma];
            r = mvnrnd(mu,SIGMA,1);

            y(1, i, j) = r(1);
            y(2, i, j) = r(2);
        end
    end
end