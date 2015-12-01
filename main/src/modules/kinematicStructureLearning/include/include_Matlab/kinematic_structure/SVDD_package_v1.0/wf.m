function p = wf(x)

[R C] = size(x);

if R == 1
    p = 1/sqrt(2*pi) * exp(-1/2 * (x*x'));
elseif C ==1
    p = 1/sqrt(2*pi) * exp(-1/2 * (x'*x));    
end

% r = 1/sqrt(2*pi) * exp(-1/2 * (x*x'));
% p = diag(r);