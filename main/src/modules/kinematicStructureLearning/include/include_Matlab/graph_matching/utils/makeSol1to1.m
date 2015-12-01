function [ Y ] = makeSol1to1( X )
s = size(X);
Y = zeros(s);
for k = 1:min(s)
    [val idx] = max(X(:));
    [a b] = ind2sub(s,idx);
    Y(a,b) = 1;
    X(a,:) = 0;
    X(:,b) = 0;
end
