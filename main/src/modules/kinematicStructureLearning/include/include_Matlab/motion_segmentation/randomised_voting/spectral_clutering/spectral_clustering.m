function [I,qual]=spectral_clustering(Aff,k)


%first build distance matrix
D=diag(Aff*ones(size(Aff,1),1));


%multicut mapping
[v d]=eigs(Aff,D,k,'la'); %generalised eigevectors
dd=diag(d);
[~ ,indices]=sort(-dd);
X=v(:,indices);



Y=normr_singleMatrix(X); %normalise the eigenvectors


%% EIGENSPACE CLUSTERING
%  k-means Euclidean
[~,I,~,~,quality] = fastkmeans(Y,k,2);


%calculate cluster quality
qual=quality; %min is good


end

function y = normr_singleMatrix(x)
cols = size(x,2);
n = 1 ./ sqrt(sum(x.*x,2));
y = x .* n(:,ones(1,cols));
y(~isfinite(y)) = 1;
end