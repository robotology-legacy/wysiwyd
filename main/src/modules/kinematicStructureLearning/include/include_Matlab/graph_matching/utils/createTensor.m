function problem = createTensor(bPermute, P1, P2)

nP1 = size(P1, 2);
nP2 = size(P2, 2);

% permute graph sequence (prevent accidental good solution)
if bPermute, seq = randperm(nP2); problem.whole_seq = seq; P2(:,seq) = P2; seq = seq(1:nP1); else seq = 1:nP1; end

%% 3rd Order Tensor Construction
% This part is taken from Duchenne's code
nT = nP1*nP2; % # of triangles in graph 1
t1=floor(rand(3,nT)*nP1);
while 1
    probFound=false;
    for i=1:3
        ind=(t1(i,:)==t1(1+mod(i,3),:));
        if(nnz(ind)~=0)
            t1(i,ind)=floor(rand(1,nnz(ind))*nP1);
            probFound=true;
        end
    end
    if(~probFound)
        break;
    end
end

%generate features
t1=int32(t1);

[feat1,feat2] = mexComputeFeature(P1,P2,int32(t1),'simple');

%number of nearest neighbors used for each triangle (results can be bad if too low)
nNN = nT;
% nNN = 30;

%find the nearest neighbors
[inds, dists] = annquery(feat2, feat1, nNN, 'eps', 10);

%build the tensor
[i j k]=ind2sub([nP2,nP2,nP2],inds);
tmp=repmat(1:nT,nNN,1);
indH3 = double(t1(:,tmp(:))'*nP2) + [k(:)-1 j(:)-1 i(:)-1];
valH3 = exp(-dists(:) / mean(dists(:)));

% added by Quynh Nguyen
%remove duplicated tuples
indH3 = sort(indH3, 2);
[indH3 id1 id2] = unique(indH3, 'rows');
valH3 = valH3(id1);
%remove duplicated indices: (i,i,i), (i,i,j), (i,j,i), (i,j,j), etc
t1 = indH3(:, 1) - indH3(:, 2);
t2 = indH3(:, 1) - indH3(:, 3);
t3 = indH3(:, 2) - indH3(:, 3);
t4 = (t1 == 0) + (t2 == 0) + (t3 == 0);
nnz(t4)
indH3 = indH3(t4 == 0, :);
valH3 = valH3(t4 == 0);
% upperbound the number of nonzeros
maxL = min(5*10^6, length(valH3));
[v id] = sort(valH3, 'descend');
% id = randperm(length(valH3));
id = id(1:maxL);
valH3 = valH3(id);
indH3 = indH3(id, :);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Super-symmetrize the 3rd order tensor: if (i,j,k) with i#j#k is a nonzero
% entry of the tensor, then all of its six permutations should also be the entries
% NOTE that this step is important for the current implementation of our algorithms !!!
% Thus, if a tuple (i,j,k) for i#j#k is stored in 'indH3' below, then all of its
% permutations should also be stored in 'indH3' and 'valH3'.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ps = perms([1 2 3]);
Nt3 = length(valH3);
valH3 = [valH3; valH3; valH3; valH3; valH3; valH3];
old_indH3 = indH3;
indH3 = [];
for i = 1:size(ps, 1)
    indH3((i-1)*Nt3+1:i*Nt3, :) = old_indH3(:, ps(i, :));
end

% sort tuples in ascending order
dim = nP1 * nP2;
uid = indH3(:, 1)*dim*dim + indH3(:, 2)*dim + indH3(:, 3);
[v id] = sort(uid);
valH3 = valH3(id);
indH3 = indH3(id, :);

clear uid v id t1 t2 t3 t4 old_indH3 inds dists feat1 feat2;

%% save the matching problem
problem.nP1 = nP1;
problem.nP2 = nP2;
problem.P1 = P1;
problem.P2 = P2;
problem.indH1 = [];
problem.valH1 = [];
problem.indH2 = [];
problem.valH2 = [];
problem.indH3 = int32(indH3);
problem.valH3 = double(valH3);
problem.trueMatch = seq;
problem.gtruth = zeros(nP2, nP1);
for i = 1:nP1
    problem.gtruth(seq(i), i) = 1;
end

problem.X0 = ones(nP2, nP1);

clear indH1 valH1 indH2 valH2 indH3 valH3;


