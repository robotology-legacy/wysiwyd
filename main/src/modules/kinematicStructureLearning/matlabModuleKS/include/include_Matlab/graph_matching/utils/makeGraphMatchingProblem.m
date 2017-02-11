function [ problem ] = makeGraphMatchingProblem( Set )
%% Extract all variables from Set
strNames = fieldnames(Set);
for i = 1:length(strNames), eval([strNames{i} '= Set.' strNames{i} ';']); end
%% Set number of nodes
if bOutBoth, nP1 = nInlier + nOutlier; else nP1 = nInlier; end
nP2 = nInlier + nOutlier;
%% Generate two graps
% GT sequence
if bPermute, seq = randperm(nP2); else seq = 1:nP2; end
% Generate graph 1 
G1 = tril(rand(nP1),-1); % lower triangular graph
G1 = G1+G1';
P = tril(rand(nP1),-1);
P = P+P';
P = P > ratioFill;
G1(P) = NaN;

N = deformation*tril(randn(nP2),-1);
N = N+N';

G2 = tril(rand(nP2),-1);
G2 = G2+G2';
P = tril(rand(nP2),-1);
P = P+P';
P = P > ratioFill;
G2(P) = NaN;
G2(seq(1:nInlier),seq(1:nInlier)) = G1(1:nInlier,1:nInlier);
G2 = G2+N;

E12 = ones(nP1,nP2);
[L12(:,1) L12(:,2)] = find(E12);
[group1 group2] = make_group12(L12);

M = (repmat(G1, nP2, nP2)-kron(G2,ones(nP1))).^2;
M = exp(-M./scale_2D);
M(isnan(M)) = 0;
M(1:(length(M)+1):end)=0;
M = M+M';

%% Ground Truth
GT.seq = seq(1:nInlier);
GT.matrix = zeros(nP1, nP2);
for i = 1:nInlier, GT.matrix(i,seq(i)) = 1; end
GT.bool = GT.matrix(:);

%% Return results
problem.nP1 = nP1;
problem.nP2 = nP2;
problem.E12 = E12;
problem.L12 = L12;
problem.affinityMatrix = M;
problem.group1 = group1;
problem.group2 = group2;

problem.GTbool = GT.bool;

