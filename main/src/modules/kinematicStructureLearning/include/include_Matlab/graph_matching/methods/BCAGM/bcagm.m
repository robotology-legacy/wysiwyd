% written by Quynh Nguyen
% References
% 	Quynh Nguyen, Antoine Gautier, Matthias Hein
%  	A Flexible Tensor Block Coordinate Ascent Scheme for Hypergraph Matching, CVPR 2015

% NOTE that the input arrays: problem.indH3 and problem.valH3 should be symmetric themselves. 
% This means they should store all the six permutations of each non-zero entries instead of storing only one

function [X, objs, nIter] = bcagm(alg, problem, X1, X2, X3, X4, maxIter)
if alg.bOrder(1)
    indH1 = problem.indH1;
    valH1 = problem.valH1;
else
    indH1 = [];
    valH1 = [];
end

if alg.bOrder(2)
    indH2 = problem.indH2;
    valH2 = problem.valH2;
else
    indH2 = [];
    valH2 = [];
end

if alg.bOrder(3)
    indH3 = problem.indH3;
    valH3 = problem.valH3;
else
    indH3 = [];
    valH3 = [];
end

if isempty(indH1) && isempty(valH1)
  indH1=int32(zeros(0,1));
  valH1=zeros(0,1);
end
if isempty(indH2) && isempty(valH2)
  indH2=int32(zeros(0,2));
  valH2=zeros(0,1);
end
if isempty(indH3) && isempty(valH3)
  indH3=int32(zeros(0,3));
  valH3=zeros(0,1);
end

[N2, N1] = size(X1);
X0 = [X1(:); X2(:); X3(:); X4(:)];
[X, objs, nIter] = mexBCAGMMatching(int32(indH1'), double(valH1),...
                                    int32(indH2'), double(valH2),...
                                    int32(indH3'), double(valH3),...
                                    int32(N1), int32(N2), double(X0), int32(maxIter));
                                
                                
