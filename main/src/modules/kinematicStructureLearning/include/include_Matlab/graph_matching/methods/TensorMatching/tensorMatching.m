% written by Oliver Duchenne: 
% History
%	- modified by Jungmin Lee, Minsucho, Kyoung Mu Lee (for CVPR 2011)
%	- modified by Quynh Nguyen (for CVPR 2015)
% function [X2 score] = tensorMatching(X,indH1,valH1,indH2,valH2,indH3,valH3,nIter,sparsity,stoc)
function X2 = tensorMatching(alg, problem)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%function [X2 score] = tensorMatching(X,indH1,valH1,indH2,valH2,indH3,valH3[,nIter,sparsity,stoc])
%input:
%  X / N1 x N2 matrix / initial assignment matrix
%  indHi / Nt x i matrix / indices in sparse tensor of order i
%  valHi / Nt x 1 / values in sprase tensor of order i
%  nIter / scalar integer / number of iteration for the power method
%  sparsity / boolean / should the algorithm use the sparsity trick as
%                         discribed in the article
%  stoc / boolean / 0 for no contraint on X, 1 for stochastic
%                         constraint, 2 for doubly stochastic
%  
%output:
%  X2 / N1 x N2 matrix / output assignment matrix
%  score / scalar / score of the matching (high means good matching)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

nIter=100;
sparsity=1;
switch alg.stoc
    case 'doubly'
        stoc = 2;
    case 'single'
        stoc = 1;
    otherwise
        stoc = 0;
end

% Modified by Quynh Nguyen
% X = 1/problem.nP2*ones(problem.nP2,problem.nP1);
X = problem.X0;
X = X ./ repmat(sum(X), size(X,1), 1);

[X2, score]=mexTensorMatching(double(X),indH1,valH1,indH2,valH2,indH3,valH3,nIter,sparsity,stoc);

