% written by Jungmin Lee, Minsucho
% History
%	- modified by Quynh Nguyen (for CVPR 2015)
function [X, nIter] = RRWHM( alg, problem )

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

iterMax = 300;
c = 0.2;

% modified by Quynh Nguyen
% X = ones(problem.nP2,problem.nP1)/problem.nP2/problem.nP2;
X = problem.X0;
X = X ./ sum(sum(X));

[X, nIter] = mexRRWHM(X,indH1,valH1,indH2,valH2,indH3,valH3,iterMax,c);
disp(['RRWHM: ', num2str(nIter), ' iters']);
