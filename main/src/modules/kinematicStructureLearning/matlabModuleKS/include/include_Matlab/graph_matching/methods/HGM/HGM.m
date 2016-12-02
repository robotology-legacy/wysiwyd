function X = HGM( Alg, problem )
%%
indH = problem.indH3+1;
valH = problem.valH3;

% modified by Quynh Nguyen
% E12 = ones(problem.nP2, problem.nP1);
E12 = problem.X0;

[L(:,1) L(:,2)] = find(E12);

Y = zeros(problem.nP2, problem.nP1);
for t = 1:length(valH)
    Y(L(indH(t,1),1), L(indH(t,1),2)) = Y(L(indH(t,1),1), L(indH(t,1),2)) + valH(t);
    Y(L(indH(t,2),1), L(indH(t,2),2)) = Y(L(indH(t,2),1), L(indH(t,2),2)) + valH(t);
    Y(L(indH(t,3),1), L(indH(t,3),2)) = Y(L(indH(t,3),1), L(indH(t,3),2)) + valH(t);
end

X = hypergraphMatching(Y,problem.nP1);
