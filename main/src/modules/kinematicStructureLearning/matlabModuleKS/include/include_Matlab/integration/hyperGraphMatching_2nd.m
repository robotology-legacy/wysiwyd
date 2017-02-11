setAlg;

%%
nAlg = 8;

%%
problem_2nd = similarityScore_2nd( KineStruct_P, KineStruct_Q );

if nAlg == 1
    [X] = wrapper_GM(Alg(nAlg), problem_2nd);
    X = reshape(X, problem_2nd.nP1, problem_2nd.nP2);
elseif nAlg == 2
    X = feval(Alg(nAlg).fhandle, Alg(nAlg), problem_2nd);
end
% X = X';

X = asgHun(X);
[val ind] = max(X);