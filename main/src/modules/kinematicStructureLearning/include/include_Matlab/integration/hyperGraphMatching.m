function X = hyperGraphMatching(problem, HGM_method, Alg)

% setAlg;

%%
AlgIdx = HGM_method;
X0 = problem.X0;
% for AlgIdx = 1:length(Alg)

switch AlgIdx
    
%------------- 3rd order method           
    case 1  % HGM
        X = feval(Alg(AlgIdx).fhandle, Alg(AlgIdx), problem);        
    case 2  % TM
        X = feval(Alg(AlgIdx).fhandle, Alg(AlgIdx), problem);        
    case 3  % RRWHM
        X = feval(Alg(AlgIdx).fhandle, Alg(AlgIdx), problem);        
    case 4  % BCAGM
        [X, objs, nIter] = bcagm(Alg(AlgIdx), problem, X0, X0, X0, X0, 30);
    case 5  % BCAGM+MP
        [X, objs, nIter] = bcagm_quad(Alg(AlgIdx), problem, X0, X0, 1); % 1-MP, 2-IPFP
    case 6  % BCAGM+IPFP
        [X, objs, nIter] = bcagm_quad(Alg(AlgIdx), problem, X0, X0, 2); % 1-MP, 2-IPFP
        
%------------- 2nd order method       
    case 7  % MPM
        [X] = wrapper_GM(Alg(AlgIdx), problem_2nd);
        X = reshape(X, problem_2nd.nP1, problem_2nd.nP2);
        X = X';
    case 8  % RRWM
        [X] = wrapper_GM(Alg(AlgIdx), problem_2nd);
        X = reshape(X, problem_2nd.nP1, problem_2nd.nP2);
        X = X';
    case 9  % IPFP
        [X] = wrapper_GM(Alg(AlgIdx), problem_2nd);
        X = reshape(X, problem_2nd.nP1, problem_2nd.nP2);
        X = X';
    case 10 % SM
        [X] = wrapper_GM(Alg(AlgIdx), problem_2nd);
        X = reshape(X, problem_2nd.nP1, problem_2nd.nP2);
        X = X';
end
disp(['*** Hypergraph Matching Method: ',Alg(AlgIdx).strName]);

% figure
% surf(X)

X = asgHun(X)';

end