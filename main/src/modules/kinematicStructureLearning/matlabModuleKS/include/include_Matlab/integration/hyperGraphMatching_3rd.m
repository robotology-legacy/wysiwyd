setAlg;

%%
AlgIdx = 1;

%%
problem = createMotionTensor(KineStruct_P, KineStruct_Q);
X0 = problem.X0;

%%
X0 = problem.X0;
% X_set = cell(1,length(Alg));
X_set = cell(1,6);
% for AlgIdx = 1:length(Alg)
for AlgIdx = 1:6
    if strcmpi(Alg(AlgIdx).strName, 'BCAGM')
        [X, objs, nIter] = bcagm(Alg(AlgIdx), problem, X0, X0, X0, X0, 30);
        disp(['BCAGM: ', num2str(nIter)]);
    elseif strcmpi(Alg(AlgIdx).strName, 'BCAGM+MP')
        [X, objs, nIter] = bcagm_quad(Alg(AlgIdx), problem, X0, X0, 1); % 1-MP, 2-IPFP
        disp(['BCAGM+MP: ', num2str(nIter)]);
    elseif strcmpi(Alg(AlgIdx).strName, 'BCAGM+IPFP')
        [X, objs, nIter] = bcagm_quad(Alg(AlgIdx), problem, X0, X0, 2); % 1-MP, 2-IPFP
        disp(['BCAGM+IPFP: ', num2str(nIter)]);
    elseif AlgIdx >= 6 % 2nd order methods
        [X] = wrapper_GM(Alg(AlgIdx), problem_2nd);
        X = reshape(X, problem_2nd.nP1, problem_2nd.nP2);
        X = X';
    else % other 3rd order approaches
        X = feval(Alg(AlgIdx).fhandle, Alg(AlgIdx), problem);
    end
    X_set{AlgIdx} = asgHun(X)';    
    disp(Alg(AlgIdx).strName)
    asgHun(X)'
end
% X = asgHun(X);
% [val ind] = max(X);

X = X_set{1}