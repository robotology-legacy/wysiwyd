%% Show Demo Matching Result on a Random Task of the Synthetic Dataset
% Created by: Quynh Nguyen (17.10.2014)
%
%       @testname:      the name of the experimental setting where the test case will be generated from, e.g.
%       'def_20inlier', 'out_001def', etc.
%       @saveToFile:    0-1 option for saving the figures to files
% Example of Usage:
%      - demo_matching('def_20inlier', 1): show demo matching results of all algorithms with a random problem generated from
%       the deformation test whose settings are stored in 'setSettings_def_20inlier'.
function demo_matching(testname, saveToFile)

close all; clc;

%% Settings Evaluations
setPath;
eval(['setSettings_', testname]);
setAlg;

% disp('* Check experiment settings *');
% disp(Set);

% i = length(settings{Con}{4});
i = 9;
eval(['Set.' settings{Con}{3} '=' num2str(settings{Con}{4}(i)) ';']);
problem = makeMatchingProblem(Set);
disp(Set);
eval(['Set.' settings{Con}{3} '= settings{' num2str(Con) '}{4};']);

P1 = problem.P1;
P2 = problem.P2;
nP1 = problem.nP1;
nP2 = problem.nP2;
problem_2nd = makeMatchingProblem_2nd(problem, 2500);

%========================= show grouth truth ==========================
if saveToFile
    addpath('export_fig-master');
end
GT = problem.trueMatch;
figure; hold on;
% plot outliers from 2nd point cloud
plot(P2(1, :), P2(2, :), 'k*', 'markerSize', 5);
% plot inliers from 1st point cloud
plot(P1(1, :), P1(2, :), 'ro', 'markerFaceColor', 'r', 'markerSize', 6);
% plot inliers from 2nd point cloud
for i = 1:nP1
    plot(P2(1, GT(i)), P2(2, GT(i)), 'bo', 'markerFaceColor', 'b', 'markerSize', 6);
end
% plot true matches
% for i = 1:nP1
%     plot([P1(1, i), P2(1, GT(i))], [P1(2, i), P2(2, GT(i))], 'b-', 'lineWidth', 2);
% end
if saveToFile
    %     axis square;
    %     axis off;
    box on;
    set(gcf, 'color', 'w');
    export_fig(['groundTruth'], '-eps', '-painters', '-native');
else
    title('Grouth Truth');
end

X0 = problem.X0;
for j = 1:length(Alg)
    if strcmpi(Alg(j).strName, 'BCAGM')
        tic;
        [X, objs, nIter] = bcagm(Alg(j), problem, X0, X0, X0, X0, 30);
        time(j) = toc;
        disp(['BCAGM: ', num2str(nIter)]);
    elseif strcmpi(Alg(j).strName, 'BCAGM+MP')
        tic;
        [X, objs, nIter] = bcagm_quad(Alg(j), problem, X0, X0, 1); % 1-MP, 2-IPFP
        time(j) = toc;
        disp(['BCAGM+MP: ', num2str(nIter)]);
    elseif strcmpi(Alg(j).strName, 'BCAGM+IPFP')
        tic;
        [X, objs, nIter] = bcagm_quad(Alg(j), problem, X0, X0, 2); % 1-MP, 2-IPFP
        time(j) = toc;
        disp(['BCAGM+IPFP: ', num2str(nIter)]);
    elseif j >= 6 % 2nd order methods
        tic;
        [X] = wrapper_GM(Alg(j), problem_2nd);
        X = reshape(X, problem_2nd.nP1, problem_2nd.nP2);
        X = X';
        time(j) = toc;
        disp(Alg(j).strName);
    else % other 3rd order approaches
        tic;
        X = feval(Alg(j).fhandle, Alg(j), problem);
        time(j) = toc;
    end
    % discritize the returned solution by the Hungarian method
    X = asgHun(X);
    [val ind] = max(X);
    Xbin = zeros(size(X));
    for p = 1:size(X,2), Xbin(ind(p),p) = 1; end
    acc = (sum(ind == problem.trueMatch)) / problem.nP1;
    score = getMatchScore(problem.indH3, problem.valH3, Xbin);
    
    title_string = [Alg(j).strName, ': acc ', num2str(acc), ', score ', num2str(score)];
    show_matching(Alg(j).strName, P1, P2, Xbin, ind, problem.trueMatch, title_string, saveToFile);
end
