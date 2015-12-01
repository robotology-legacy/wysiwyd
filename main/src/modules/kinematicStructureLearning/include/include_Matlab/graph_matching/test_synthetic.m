%% Evaluating all algorithms on Synthetic Dataset
%
% Created by: Quynh Nguyen (17.10.2014)
%       @test:          1 - re-evaluate all algorithms from the beginning and then plot the results
%                       0 - simply plot the results from the saved output file 
%       @testname:      the name of the experimental setting, e.g. 'def_20inlier', 'out_001def', etc
%       @show_legend:   0-1 option for displaying legends
% Example of Usage:
%      - test_synthetic(1, 'def_20inlier', 1): evaluate algorithms on the
%      deformation test with 20 inlier points in the first point set, and
%      save results to output file 'def_20inlier.mat'.
%
%      - test_synthetic(0, 'def_20inlier', 1): read the saved output file
%      'def_20inlier.mat' and plot the results.
function test_synthetic(test, testname, show_legend)

%%%%%%%%%%%%%% RE-EVALUATION %%%%%%%%%%%%%%%
if test
    close all; clc;
    
    %%%%%% SET SOURCE PATHS
    setPath;
    
    %%%%%% SET EXPERIMENT SETTINGS
    eval(['setSettings_', testname]);
    
    %%%%%% SET ALGORITHMS
    setAlg;
    
    disp('* Check experiment settings *');
    disp(Set);
    
    %%%%%% START EVALUATING ALGORITHMS
    Accuracy = zeros(length(settings{Con}{4}), length(Alg), Set.nTest);
    MatchScore = zeros(length(settings{Con}{4}), length(Alg), Set.nTest);
    Time = zeros(length(settings{Con}{4}), length(Alg), Set.nTest);
    
    for i = length(settings{Con}{4}):-1:1
        fprintf('param: %d\n', settings{Con}{4}(i));
        
        for k = 1:Set.nTest
            eval(['Set.' settings{Con}{3} '=' num2str(settings{Con}{4}(i)) ';']);
            problem = makeMatchingProblem(Set);
            %         eval(['Set.' settings{Con}{3} '= settings{' num2str(Con) '}{4};']);
            
            problem_2nd = makeMatchingProblem_2nd(problem);
            
            X0 = problem.X0;
            % figure; imagesc(problem.gtruth);
            
            for j = 1:length(Alg)
                if strcmpi(Alg(j).strName, 'BCAGM')
                    tic;
                    [X, objs, nIter] = bcagm(Alg(j), problem, X0, X0, X0, X0, 30);
                    Time(i, j, k) = toc;
                    disp(['BCAGM: ', num2str(nIter)]);
                elseif strcmpi(Alg(j).strName, 'BCAGM+MP')
                    tic;
                    [X, objs, nIter] = bcagm_quad(Alg(j), problem, X0, X0, 1); % 1-MP, 2-IPFP
                    Time(i, j, k) = toc;
                    disp(['BCAGM+MP: ', num2str(nIter)]);
                elseif strcmpi(Alg(j).strName, 'BCAGM+IPFP')
                    tic;
                    [X, objs, nIter] = bcagm_quad(Alg(j), problem, X0, X0, 2); % 1-MP, 2-IPFP
                    Time(i, j, k) = toc;
                    disp(['BCAGM+IPFP: ', num2str(nIter)]);
                elseif j >= 6 % 2nd order methods
                    tic;
                    [X] = wrapper_GM(Alg(j), problem_2nd);
                    X = reshape(X, problem_2nd.nP1, problem_2nd.nP2);
                    X = X';
                    Time(i, j, k) = toc;
                    disp(Alg(j).strName);
                else % other 3rd order approaches
                    tic;
                    X = feval(Alg(j).fhandle, Alg(j), problem);
                    Time(i, j, k) = toc;
                end
                % discritize the returned solution by the Hungarian method
                X = asgHun(X);
                [val ind] = max(X);
                Xbin = zeros(size(X));
                for p = 1:size(X,2), Xbin(ind(p),p) = 1; end
                Accuracy(i, j, k) = (sum(ind == problem.trueMatch)) / problem.nP1;
                MatchScore(i, j, k) = getMatchScore(problem.indH3, problem.valH3, Xbin);
            end
            
            for j = 1:length(Alg)
                disp([Alg(j).strName, ':    ', num2str(Accuracy(i, j, k)), '    ', num2str(MatchScore(i, j, k)), '    ', num2str(Time(i, j, k))]);
            end
            fprintf('=============================\n');
        end
        fprintf('\n');
    end
    
    %%%%%% SAVE RESULTS
    save([testname, '.mat'], 'Accuracy', 'MatchScore', 'Time', 'Alg');
end

%%%%%%%%%%%%%%%%% PLOT RESULTS FROM SAVED FILE %%%%%%%%%%%%%%%%%%%
%% load data
load(testname);

%% Settings Evaluations
setPath;
eval(['setSettings_', testname]);
disp('* Check experiment settings *');
disp(Set);

%% just for plotting order (optional)
order = [10 9 8 7 1 2 3 4 5 6];
Alg = Alg(order);
Accuracy = Accuracy(:, order, :);
MatchScore = MatchScore(:, order, :);
Time = Time(:, order, :);

%% prepare data for plotting
stdScore = squeeze(std(MatchScore, 1, 3));
meanScore = mean(MatchScore, 3);

stdAcc = squeeze(std(Accuracy, 1, 3));
meanAcc = mean(Accuracy, 3);

stdTime = squeeze(std(Time, 1, 3));
meanTime = mean(Time, 3);

plotSet.lineWidth = 3;
plotSet.markerSize = 10;
plotSet.fontSize = 20;
% plotSet.font = '\fontname{times new roman}';
plotSet.font = '\fontname{Arial}';

handleCount = 0;
stdData = stdAcc; yData = meanAcc; yLabelText = 'Accuracy'; Ymin = 0; Ymax = 1; filename = [testname, '_acc', num2str(show_legend)]; plotResults;
stdData = stdScore; yData = meanScore; yLabelText = 'Matching Score'; filename = [testname, '_score', num2str(show_legend)]; plotResults;
stdData = stdTime; yData = meanTime; yLabelText = 'Running Time'; filename = [testname, '_time', num2str(show_legend)]; plotResults;

%% CLOSE MATLAB (OPTIONAL)
% quit;

