%% Implementaion of Reweighted Max-Pooling Matching algortihm
function [ X ] = MPM( M, group1, group2, varargin)
% Max-Pooling Matching alrothim
%
% M. Cho, J. Sun, O. Duchenne, J. Ponce
% Finding Matches in a Haystack: A Max-Pooling Strategy for Graph Matching in the Presence of Outliers 
% Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (2014) 
% http://www.di.ens.fr/willow/research/maxpoolingmatching/
%
% Please cite our work if you find this code useful in your research. 
%
% written by Minsu Cho, Inria - WILLOW / Ecole Normale Superieure 
% http://www.di.ens.fr/~mcho/
%
% input
%       M: affinity matrix
%       group1: conflicting match groups in domain 1 (size(M,1) x nGroup1)
%       group2: conflicting match groups in domain 2 (size(M,1) x nGroup2)
%                 
%       e.g. find(group1(:,3)) represents the third goup of matches  
%                               sharing the same feature in domain1   
%
% output
%       X: score distribution

%% Default Parameters
param = struct( ...
    'amp_max', 30, ...              % maximum value for amplification procedure
    'iterMax', 300, ...             % maximum value for amplification procedure
    'thresConvergence', 1e-15 ...  % convergence threshold 
);
param = parseargs(param, varargin{:});

%% parameter structure -> parameter value
strField = fieldnames(param);
for i = 1:length(strField), eval([strField{i} '=param.' strField{i} ';']); end

% eliminate conflicting elements to prevent conflicting walks
M = M.*~full(getConflictMatrix(group1, group2));

% initialize answer
nMatch = length(M);
prev_score = ones(nMatch,1)/nMatch; % buffer for the previous score
prev_score2 = prev_score;         % buffer for the two iteration ahead

bCont = 1;  iter_i = 0;

% for convergence check of power iteration
thresConvergence2 = length(prev_score)*norm(M,1)*eps;
la = prev_score'*M*prev_score;

cur_score = zeros(nMatch,1);

%% start main iteration
while bCont && iter_i < iterMax
    
    iter_i = iter_i + 1;
    cur_score = RMP_mult(M, prev_score, group1);
    sumCurScore = sqrt(sum(cur_score.^2)); % normalization of unit 2norm
    if sumCurScore>0, cur_score = cur_score./sumCurScore; end
    
    % Check the convergence
    if 1
        diff1 = sum((cur_score-prev_score).^2);
        diff2 = sum((cur_score-prev_score2).^2); % to prevent oscillations
        diff_min = min(diff1, diff2);
        if diff_min < thresConvergence
            bCont = 0;
        end
    else
        normed_cur_score = cur_score/norm(cur_score);
        if norm(M*normed_cur_score - la*normed_cur_score,1) < thresConvergence2
            bCont = 0;
        end
        la = normed_cur_score'*M*normed_cur_score;
    end

    prev_score2 = prev_score;
    prev_score = cur_score;
 
end % end of main iteration

X = cur_score;
end