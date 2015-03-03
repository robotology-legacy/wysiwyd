%% Latent Tree Model

motion_seg_idx = cell(c,1);
for i=1:c
    motion_seg_idx{i} = ['Seg_{',num2str(i),'}'];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
num_node = size(motion_seg_idx,1);
cov_mat = zeros(num_node,num_node);
ticker = motion_seg_idx;

% root = 9;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

useDistances = 1;
verbose = 0;
numSamples = 10000;
% numSamples = 2913;
% numSamples = size(output_num,2);
comp_time = zeros(5,1);

algorithm_name = {'ChowLiu','CLNJ','CLRG','NJ','RG'};

tic;
adjmatT{1} = ChowLiu(-distance);
edgeD{1} = distance.*adjmatT{1};
comp_time(1) = toc; tic;
[adjmatT{2},edgeD{2}] = CLNJ(distance, useDistances);
comp_time(2) = toc; tic;
[adjmatT{3},edgeD{3}] = CLRG(distance, useDistances, numSamples);
comp_time(3) = toc; tic;
[adjmatT{4},edgeD{4}] = NJ(distance, useDistances);
comp_time(4) = toc; tic;
[adjmatT{5},edgeD{5}] = RG(distance, useDistances, numSamples);
comp_time(5) = toc;

%%
% selecting root
adjmatT_buf = adjmatT{4};
adjmatT_full = full(adjmatT_buf);
adjmatT_full(1:num_node, 1:num_node) = 0;
buf = sum(adjmatT_full,1);
[value,root] = max(buf(1:num_node));

% root = 7;

%%
% drawing the LTM trees
figure(100);
clf
for i=1:length(adjmatT)
%     ll(i) = numSamples*computeLL_Gaussian(sample_cov, edgeD{i});
%     bic(i) = ll(i) - (size(adjmatT{i},1)-1)/2 * log(numSamples);
%     fprintf('Log-likelihood = %f, BIC score = %f, num_hidden = %d\n',ll(i),bic(i),size(adjmatT{i},1)-m);
    subplot(1,5,i);
    title(algorithm_name{i});
    drawLatentTree(adjmatT{i},num_node,root,ticker);
end