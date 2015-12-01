%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main code
%
% Preparing CVPR 2016 submission
%
% Hyung Jin Chang
% hj.chang@imperial.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear all
close all

%%
%=========================================================
% add path
%=========================================================
addpath(genpath('../../cvpr2016'));
rmpath(genpath('../../cvpr2016/code/comparisons/appearance_match'));

%%
control_params;
%%
%=========================================================
% data load
%=========================================================
%-----------------------
% from mat files
%-----------------------
[filename_P, pathname_P, filterindex_P] = uigetfile('../dataset/data/');
[filename_Q, pathname_Q, filterindex_Q] = uigetfile('../dataset/data/');

fileNameOnly_P = filename_P(1:end-4);
fileNameOnly_Q = filename_Q(1:end-4);
filePath_P = [pathname_P,filename_P];
filePath_Q = [pathname_Q,filename_Q];

%%
%=========================================================
% Kinematic Structure Estimation
%=========================================================
KineStruct_P = estimate_KineStruct(filePath_P, ctrl_param);
KineStruct_Q = estimate_KineStruct(filePath_Q, ctrl_param);

%%
%=========================================================
% Calculate Similarity
%=========================================================
%(1:HGM / 2:TM / 3:RRWHM / 4:BCAGM / 5:BCAGM+MP / 6:BCAGM+IPFP / 7:MPM / 8:RRWM / 9:IPFP / 10:SM)
HGM_method = 3;
%%
if HGM_method <= 3
    tic
%         problem = createSimilarity(KineStruct_P, KineStruct_Q);
    problem = createSimilarity_RRWHM_relative(KineStruct_P, KineStruct_Q);
%     problem = createSimilarity_RRWHM_relative_unweighted(KineStruct_P, KineStruct_Q);
%     problem = createSimilarity_RRWHM_relative_nonPreprocessing(KineStruct_P, KineStruct_Q);
% createSimilarity_RRWHM_relative_nonPreprocessing
    time = toc
    
elseif HGM_method == 4 || HGM_method == 5 || HGM_method == 6
    problem = createSimilarity_BCAGM(KineStruct_P, KineStruct_Q);
end

%%
%=========================================================
% Hypergraph Matching
%=========================================================
setAlg;
% Alg(HGM_method).bOrder = [1 0 0];   % 1st
% Alg(HGM_method).bOrder = [0 1 0];   % 2nd
% Alg(HGM_method).bOrder = [0 0 1];   % 3rd
% Alg(HGM_method).bOrder = [1 1 1];   % all

X = hyperGraphMatching(problem, HGM_method, Alg);

figure(98); imagesc(X);

%%
%=========================================================
% Draw Matching Result
%=========================================================
step_size = 5;
img_acc_P = genAccImageSeq(pathname_P, KineStruct_P, step_size);
% figure(1001)
% imshow(img_acc_P);
img_acc_Q = genAccImageSeq(pathname_Q, KineStruct_Q, step_size);
% figure(1002)
% imshow(img_acc_Q);

%% Save images
% img_combined_color = genColorExampleImage(pathname_P, KineStruct_P, pathname_Q, KineStruct_Q);
% img_combined_color_ACC = genAccColorExampleImage(pathname_P, KineStruct_P, pathname_Q, KineStruct_Q, step_size);
% img_output = genColorImageStructure(pathname_P, KineStruct_P, pathname_Q, KineStruct_Q);

%% Draw result
switch(HGM_method)
    case 1
        img_combined = genMatchImage(img_acc_P, img_acc_Q, KineStruct_P, KineStruct_Q, X, 'PROPOSED_HGM');
    case 2
        img_combined = genMatchImage(img_acc_P, img_acc_Q, KineStruct_P, KineStruct_Q, X, 'PROPOSED_TM');
    case 3
        img_combined = genMatchImage(img_acc_P, img_acc_Q, KineStruct_P, KineStruct_Q, X, 'PROPOSED_RRWHM');
    case 4
        img_combined = genMatchImage(img_acc_P, img_acc_Q, KineStruct_P, KineStruct_Q, X, 'PROPOSED_BCAGM');
    case 5
        img_combined = genMatchImage(img_acc_P, img_acc_Q, KineStruct_P, KineStruct_Q, X, 'PROPOSED_BCAGM+MP');
    case 6
        img_combined = genMatchImage(img_acc_P, img_acc_Q, KineStruct_P, KineStruct_Q, X, 'PROPOSED_BCAGM+IPFP');
end

%% Calculate Accuracy
accuracy = cal_accuracy_GT(KineStruct_P, KineStruct_Q, X)

%%
%=========================================================
% remove path
%=========================================================
% rmpath(genpath('../../cvpr2016'));