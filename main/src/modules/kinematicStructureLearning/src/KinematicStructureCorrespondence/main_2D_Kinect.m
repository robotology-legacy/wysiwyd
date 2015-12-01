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
% Data Load & Kinematic Structure Estimation
%=========================================================
% for Kinematic structure P
[filename_P, pathname_P, filterindex_P] = uigetfile('../dataset/data/');
% filename_P = 'dancing.mat';
% pathname_P = '/home/hjchang/Research/code/Matlab/cvpr2016/dataset/data/dancing/';
fileNameOnly_P = filename_P(1:end-4);
filePath_P = [pathname_P,filename_P];
KineStruct_P = estimate_KineStruct(filePath_P, ctrl_param);

pathname_Q = uigetdir('../dataset/data/kinect_data/');
% KineStruct_Q = estimate_KineStruct_from_Kinect(foldername_Q);
KineStruct_Q = estimate_KineStruct_from_Kinect_3D(pathname_Q);

%%
%=========================================================
% Calculate Similarity
%=========================================================
%(1:HGM / 2:TM / 3:RRWHM / 4:BCAGM / 5:BCAGM+MP / 6:BCAGM+IPFP / 7:MPM / 8:RRWM / 9:IPFP / 10:SM)
HGM_method = 3;
%%
if HGM_method <= 3
    tic
    %     problem = createSimilarity(KineStruct_P, KineStruct_Q);
    problem = createSimilarity_RRWHM_relative_2D_3D(KineStruct_P, KineStruct_Q);
    time = toc
    
elseif HGM_method == 4 || HGM_method == 5 || HGM_method == 6
    problem = createSimilarity_BCAGM(KineStruct_P, KineStruct_Q);
end

%%
%=========================================================
% Hypergraph Matching
%=========================================================
setAlg;
X = hyperGraphMatching(problem, HGM_method, Alg);

figure(98); imagesc(X);

% %%
% %=========================================================
% % Draw Matching Result
% %=========================================================
% step_size = 5;
% img_acc_P = genAccImageSeq(pathname_P, KineStruct_P, step_size);
% % figure(1001)
% % imshow(img_acc_P);
% img_acc_Q = genAccImageSeq(pathname_Q, KineStruct_Q, step_size);
% % figure(1002)
% % imshow(img_acc_Q);

%%
% img_combined_color = genColorExampleImage(pathname_P, KineStruct_P, pathname_Q, KineStruct_Q);
% img_combined_color_ACC = genAccColorExampleImage(pathname_P, KineStruct_P, pathname_Q, KineStruct_Q, step_size);
% img_output = genColorImageStructure(pathname_P, KineStruct_P, pathname_Q, KineStruct_Q);
%%
% if HGM_method == 3
%     img_combined = genMatchImage(img_acc_P, img_acc_Q, KineStruct_P, KineStruct_Q, X, 'PROPOSED_RRWHM');
% elseif HGM_method >= 4
%     img_combined = genMatchImage(img_acc_P, img_acc_Q, KineStruct_P, KineStruct_Q, X, 'PROPOSED_BCAGM');
% end

%%
figure(98); imagesc(X);
img_combined = genMatchSynth(KineStruct_P, KineStruct_Q, X);
