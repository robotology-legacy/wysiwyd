% ==========================================================================
% Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
% Authors: Hyung Jin Chang
% email:   (hj.chang@imperial.ac.uk)
% Permission is granted to copy, distribute, and/or modify this program
% under the terms of the GNU General Public License, version 2 or any
% later version published by the Free Software Foundation.
% 
% A copy of the license can be found at
% wysiwyd/license/gpl.txt
% 
% This program is distributed in the hope that it will be useful, but
% WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
% Public License for more details
%
% This is a main code of kinematic structures from two input videos 
% and find kinematic correspondences between the generated kinematic
% structures
% =====================================================================

clc
close all
clear all

%%
%=========================================================
% add path
%=========================================================
addpath(genpath('../../include/include_Matlab'));   % add path of libraries
addpath(genpath('submodule'));     % add path of required submodules

%%
control_params;

%%
%=========================================================
% video input selection
%=========================================================
%=========================================================
% video load 
% -- feature extraction
% ---- saving feature values
%=========================================================
disp('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++');
[videoName_P, numOfSegments_P] = videoInputSelect();
cdata_P = featureExtraction(videoName_P);
[videoName_Q, numOfSegments_Q] = videoInputSelect();
cdata_Q = featureExtraction(videoName_Q);

%%
%=========================================================
% converting feature data to y
%=========================================================
[y_P, W_P, frames_P, points_P] = submodule_cvuKltRead([pwd,'/points/',cdata_P.filename(1:end-4),'/point_seq_%d.txt'], 1, cdata_P.nFrames, 'workspace', cdata_P.points_total);
[y_Q, W_Q, frames_Q, points_Q] = submodule_cvuKltRead([pwd,'/points/',cdata_Q.filename(1:end-4),'/point_seq_%d.txt'], 1, cdata_Q.nFrames, 'workspace', cdata_Q.points_total);

%%
%=========================================================
% submodule_bg_removal;
%=========================================================
% W = W_fg;
% y = y_fg;

%%
%=========================================================
% Kinematic Structure Generation
%=========================================================
disp('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++');
% submodule_motion_segmentation;
KineStruct_P = genKineStruct(y_P, numOfSegments_P, cdata_P, ctrl_param);
KineStruct_Q = genKineStruct(y_Q, numOfSegments_Q, cdata_Q, ctrl_param);

if KineStruct_P.num_seg > KineStruct_Q.num_seg
    submodule_swiping_KineStruct;
end

%%
%=========================================================
% output display
%=========================================================
% saveKineStruct(y_P, KineStruct_P, cdata_P, videoName_P);
% saveKineStruct(y_Q, KineStruct_Q, cdata_Q, videoName_Q);

%%
%=========================================================
% Calculate Similarity
%=========================================================
%(1:HGM / 2:TM / 3:RRWHM / 4:BCAGM / 5:BCAGM+MP / 6:BCAGM+IPFP / 7:MPM / 8:RRWM / 9:IPFP / 10:SM)
HGM_method = 3;
%%
if HGM_method <= 3
    problem = createSimilarity_RRWHM_relative(KineStruct_P, KineStruct_Q);   
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
img_acc_P = genAccImageSeq(cdata_P.pathname, KineStruct_P, step_size);
img_acc_Q = genAccImageSeq(cdata_Q.pathname, KineStruct_Q, step_size);

%% 
%=========================================================
% Draw result
%=========================================================
img_combined = genMatchImage(img_acc_P, img_acc_Q, KineStruct_P, KineStruct_Q, X, 'PROPOSED_RRWHM');

%%
%=========================================================
% Remove path
%=========================================================
rmpath(genpath('../../include/include_Matlab'));   % add path of libraries
rmpath(genpath('submodule'));     % add path of required submodules