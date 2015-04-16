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
% This is a main code of kinematic structure learning from video inputs
% =====================================================================

clc
close all
clear all

%%
addpath(genpath('../../include/include_Matlab'));
%%
% help

%%
% video input selection
disp('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++');
submodule_video_input_select;

%%
% video load 
% -- feature extraction
% ---- saving feature values
submodule_video_loading_feature_extraction;

%%
% converting feature data to y
[y, W, frames, points] = submodule_cvuKltRead([pwd,'/points/',filename(1:end-4),'/point_seq_%d.txt'], 1, nFrames, 'workspace', points_total);
% [y, W, frames, points] = submodule_cvuKltRead(['/home/human/robot/wysiwyd/main/build/bin/data2014-12-17-15-00/points/video/point_seq_%d.txt'], 1, 279, 'file', []);

%%
% submodule_bg_removal;
% W = W_fg;
% y = y_fg;

%%
% main algorithm
disp('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++');
submodule_motion_segmentation;
%%
% output display
submodule_display_save_structure;
