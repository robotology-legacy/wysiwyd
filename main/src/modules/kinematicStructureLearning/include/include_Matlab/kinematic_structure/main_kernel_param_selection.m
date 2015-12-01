%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Latent Kinematic Structure Estimation
%
% submission to CVPR 2015
%
% Hyung Jin Chang
% hj.chang@imperial.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
%=========================================================
% add path
%=========================================================
addpath(genpath('../../cvpr2015'));

%%
%=========================================================
% control
%=========================================================
clc
clear all
close all

%%
%=========================================================
% data load
%=========================================================
%-----------------------
% from mat files
%-----------------------
[filename, pathname, filterindex] = uigetfile('../dataset/*.mat');

load([pathname,filename]);
num_frames = size(y,3);
num_points = size(y,2);
%-------------------------------------
W = zeros(2*num_frames, num_points);

for f = 1:num_frames
    W(f,:) = y(1,:,f);
    W(f+num_frames,:) = y(2,:,f);
end

%%
%=========================================================
% shape based skeleton estimation
%=========================================================

% adaptive kernel parameter selection
% SVDD_param_selection_analysis;
% SVDD_param_selection_analysis_Landgrebe;
SVDD_param_selection_analysis_SA;
% SVDD_param_selection_analysis_Kim;


