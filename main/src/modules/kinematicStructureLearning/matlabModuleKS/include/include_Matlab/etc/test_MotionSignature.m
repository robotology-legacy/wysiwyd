%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Motion Signature Test
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

%%
control_params;
%%
%=========================================================
% data load
%=========================================================
%-----------------------
% from mat files
%-----------------------
[filename, pathname, filterindex] = uigetfile('../dataset/cvpr2015/*.mat');

load([pathname,filename]);
num_frames = size(y,3);
num_points = size(y,2);
%-------------------------------------
W = zeros(2*num_frames, num_points);

for f = 1:num_frames
    W(f,:) = y(1,:,f);
    W(f+num_frames,:) = y(2,:,f);
end

%-----------------------
% from video file
%-----------------------
% videoFileName = 'iCub_motion.avi'; width = 720; height = 480;
% init_frm = 31;
% end_frm = 280;
% [y, W, frames, points] = mycvuKltRead(['/home/hjchang/OpenCV/test_codes/lk_track/points/',videoFileName,'/point_seq_%d.txt'], init_frm, end_frm);

%%
%=========================================================
% motion segmentation using Randomised Voting
%=========================================================
% parameter setting
num_seg = 4;

%-------------------------------------
T_i = 300; % the number of iteration     % 150
T = 100; % the number of trial
T_c = 15; % convergence test, noise free : 15, noise : 5
T_r = 30; % reinitialization at T_r
alpha = 0.9; % decay parameter
lambda = 2; % voting strength

d = 1;
converge_check2 = 0;

max_dist = ones(1,num_seg);
best_overlap = 0

% y is trajectory, 3 x (the number of points) x (the number of frames)
% T, T_i, T_c, T_r, alpha, lambda : tuning parameters
% c : the number of group
% s : ground truth, (the number of points) x 1

final = my_motion_segmentation_parallel(y, T, T_i, T_c, T_r, alpha, lambda, num_seg);

%%
% do segmentation
motion_seg_from_final;

motion_seg_center_calculation;

% segmentation result visualisation
motion_seg_visualisation;

% connecting segments
motion_seg_connection;

%%
% Motion signature

% F = [];
% for frm_idx = 1:frames-1
%     x_k = y(:,:,frm_idx);
%     x_l = y(:,:,frm_idx+1);
%     [F_buf,e1,e2] = fundmatrix(x_k,x_l);
%     F = [F ; vec(F_buf-eye(3))'];
% end

F = [];
for frm_idx = 1:frames-1
    x_k = y(:,:,frm_idx);
    x_l = y(:,:,frm_idx+1);
    [F_buf,e1,e2] = fundmatrix(x_k,x_l);
    [rot, t] = EssentialMatrixToCameraMatrix(F_buf);
    R = rot(:,:,1);
    T = t(:,:,1);
    F = [F ; [vec(R-eye(3))',T']];
end

% F = [];
% for frm_idx = 1:frames-1
%     for seg_num = 1:num_seg
%         x_k = y(:,seg_idx{seg_num},frm_idx);
%         x_l = y(:,seg_idx{seg_num},frm_idx+1);
%         [F_buf,e1,e2] = fundmatrix(x_k,x_l);
%         F = [F ; vec(F_buf-eye(3))'];
%     end
% end




%%
%=========================================================
% remove path
%=========================================================
rmpath(genpath('../../cvpr2015'));