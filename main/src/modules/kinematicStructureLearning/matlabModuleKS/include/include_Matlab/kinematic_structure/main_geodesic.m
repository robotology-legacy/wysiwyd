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

%--------------------------------------------------------
% result save
% result_save_ON = true;
result_save_ON = false;

%--------------------------------------------------------
% motion module control
motion_ON = true;
% motion_ON = false;

% motion_seg_plot_ON = true;
motion_seg_plot_ON = false;

motion_seg_connect_plot_ON = true;
% motion_seg_connect_plot_ON = false;

% motion_seg_connect_plot_with_video_ON = true;
motion_seg_connect_plot_with_video_ON = false;

%--------------------------------------------------------
% shape module control
shape_ON = true;
% shape_ON = false;

% shape_SVDD_training_plot_ON = true;
shape_SVDD_training_plot_ON = false;

shape_SVDD_load_ON = true;
% shape_SVDD_load_ON = false;

% shape_SVDD_load_plot_ON = true;
shape_SVDD_load_plot_ON = false; 

% shape_SVDD_result_save_ON = true;
shape_SVDD_result_save_ON = false;

%--------------------------------------------------------
% integration_seg_connect_plot_ON = true;
integration_seg_connect_plot_ON = false;

%--------------------------------------------------------
% LTM_ON = true;
LTM_ON = false;

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

error_buf = [];

%-----------------------
% from video file
%-----------------------
% videoFileName = 'iCub_motion.avi'; width = 720; height = 480;
% init_frm = 31;
% end_frm = 280;
% [y, W, frames, points] = mycvuKltRead(['/home/hjchang/OpenCV/test_codes/lk_track/points/',videoFileName,'/point_seq_%d.txt'], init_frm, end_frm);

%%
%=========================================================
% data save
%=========================================================
if result_save_ON
    current_time = clock;
    current_year = num2str(current_time(1));
    current_month = num2str(current_time(2));
    current_date = num2str(current_time(3));
    current_hour = num2str(current_time(4));
    current_minute = num2str(current_time(5));

    result_save_folder = ['../result/',filename(1:end-4),'/',current_year,'_',current_month,'_',current_date,'_',current_hour,'_',current_minute];

    mkdir(result_save_folder);
end

tic
%%
%=========================================================
% motion segmentation
%=========================================================
if motion_ON
    % parameter setting
    %     num_seg = 7;
    %     num_seg = floor(size(y,2)/(16))
    %     num_seg = floor(size(y,2)/(8))
    %     num_seg = floor(rank(W,10)/4)
    %     num_seg = ceil(rank(W,10)/4)
%     num_seg = min(ceil(rank(W,10)/4), floor(size(y,2)/(16)))
%     num_seg = max(ceil(rank(W,20)/4), floor(size(y,2)/(8)))
    num_seg = floor(size(y,2)/(8))
    %     num_seg = ceil(rank(W,10)/3)
    %     num_seg = floor(rank(W,num_points/10)/4)
    %     num_seg = ceil(rank(W,num_points/10)/4)
    
%         num_seg = 7
    %     num_seg = 11
    %-------------------------------------
    
    num_seg_history = [];
    seg_idx_history = cell(100,1);
    seg_idx_history_idx = 1;
    
    T_i = 200; % the number of iteration     % 150
    T = 3; % the number of trial
    T_c = 20; % convergence test, noise free : 15, noise : 5
    T_r = 15; % reinitialization at T_r
    alpha = 0.9; % decay parameter
    lambda = 2; % voting strength
    
    d = 1;
    converge_check2 = 0;
    
    max_dist = ones(1,num_seg);
    best_overlap = 0;
    
    % y is trajectory, 3 x (the number of points) x (the number of frames)
    % T, T_i, T_c, T_r, alpha, lambda : tuning parameters
    % c : the number of group
    % s : ground truth, (the number of points) x 1
    
    while(1)
        %    final = motion_segmentation(y, T, T_i, T_c, T_r, alpha, lambda, c, s);
        num_seg_history = [num_seg_history;num_seg];        
        final = my_motion_segmentation(y, T, T_i, T_c, T_r, alpha, lambda, num_seg);
        
        % do segmentation
        motion_seg_from_final;
        seg_idx_history{seg_idx_history_idx} = seg_idx;
        seg_idx_history_idx = seg_idx_history_idx + 1;
        motion_seg_num_small_set;
        if num_small_seg == 0
            break;
        else
            num_seg = num_seg - num_small_seg
        end
    end
    
    
    motion_seg_center_calculation;
    
    % segmentation result visualisation
%     motion_seg_visualisation;
    
    % connecting segments
%     motion_seg_connection;
    
    %////
    %     motion_seg_combining;
    %     % result visualisation
    %     motion_seg_visualisation;
    %     % connecting segments
    %     motion_seg_connection;
    
end
%%
%=========================================================
% shape based skeleton estimation
%=========================================================
if shape_ON
    
    if shape_SVDD_load_ON
        
        SVDD_data_load_draw;
        
    else
        % adaptive kernel parameter selection
        SVDD_param_selection;
        
        % shape and skeleton generation
        SVDD_shape_skeleton;
    end
    
end

%%
%=========================================================
% segmentation merging
%=========================================================
if motion_ON && shape_ON
    
    integration_cal_min_radius;
    motion_seg_connection_geodesic;
           
    do_seg_merge = 1;
    while(do_seg_merge)
        num_seg
        num_seg_history = [num_seg_history;num_seg];        
        
        integration_cal_seg_rad;
        integration_seg_merge;  
        seg_idx_history{seg_idx_history_idx} = seg_idx;
        seg_idx_history_idx = seg_idx_history_idx + 1;
        
        motion_seg_center_calculation;        
%         motion_seg_connection;
        motion_seg_connection_geodesic;
    end
    
%     SVDD_skeleton_weight_connection;    
%     SVDD_seg_connection;
%     
%     integration_seg_connection;    
%     integration_radius_drawing;        
%     motion_seg_visualisation;
%     connection_drawing_with_video;
end

total_time = toc
%%
%=========================================================
% Latent Tree Model
%=========================================================
if LTM_ON
    distance = integrated_dist;
    
    LTM_gen;
end

%%
%=========================================================
% error calculation
%=========================================================
[seg_idx_GT, seg_center_GT] = cal_seg_GT(s,y);
error = cal_error(y, s, seg_idx, seg_center, seg_idx_GT, seg_center_GT);

disp(['Error: ',num2str(error)]);
error_buf = [error_buf,error]

%%
%=========================================================
% save results
%=========================================================
% save([result_save_folder,'/workspace.mat']);

%%
%=========================================================
% remove path
%=========================================================
% rmpath(genpath('../../cvpr2015'));