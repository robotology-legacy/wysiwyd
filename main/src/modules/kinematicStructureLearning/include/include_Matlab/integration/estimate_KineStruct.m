function KineStruct = estimate_KineStruct(filepath, ctrl_param)


%%
% Load file from the filepath
load(filepath);

num_frames = size(y,3);
num_points = size(y,2);
%-------------------------------------
W = zeros(2*num_frames, num_points);

for f = 1:num_frames
    W(f,:) = y(1,:,f);
    W(f+num_frames,:) = y(2,:,f);
end

disp('=========================================================');
disp(['Perform Kinematic Structure Building...',videoFileName]);
disp('=========================================================');

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
num_seg = length(unique(segGT));

%-------------------------------------
T_i = 300; % the number of iteration     % 150
T = 50; % the number of trial
T_c = 15; % convergence test, noise free : 15, noise : 5
T_r = 30; % reinitialization at T_r
alpha = 0.9; % decay parameter
lambda = 2; % voting strength

% y is trajectory, 3 x (the number of points) x (the number of frames)
% T, T_i, T_c, T_r, alpha, lambda : tuning parameters
% c : the number of group
% s : ground truth, (the number of points) x 1


if ctrl_param.KineStruct.motion_GT_load_ON
    final = segGT;
else
    final = my_motion_segmentation_parallel(y, T, T_i, T_c, T_r, alpha, lambda, num_seg);
end

%-----------------------
% do motion segmentation
%-----------------------
motion_seg_from_final;

motion_seg_center_calculation;

%-----------------------
% segmentation result visualisation
%-----------------------
motion_seg_visualisation;

%-----------------------
% connecting segments
%-----------------------
motion_seg_connection;

%%
%=========================================================
% Shape based Skeleton Estimation
%=========================================================
if ctrl_param.KineStruct.shape_ON
    
    if ctrl_param.KineStruct.shape_SVDD_load_ON
        
        SVDD_data_load_draw;
        
    else
        % adaptive kernel parameter selection
        % SVDD_param_selection;
        SVDD_param_selection_proposed_SA
        
        % shape and skeleton generation
        SVDD_shape_skeleton;
    end
    
end

%%
%=========================================================
% Segmentation Merging
%=========================================================
if ctrl_param.KineStruct.motion_ON && ctrl_param.KineStruct.shape_ON
    
    %%
%     integration_cal_min_radius;
%     motion_seg_connection_geodesic;
    
    %%
    %     motion_seg_connection;
    
    %     do_seg_merge = 1;
    %     num_seg_history = [];
    %
    %     if do_seg_merge == 1
    %     while(do_seg_merge)
    %
    %         integration_cal_seg_rad;
    %         integration_seg_merge;
    %
    %         motion_seg_center_calculation;
    %         motion_seg_connection_geodesic;
    %     end
    %     else
    %         motion_seg_center_calculation;
    %         motion_seg_connection_geodesic;
    %     end
    
    %     SVDD_skeleton_weight_connection;
    %     SVDD_seg_connection;
    %
    %     integration_seg_connection;
    %     integration_radius_drawing;
    %     motion_seg_visualisation;
    %     connection_drawing_with_video;
end

%% Kinematic Structure
KineStruct.y = y;
KineStruct.W = W;
KineStruct.width = width;
KineStruct.height = height;
KineStruct.segGT = segGT;
KineStruct.seg_idx = seg_idx;
KineStruct.seg_center = seg_center;
KineStruct.num_seg = num_seg;
KineStruct.num_frames = num_frames;
KineStruct.num_points = num_points;
KineStruct.motion_dist = motion_dist;
KineStruct.affinity = motion_dist / max(max(motion_dist));
KineStruct.structure_i = motion_MST_ii;
KineStruct.structure_j = motion_MST_jj;
KineStruct.videoFileName = videoFileName;

%% Finding Joints
KineStruct = findingJoint(KineStruct,ctrl_param);

end