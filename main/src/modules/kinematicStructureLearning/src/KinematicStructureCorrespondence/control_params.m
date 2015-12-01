
%--------------------------------------------------------
% motion module control
ctrl_param.KineStruct.motion_ON = true;
% ctrl_param.KineStruct.motion_ON = false;

ctrl_param.KineStruct.motion_GT_load_ON = true;
% ctrl_param.KineStruct.motion_GT_load_ON = false;

% ctrl_param.KineStruct.motion_seg_plot_ON = true;
ctrl_param.KineStruct.motion_seg_plot_ON = false;

% ctrl_param.KineStruct.motion_seg_connect_plot_ON = true;
ctrl_param.KineStruct.motion_seg_connect_plot_ON = false;

% ctrl_param.KineStruct.motion_seg_connect_plot_with_video_ON = true;
ctrl_param.KineStruct.motion_seg_connect_plot_with_video_ON = false;

% ctrl_param.KineStruct.motion_seg_connect_color_plot_ON = true;
ctrl_param.KineStruct.motion_seg_connect_color_plot_ON = false;

%--------------------------------------------------------
% shape module control
% ctrl_param.KineStruct.shape_ON = true;
ctrl_param.KineStruct.shape_ON = false;

% ctrl_param.KineStruct.shape_SVDD_training_plot_ON = true;
ctrl_param.KineStruct.shape_SVDD_training_plot_ON = false;

% ctrl_param.KineStruct.shape_SVDD_load_ON = true;
ctrl_param.KineStruct.shape_SVDD_load_ON = false;

% ctrl_param.KineStruct.shape_SVDD_load_plot_ON = true;
ctrl_param.KineStruct.shape_SVDD_load_plot_ON = false; 

ctrl_param.KineStruct.shape_SVDD_result_save_ON = ~ctrl_param.KineStruct.shape_SVDD_load_ON;

%--------------------------------------------------------
% ctrl_param.KineStruct.integration_seg_connect_plot_ON = true;
ctrl_param.KineStruct.integration_seg_connect_plot_ON = false;

%--------------------------------------------------------
% ctrl_param.KineStruct.joint_connect_plot_ON = true;
ctrl_param.KineStruct.joint_connect_plot_ON = false;

%--------------------------------------------------------
% result save
ctrl_param.KineStruct.result_save_ON = false;
