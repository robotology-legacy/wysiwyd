%%
%=========================================================
% data load
%=========================================================
%-----------------------
% from mat files
%-----------------------
clc
clear all
close all

pathname = '/home/hjchang/Research/code/Matlab/cvpr2015/dataset/';
filename_buf = {'arm.avi.mat','toy.avi.mat','YanPollefeys.mp4.mat','iCub_motion.avi.mat','iCub_hand.avi.mat','robot_arm.mp4.mat','baxter_cut.mp4.mat','hand1.mp4.mat'};
% filename_buf = {'hand1.mp4.mat'};
% [filename, pathname, filterindex] = uigetfile('../dataset/*.mat');

%%
video_save_path = '/home/hjchang/Research/code/Matlab/cvpr2015/result/proposed_result_video/';

load('/home/hjchang/Research/code/Matlab/cvpr2015/result/proposed_Euclidean/trial_3/workspace.mat');

for filename_idx = 1:size(filename_buf,2)
    
    filename = filename_buf{filename_idx};
    
    load([pathname,filename]);
    num_frames = size(y,3);
    num_points = size(y,2);
    %-------------------------------------
    W = zeros(2*num_frames, num_points);
    
    for f = 1:num_frames
        W(f,:) = y(1,:,f);
        W(f+num_frames,:) = y(2,:,f);
    end
    
    videoFileName = filename(1:end-4);
    
    %%
    % connections
    % baxter
    switch(videoFileName)
        case {'arm.avi'}
            %             y(2,:,:) = height - y(2,:,:);
            %             seg_center(2,:,:) = height - seg_center(2,:,:);
            iter_idx_total = [4];
        case {'toy.avi'}
            y(2,:,:) = height - y(2,:,:);
            iter_idx_total = [2];
        case {'YanPollefeys.mp4'}
            y(2,:,:) = height - y(2,:,:);
            iter_idx_total = [89,55,44,25,8,98,72,68];
        case {'iCub_motion.avi'}
            y(2,:,:) = height - y(2,:,:);
            iter_idx_total = [83,55,86,93,76,87,80,58,53,54,6,33,35,18];
        case {'iCub_hand.avi'}
            y(2,:,:) = height - y(2,:,:);
            iter_idx_total = [66,39];
        case {'robot_arm.mp4'}
            y(2,:,:) = height - y(2,:,:);
            iter_idx_total = [30,12,17,10,96,83,61];
        case {'baxter_cut.mp4'}
            y(2,:,:) = height - y(2,:,:);
            iter_idx_total = [24,66,60,42];
        case {'hand1.mp4'}
            y(2,:,:) = height - y(2,:,:);
            iter_idx_total = [93,19,24,11,36];
    end
    
    for iter_idx = iter_idx_total
        
        seg_center = seg_center_total{iter_idx,filename_idx};
        seg_idx_buf = seg_idx_total{iter_idx,filename_idx};
        
        empty_check_idx = 1;
        while(1)
            if(isempty(seg_idx_buf{empty_check_idx}))
                break;
            end
            empty_check_idx = empty_check_idx + 1;
        end
        seg_idx = seg_idx_buf{empty_check_idx-1};
        num_joint = size(seg_center,2);
        num_seg = size(seg_idx,1);
        
        switch(videoFileName)
            case {'arm.avi'}
                seg_center(2,:,:) = height - seg_center(2,:,:);
        end
        
        
        motion_seg_connect_plot_ON = 0;
        motion_seg_connection;
        
        %%
        %         writerobj_white = VideoWriter([video_save_path,videoFileName(1:end-4),'_result_white_',num2str(iter_idx)]);
        %         open(writerobj_white);
        writerobj_black = VideoWriter([video_save_path,videoFileName(1:end-4),'_result_black',num2str(iter_idx)]);
        open(writerobj_black);
        
        %%
        color_idx_white = 'rgbcmky';
        color_idx_black = 'rgbcmwy';
        marker_idx = '+o*xsd^v><ph';
        
        for frm_idx = 1:num_frames
            %             %%
            %             % White background
            %             % feature points
            %
            %             h_white = figure(1);
            %             set(gcf,'color','w');
            %             clf
            %             for i=1:num_seg
            %                 plot(y(1,seg_idx{i},frm_idx), y(2,seg_idx{i},frm_idx),marker_idx(mod(i,12)+1),'Color',color_idx_white(mod(i,7)+1));
            %                 %     plot(y(1,seg_idx{i},frm_idx), height-y(2,seg_idx{i},frm_idx),marker_idx(mod(i,13)+1),'Color',color_idx(mod(i,7)+1));
            %                 hold on
            %             end
            %             axis([0 width 0 height]);
            %             axis off
            %             % drawing connections
            %             for m = 1:size(connection_idx,1)
            %                 plot([seg_center(1,connection_idx(m,1),frm_idx),seg_center(1,connection_idx(m,2),frm_idx)],[seg_center(2,connection_idx(m,1),frm_idx),seg_center(2,connection_idx(m,2),frm_idx)],'-ko',...
            %                     'LineWidth',4,...
            %                     'MarkerSize',12,...
            %                     'MarkerEdgeColor','k',...
            %                     'MarkerFaceColor',[1.0,0.2,0.2]);
            %                 %     plot([seg_center(1,connection_idx(m,1),frm_idx),seg_center(1,connection_idx(m,2),frm_idx)],[height - seg_center(2,connection_idx(m,1),frm_idx),height - seg_center(2,connection_idx(m,2),frm_idx)],'-wo',...
            %                 %         'LineWidth',4,...
            %                 %         'MarkerSize',12,...
            %                 %         'MarkerEdgeColor','w',...
            %                 %         'MarkerFaceColor',[1.0,0.2,0.2]);
            %             end
            %             F_white = getframe(h_white);
            %             writeVideo(writerobj_white,F_white);
            
            %%
            % Black background
            % feature points
            
            h_black = figure(2);
            set(gcf,'color','k');
            clf
            for i=1:num_seg
                plot(y(1,seg_idx{i},frm_idx), y(2,seg_idx{i},frm_idx),marker_idx(mod(i,12)+1),'Color',color_idx_black(mod(i,7)+1));
                %     plot(y(1,seg_idx{i},frm_idx), height-y(2,seg_idx{i},frm_idx),marker_idx(mod(i,13)+1),'Color',color_idx(mod(i,7)+1));
                hold on
            end
            axis([0 width 0 height]);
            axis off
            % drawing connections
            for m = 1:size(motion_MST_ii,1)
                plot([seg_center(1,motion_MST_ii(m),frm_idx),seg_center(1,motion_MST_jj(m),frm_idx)],...
                    [height-seg_center(2,motion_MST_ii(m),frm_idx),height-seg_center(2,motion_MST_jj(m),frm_idx)],'-wo',...
                    'LineWidth',4,...
                    'MarkerSize',12,...
                    'MarkerEdgeColor','w',...
                    'MarkerFaceColor',[1.0,0.2,0.2]);
                
            end
            F_black = getframe(h_black);
            writeVideo(writerobj_black,F_black);
        end
        %         close(writerobj_white);
        close(writerobj_black);
    end
end
