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
video_save_path = '/home/hjchang/Research/code/Matlab/cvpr2015/result/GT/';

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
    xyloObj = VideoReader(['/home/hjchang/OpenCV/test_codes/lk_track/',videoFileName]);
    
    switch(videoFileName)
        case {'arm.avi'}
            %             y(2,:,:) = height - y(2,:,:);
            init_frm = 1;
            end_frm = 30;
        case {'YanPollefeys.mp4'}
            y(2,:,:) = height - y(2,:,:);
            end_frm = 58;
        case {'baxter_cut.mp4'}
            y(2,:,:) = height - y(2,:,:);
            end_frm = 513;
        case {'hand1.mp4'}
            y(2,:,:) = height - y(2,:,:);
            end_frm = 659;
        otherwise
            y(2,:,:) = height - y(2,:,:);
    end
    
    nFrames = end_frm - init_frm + 1;
    width = xyloObj.Width;
    height = xyloObj.Height;
    
    clear mov
    mov(1:nFrames) = ...
        struct('cdata',zeros(height,width, 3,'uint8'),...
        'colormap',[]);
    
    for k = 1 : nFrames
        mov(k).cdata = read(xyloObj, k+init_frm-1);
    end
    
    writerobj_image = VideoWriter([video_save_path,videoFileName(1:end-4),'_GT_image']);
    open(writerobj_image);
    writerobj_white = VideoWriter([video_save_path,videoFileName(1:end-4),'_GT_white']);
    open(writerobj_white);
    writerobj_black = VideoWriter([video_save_path,videoFileName(1:end-4),'_GT_black']);
    open(writerobj_black);
    
    %%
    % connections
    % baxter
    switch(videoFileName)
        case {'baxter_cut.mp4'}
            connection_idx = [1,2 ; 2,3 ; 3,4 ; 4,5 ; 5,6 ; 5,7 ; 5,11 ; 7,8 ; 8,9 ; 9,10];
        case {'YanPollefeys.mp4'}
            connection_idx = [1,2 ; 2,3 ; 3,4 ; 4,5 ; 3,6];
        case {'toy.avi'}
            connection_idx = [1,2 ; 2,3];
        case {'robot_arm.mp4'}
            connection_idx = [1,2 ; 2,3 ; 3,4 ; 4,5 ; 5,7 ; 7,6 ; 8,6];
        case {'hand1.mp4'}
            %         connection_idx = [1,2 ; 2,3 ; 3,4 ; 4,5 ; 4,7 ; 7,8 ; 7,6 ; 8,9 ; 9,10 ; 10,11 ; 9,12 ; 12,13 ; 13,14 ; 12,15 ; 15,16 ; 16,17 ; 15,18 ; 18,19 ; 19,20];
            %             connection_idx = [1,2 ; 2,3 ; 3,4 ; 4,5 ; 4,7 ; 7,8 ; 18,6 ; 8,9 ; 9,10 ; 10,11 ; 9,12 ; 12,13 ; 13,14 ; 12,15 ; 15,16 ; 16,17 ; 15,18 ; 18,19 ; 19,20];
            connection_idx = [1,2 ; 2,7 ; 3,4 ; 4,5 ; 4,7 ; 7,8 ; 7,6 ; 8,9 ; 9,10 ; 10,11 ; 9,12 ; 12,13 ; 13,14 ; 12,15 ; 15,16 ; 16,17 ; 15,18 ; 18,19 ; 19,20];
        case {'iCub_hand.avi'}
            %             connection_idx = [1,2 ; 2,7 ; 7,8 ; 2,3 ; 3,4 ; 3,5 ; 5,6 ];
            connection_idx = [1,2 ; 2,7 ; 7,8 ; 2,5 ; 3,4 ; 3,5 ; 5,6 ];
        case {'iCub_motion.avi'}
            connection_idx = [1,2 ; 2,3 ; 3,4 ; 4,5 ; 3,6 ; 3,7];
        case {'arm.avi'}
            connection_idx = [1,2];
    end
    %%
    color_idx_white = 'rgbcmky';
    color_idx_black = 'rgbcmwy';
    marker_idx = '+o*xsd^v><ph';
    
    for frm_idx = 1:nFrames
        %%
        % segmentation
        num_seg_GT = max(s);
        seg_idx_GT = cell(num_seg_GT,1);
        seg_center_GT = zeros(2,num_seg_GT,num_frames);
        for i=1:num_seg_GT
            seg_idx_GT{i} = find(s==i);
        end
        
        for i=1:num_seg_GT
            seg_center_GT(1,i,frm_idx) = mean(y(1,seg_idx_GT{i},frm_idx));
            seg_center_GT(2,i,frm_idx) = mean(y(2,seg_idx_GT{i},frm_idx));
        end    
        
        %%
        % Image background
        
        h_image = figure(3);
        set(gcf,'color','k');
        clf
        imshow(mov(frm_idx).cdata);
        hold on
        axis([0 width 0 height]);
        axis off
        
        % drawing connections
        for m = 1:size(connection_idx,1)
            plot([seg_center_GT(1,connection_idx(m,1),frm_idx),seg_center_GT(1,connection_idx(m,2),frm_idx)],[height-seg_center_GT(2,connection_idx(m,1),frm_idx),height-seg_center_GT(2,connection_idx(m,2),frm_idx)],'-wo',...
                'LineWidth',4,...
                'MarkerSize',12,...
                'MarkerEdgeColor','w',...
                'MarkerFaceColor',[1.0,0.2,0.2]);
            %     plot([seg_center_GT(1,connection_idx(m,1),frm_idx),seg_center_GT(1,connection_idx(m,2),frm_idx)],[height - seg_center_GT(2,connection_idx(m,1),frm_idx),height - seg_center_GT(2,connection_idx(m,2),frm_idx)],'-wo',...
            %         'LineWidth',4,...
            %         'MarkerSize',12,...
            %         'MarkerEdgeColor','w',...
            %         'MarkerFaceColor',[1.0,0.2,0.2]);
        end
        F_image = getframe(h_image);
        writeVideo(writerobj_image,F_image);
        pause(0.03);
        %%
        % White background
        % feature points
        
        h_white = figure(1);
        set(gcf,'color','w');
        clf
        for i=1:num_seg_GT
            plot(y(1,seg_idx_GT{i},frm_idx), y(2,seg_idx_GT{i},frm_idx),marker_idx(mod(i,12)+1),'Color',color_idx_white(mod(i,7)+1));
            %     plot(y(1,seg_idx_GT{i},frm_idx), height-y(2,seg_idx_GT{i},frm_idx),marker_idx(mod(i,13)+1),'Color',color_idx(mod(i,7)+1));
            hold on
        end
        axis([0 width 0 height]);
        axis off
        % drawing connections
        for m = 1:size(connection_idx,1)
            plot([seg_center_GT(1,connection_idx(m,1),frm_idx),seg_center_GT(1,connection_idx(m,2),frm_idx)],[seg_center_GT(2,connection_idx(m,1),frm_idx),seg_center_GT(2,connection_idx(m,2),frm_idx)],'-ko',...
                'LineWidth',4,...
                'MarkerSize',12,...
                'MarkerEdgeColor','k',...
                'MarkerFaceColor',[1.0,0.2,0.2]);
            %     plot([seg_center_GT(1,connection_idx(m,1),frm_idx),seg_center_GT(1,connection_idx(m,2),frm_idx)],[height - seg_center_GT(2,connection_idx(m,1),frm_idx),height - seg_center_GT(2,connection_idx(m,2),frm_idx)],'-wo',...
            %         'LineWidth',4,...
            %         'MarkerSize',12,...
            %         'MarkerEdgeColor','w',...
            %         'MarkerFaceColor',[1.0,0.2,0.2]);
        end
        F_white = getframe(h_white);
        writeVideo(writerobj_white,F_white);
        
        %%
        % Black background
        % feature points
        
        h_black = figure(2);
        set(gcf,'color','k');
        clf
        for i=1:num_seg_GT
            plot(y(1,seg_idx_GT{i},frm_idx), y(2,seg_idx_GT{i},frm_idx),marker_idx(mod(i,12)+1),'Color',color_idx_black(mod(i,7)+1));
            %     plot(y(1,seg_idx_GT{i},frm_idx), height-y(2,seg_idx_GT{i},frm_idx),marker_idx(mod(i,13)+1),'Color',color_idx(mod(i,7)+1));
            hold on
        end
        axis([0 width 0 height]);
        axis off
        % drawing connections
        for m = 1:size(connection_idx,1)
            plot([seg_center_GT(1,connection_idx(m,1),frm_idx),seg_center_GT(1,connection_idx(m,2),frm_idx)],[seg_center_GT(2,connection_idx(m,1),frm_idx),seg_center_GT(2,connection_idx(m,2),frm_idx)],'-wo',...
                'LineWidth',4,...
                'MarkerSize',12,...
                'MarkerEdgeColor','w',...
                'MarkerFaceColor',[1.0,0.2,0.2]);
            %     plot([seg_center_GT(1,connection_idx(m,1),frm_idx),seg_center_GT(1,connection_idx(m,2),frm_idx)],[height - seg_center_GT(2,connection_idx(m,1),frm_idx),height - seg_center_GT(2,connection_idx(m,2),frm_idx)],'-wo',...
            %         'LineWidth',4,...
            %         'MarkerSize',12,...
            %         'MarkerEdgeColor','w',...
            %         'MarkerFaceColor',[1.0,0.2,0.2]);
        end
        F_black = getframe(h_black);
        writeVideo(writerobj_black,F_black);
    end
    close(writerobj_image);
    close(writerobj_white);
    close(writerobj_black);
end
