close all
clear all
clc

addpath /home/hjchang/Research/code/Matlab/cvpr2015/dataset/    % dataset path
%%
%=========================================================
% data load
%=========================================================
%-----------------------
% from mat files
%-----------------------
% [filename, pathname, filterindex] = uigetfile('/home/hjchang/Research/code/Matlab/cvpr2015/dataset/*.mat');

pathname = '/home/hjchang/Research/code/Matlab/cvpr2015/dataset/';
filename_buf = {'arm.avi.mat','toy.avi.mat','YanPollefeys.mp4.mat','iCub_motion.avi.mat','iCub_hand.avi.mat','robot_arm.mp4.mat','baxter_cut.mp4.mat','hand1.mp4.mat'};
% filename_buf = {'arm.avi.mat'};
% [filename, pathname, filterindex] = uigetfile('../dataset/*.mat');

for filename_idx = 1:size(filename_buf,2)
    
    filename = filename_buf{filename_idx}
    
    load([pathname,filename]);
    load(['../dataset/SVDD_data/',filename(1:end-4),'_SVDDdata.mat']);
    
    num_frames = size(y,3);
    num_points = size(y,2);
    %-------------------------------------
    
    videoFileName = filename(1:end-4);
    
    %%
    switch(videoFileName)
        
        case {'arm.avi'}
%             y(2,:,:) = height - y(2,:,:);
            init_frm = 1;
            end_frm = 30;
        case {'YanPollefeys.mp4'}
            end_frm = 58;
        case {'baxter_cut.mp4'}
            end_frm = 513;
        case {'hand1.mp4'}
            end_frm = 659;
    end
    
    %%
    video_save_path = '/home/hjchang/Research/code/Matlab/cvpr2015/result/SVDD/';
    
    %%
    % binary image black background
    writerobj = VideoWriter([video_save_path,videoFileName(1:end-4),'_binary_black']);
    open(writerobj);
    
    for frm_idx = 1:num_frames
        h = figure(1);
        set(gcf,'color','k');
        clf
        
        hold on
        % feature points
%         imshow(flipud(full_binary_image(:,:,frm_idx)));
        imshow(full_binary_image(:,:,frm_idx));
        axis([0 width 0 height]);
        axis off
        
        F = getframe(h);
        writeVideo(writerobj,F);
        pause(0.03);
    end
    close(writerobj);
    
    %%
    % binary image white background
    writerobj = VideoWriter([video_save_path,videoFileName(1:end-4),'_binary_white']);
    open(writerobj);
    
    for frm_idx = 1:num_frames
        h = figure(2);
        set(gcf,'color','w');
        clf
        
        hold on
        % feature points
%         imshow(255-flipud(full_binary_image(:,:,frm_idx)));
        imshow(255-full_binary_image(:,:,frm_idx));
        axis([0 width 0 height]);
        axis off
        
        F = getframe(h);
        writeVideo(writerobj,F);
        pause(0.03);
    end
    close(writerobj);
    
    %%
    % binary image black background
    writerobj = VideoWriter([video_save_path,videoFileName(1:end-4),'_map_black']);
    open(writerobj);
    
    for frm_idx = 1:num_frames
        % Skeleton
        h = figure(3);
        set(gcf,'color','k');
        clf
        %     subplot(2,2,3)
        buf = (full_radius_map(:,:,frm_idx).^(1/2)/max(max(full_radius_map(:,:,frm_idx).^(1/2))));
%         buf = flipud(buf);
        imagesc(buf);
        colormap jet
        % axis image off
        % freezeColors
        
        hold on
        for i=1:size(y,2)
            plot(y(1,i,frm_idx),y(2,i,frm_idx),'Marker','.','Color',[0.3,0.3,0.3]);
        end
        
%         buf_binary = flipud(full_binary_image(:,:,frm_idx));        
        buf_binary = full_binary_image(:,:,frm_idx);        
        [bg_idx_h,bg_idx_w] = find(buf_binary == 0);
        plot(bg_idx_w,bg_idx_h, 'k.');
        
%         buf_skeleton = flipud(full_skeleton_binary_map(:,:,frm_idx));
        buf_skeleton = full_skeleton_binary_map(:,:,frm_idx);
        [sk_idx_h,sk_idx_w] = find(buf_skeleton == 1);
        plot(sk_idx_w,sk_idx_h, 'w.');
        
        
        axis([0 width 0 height]);
        axis off
        
        F = getframe(h);
        writeVideo(writerobj,F);
        pause(0.03);
        
    end
    close(writerobj);
    
    %%
    % binary image white background
    writerobj = VideoWriter([video_save_path,videoFileName(1:end-4),'_map_white']);
    open(writerobj);
    
    for frm_idx = 1:num_frames
        % Skeleton
        h = figure(4);
        set(gcf,'color','w');
        clf
        %     subplot(2,2,3)
        buf = (full_radius_map(:,:,frm_idx).^(1/2)/max(max(full_radius_map(:,:,frm_idx).^(1/2))));
%         buf = flipud(buf);
        imagesc(buf);
        colormap jet
        % axis image off
        % freezeColors
        
        hold on
        for i=1:size(y,2)
            plot(y(1,i,frm_idx),y(2,i,frm_idx),'Marker','.','Color',[0.3,0.3,0.3]);
        end
        
%         buf_binary = flipud(full_binary_image(:,:,frm_idx));        
        buf_binary = full_binary_image(:,:,frm_idx);        
        [bg_idx_h,bg_idx_w] = find(buf_binary == 0);
        plot(bg_idx_w,bg_idx_h, 'w.');
        
%         buf_skeleton = flipud(full_skeleton_binary_map(:,:,frm_idx));
        buf_skeleton = full_skeleton_binary_map(:,:,frm_idx);
        [sk_idx_h,sk_idx_w] = find(buf_skeleton == 1);
        plot(sk_idx_w,sk_idx_h, 'k.');
        
        
        axis([0 width 0 height]);
        axis off
        
        F = getframe(h);
        writeVideo(writerobj,F);
        pause(0.03);
        
    end
    close(writerobj);
end
