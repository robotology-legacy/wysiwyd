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
% filename_buf = {'hand1.mp4.mat'};
% [filename, pathname, filterindex] = uigetfile('../dataset/*.mat');

for filename_idx = 1:size(filename_buf,2)
    
    filename = filename_buf{filename_idx}
    
    load([pathname,filename]);
    num_frames = size(y,3);
    num_points = size(y,2);
    %-------------------------------------
    
    videoFileName = filename(1:end-4);
    
    %%
    switch(videoFileName)
        
        case {'arm.avi'}
            y(2,:,:) = height - y(2,:,:);
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
    % load init image
    xyloObj = VideoReader(['/home/hjchang/OpenCV/test_codes/lk_track/',videoFileName]);

    video_save_path = '/home/hjchang/Research/code/Matlab/cvpr2015/result/original_video/';
    
    writerobj = VideoWriter([video_save_path,videoFileName(1:end-4)]);
    open(writerobj);
    
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
    
    hf = figure(10);
    set(gcf,'color','k');     
    set(hf,'position',[150 150 width height]);
    
    for frm_idx = 1:nFrames
        hf = figure(10);
        clf
        imshow(mov(frm_idx).cdata);
        
        F = getframe(hf);
        writeVideo(writerobj,F);
        pause(0.1);
        
    end
    close(writerobj);
end
