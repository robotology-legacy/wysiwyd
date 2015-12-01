close all
clear all
clc

%%
%=========================================================
% data load
%=========================================================
%-----------------------
% from mat files
%-----------------------
[filename, pathname, filterindex] = uigetfile('/home/hjchang/Research/code/Matlab/cvpr2016/dataset/cvpr2015/*.*');

%-------------------------------------
videoFileName = filename(1:end-4);

%%
% load init image
xyloObj = VideoReader([pathname,filename]);

nFrames = xyloObj.NumberOfFrames;
width = xyloObj.Width;
height = xyloObj.Height;

for k = 1 : nFrames
    mov(k).cdata = read(xyloObj, k);
end

hf = figure(10);
set(gcf,'color','k');

accu_figure_buf = zeros(height, width);


for frm_idx = 1:10:nFrames
    frm_idx
    hf = figure(10);
    clf
    
    accu_figure_buf = (accu_figure_buf + double(rgb2gray(mov(frm_idx).cdata))) / 2;
    
    imshow(uint8(accu_figure_buf));
    
    pause(0.1);
end

