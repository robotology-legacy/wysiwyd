% ==========================================================================
% Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
% Authors: Hyung Jin Chang
% email:   (hj.chang@imperial.ac.uk)
% Permission is granted to copy, distribute, and/or modify this program
% under the terms of the GNU General Public License, version 2 or any
% later version published by the Free Software Foundation.
% 
% A copy of the license can be found at
% wysiwyd/license/gpl.txt
% 
% This program is distributed in the hope that it will be useful, but
% WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
% Public License for more details
% ==========================================================================

% help
disp('This is an extraction feature points and tracking using Lukas-Kanade optical flow.\n');

%%
% Parameter
MAX_COUNT = 500;
needToInit = true;
nightMode = false;
saveMode = false;
videoMode = 1;  % 0: webcam, 1: video

detect_interval = 5;
frame_idx = 0;
num_init_points = 0;

prev_points = [];
cur_points = [];

image = [];
cur_Gray = [];
prev_Gray = [];
mask = [];

subPixWinSize = [10,10];
termcrit = struct('type','Count+EPS', 'maxCount',20, 'epsilon',0.03);

if videoMode == 1
    videoName = 'Video';
else
    
    videoName = 'Webcam';
end

disp(['Loading video from ',videoName]);

%%
%=========================================================
% data load
%=========================================================
switch videoName
    case 'Video'
        
        %-----------------------
        % from video
        %-----------------------
        [filename, pathname, filterindex] = uigetfile();
        [pathname,filename]
        videoObj = VideoReader([pathname,filename]);
        nFrames = videoObj.NumberOfFrames;
        width = videoObj.Width;
        height = videoObj.Height;
        
    case 'Webcam'
        %-----------------------
        % From Webcam
        % Set up camera
        %-----------------------
        videoObj = cv.VideoCapture();
        
end
pause(1);

% Set up display window
window = figure('KeyPressFcn',@(obj,evt)setappdata(obj,'flag',true));
setappdata(window,'flag',false);

%%
y_total = -1*ones(MAX_COUNT,2);

%%
% Start main loop
frm_idx = 1;
while true
    
    frm_idx = frm_idx + 1;    
    switch videoName
        case 'Video'
            if frm_idx > nFrames
                break;
            else
                % from video
                image = read(videoObj,frm_idx);
            end
            
        case 'Webcam'
            % Grab and preprocess an image
            image = videoObj.read;
    end
    %     image = cv.resize(image,0.5);
    image_height = size(image,1);
    image_width = size(image,2);
    cur_Gray = cv.cvtColor(image, 'RGB2GRAY');
    
    if nightMode
        image = zeros(image_height, image_width);
    end
    
    % Draw results
    clf
    imshow(image);
    hold on;
    
    if needToInit
        cur_points = cv.goodFeaturesToTrack(cur_Gray, ...
            'MaxCorners', MAX_COUNT, ...
            'QualityLevel',0.01,...
            'MinDistance',3,...
            'BlockSize',3,...
            'K',0.04);
        cur_points = cv.cornerSubPix(cur_Gray,cur_points,...
            'WinSize', subPixWinSize,...
            'ZeroZone', [-1,-1],...
            'Criteria', termcrit);
        num_init_points = numel(cur_points);
        points_idx = [1:num_init_points];
        needToInit = false;
        
        buf = cell2mat(cur_points);
        points_buf = reshape(buf,[2,num_init_points]);
        plot(points_buf(1,:),points_buf(2,:),'go');
        
        disp('Initial feature extraction done!');
        
    elseif ~isempty(prev_points)
        
        if isempty(prev_Gray)
            prev_Gray = cur_Gray;
        end
        
        [cur_points, status, err] = cv.calcOpticalFlowPyrLK(prev_Gray, cur_Gray, prev_points, 'WinSize', [31,31],...
            'Criteria', termcrit,...
            'GetMinEigenvals', 0,...
            'MinEigThreshold', 0.001);
        %         [cur_points, status, err] = cv.calcOpticalFlowPyrLK(prev_Gray, cur_Gray, prev_points);
        
        k=1;
        for i=1:numel(cur_points)
            if ~status(i)
                continue;
            end
            
            points_idx(k) = points_idx(i);
            cur_points(k) = cur_points(i);
            k = k + 1;
        end
        
        buf = cell2mat(cur_points);
        points_buf = reshape(buf,[2,size(buf,2)/2]);
        plot(points_buf(1,:),points_buf(2,:),'go');
        cur_points = cur_points(1:k-1);
        points_idx = points_idx(1:k-1);
    end
    
    prev_Gray = cur_Gray;
    prev_points = cur_points;
    
    % Terminate if any user input
    flag = getappdata(window,'flag');
    if isempty(flag)||flag, break; end
    
    pause(0.01);
end

% Close
close(window);
