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

%%
%=========================================================
% data load
%=========================================================

cdata = struct('filename',[],...
    'pathname',[],...
    'nFrames',[],...
    'width',[],...
    'height',[],...
    'feature_point_save_path',[],...
    'points_total',[]);

switch videoName
    case 'webcam'
        %-----------------------
        % From Webcam
        % Set up camera
        %-----------------------
        disp('Connecting to the webcam...');
        cdata.nFrames = 500;
        camera = cv.VideoCapture;
        pause(2);
    case 'video'
        %-----------------------
        % from video
        %-----------------------
        disp('Loading image sequences ...');
        
        [cdata.filename, cdata.pathname, filterindex] = uigetfile(...
            {'*.avi;*.mpg;*.mpeg','Video Files (*.avi,*.mpg,*.mpeg)';
            '*.*',  'All Files (*.*)'}, ...
            'Select a video file');
        disp([cdata.pathname,cdata.filename]);
        videoObj = VideoReader([cdata.pathname,cdata.filename]);
        cdata.nFrames = videoObj.NumberOfFrames;
        cdata.width = videoObj.Width;
        cdata.height = videoObj.Height;
        
        cdata.feature_point_save_path = ['data/KS/points/',cdata.filename(1:end-4)];
        [SUCCESS,MESSAGE,MESSAGEID] = mkdir(cdata.feature_point_save_path);
        
    case 'YARP'
        %-----------------------
        % From YARP
        %-----------------------
        wrapper_YARP_ABM_retrieve_jointProjection;
        
        pause(0.1)
                 
        cdata.filename = 'video.avi';
        cdata.pathname = 'data/KS/';
        videoObj = VideoReader([cdata.pathname,cdata.filename]);
        cdata.nFrames = videoObj.NumberOfFrames;
        cdata.width = videoObj.Width;
        cdata.height = videoObj.Height;        
        
        cdata.feature_point_save_path = ['data/KS/points/',cdata.filename(1:end-4)];
        [SUCCESS,MESSAGE,MESSAGEID] = mkdir(cdata.feature_point_save_path);        
        
    case 'images'
        %-----------------------
        % From images
        %-----------------------
        disp('Please select a folder ...');
        img2video;
        pause(0.1);
        
        disp('Loading image sequences ...');
        
        cdata.pathname = [pwd,'/data/'];
        cdata.filename = 'video.avi';
        videoObj = VideoReader([cdata.pathname,cdata.filename]);
        cdata.nFrames = videoObj.NumberOfFrames;
        cdata.width = videoObj.Width;
        cdata.height = videoObj.Height;
        
        cdata.feature_point_save_path = ['data/KS/points/',cdata.filename(1:end-4)];
        [SUCCESS,MESSAGE,MESSAGEID] = mkdir(feature_point_save_path);
        
end

%%
% % Set up display window
window = figure('KeyPressFcn',@(obj,evt)setappdata(obj,'flag',true));
setappdata(window,'flag',false);

%
disp('++++++++++++++++++++++++++++++++++++++++++++++');
disp('Loading parameters for feature extraction.\n');
feature_extraction_parameters;

%
disp('++++++++++++++++++++++++++++++++++++++++++++++');
disp('Extracting feature points and tracking using Lukas-Kanade optical flow.');
y_total = -1*ones(MAX_COUNT,2);
cdata.points_total = cell(cdata.nFrames,1);

% Start main loop
frm_idx = 0;
while true
    
    frm_idx = frm_idx + 1;
    switch videoName
        case 'webCam'
            % Grab and preprocess an image
            image = camera.read;
        case 'video'
            if frm_idx > cdata.nFrames
                break;
            else
                % from video
                image = read(videoObj,frm_idx);
            end
        case 'images'
            if frm_idx > cdata.nFrames
                break;
            else
                % from video
                image = read(videoObj,frm_idx);
            end
        case 'YARP'
            if frm_idx > cdata.nFrames
                break;
            else
                % from video
                image = read(videoObj,frm_idx);
            end
    end
    %     image = cv.resize(image,0.5);
    image_height = size(image,1);
    image_width = size(image,2);

    
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     % smoothing
%     myfilter = fspecial('gaussian',[5 5], 0.5);
%     image = imfilter(image, myfilter, 'replicate');    
% 
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % remove background
    imageR = image(:,:,1);
    imageG = image(:,:,2);
    imageB = image(:,:,3);
    thres = 30;
    imageR(find(imageR < thres)) = 0;
    imageG(find(imageG < thres)) = 0;
    imageB(find(imageB < thres)) = 0;
    image(:,:,1) = imageR;
    image(:,:,2) = imageG;
    image(:,:,3) = imageB;
% %         
% %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %     % noise reduction
%     for layer_idx = 1:3
% %         image(:,:,layer_idx) = imadjust(image(:,:,layer_idx));
%         image(:,:,layer_idx) = medfilt2(image(:,:,layer_idx));
% % %         image(:,:,layer_idx) = wiener2(image(:,:,layer_idx),[5,5]);
% % 
%     end
    
    cur_Gray = cv.cvtColor(image, 'RGB2GRAY');
    
    if nightMode
        image = zeros(image_height, image_width);
    end    
    
    % Draw results
    clf
    imshow(image);
    hold on;
    
    if needToInit
%         cur_points = cv.goodFeaturesToTrack(cur_Gray, ...
%             'MaxCorners', MAX_COUNT, ...
%             'QualityLevel',0.01,...
%             'MinDistance',5,...
%             'BlockSize',5,...
%             'K',0.04);
        cur_points = cv.goodFeaturesToTrack(cur_Gray, ...
            'MaxCorners', MAX_COUNT, ...
            'QualityLevel',0.01,...
            'MinDistance',5,...
            'BlockSize',5,...
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
        
        disp('..... Initial feature extraction done!');
        
        %         points_total_buf = points_buf';
        
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
                %                 points_total_buf(i,:) = [-1,-1];
                continue;
            end
            
            points_idx(k) = points_idx(i);
            cur_points(k) = cur_points(i);
            %             points_total_buf(i,:) = cur_points{i};
            k = k + 1;
        end
        
        buf = cell2mat(cur_points);
        num_points = size(buf,2)/2;
        points_buf = reshape(buf,[2,num_points]);
        plot(points_buf(1,:),points_buf(2,:),'go');
        cur_points = cur_points(1:k-1);
        points_idx = points_idx(1:k-1);
    end
    
    % give -1 index to lost feature points
    k=1;
    for j=1:num_init_points        
        if numel(points_idx) >= k
            if j==points_idx(k)
                points_total_buf(j,:) = points_buf(:,k)';
                k = k + 1;
            else
                points_total_buf(j,:) = [-1,-1];
            end
        end
    end
    
    cdata.points_total{frm_idx} = points_total_buf;
    
    % swap data
    prev_Gray = cur_Gray;
    prev_points = cur_points;
    
    % Terminate if any user input
    flag = getappdata(window,'flag');
    if isempty(flag)||flag, break; end
    
    pause(0.01);
end

%%
% Feature data save
for frm_idx = 1:cdata.nFrames
    output_file = sprintf([cdata.feature_point_save_path,'/point_seq_%d.txt'],frm_idx);
    fid = fopen(output_file,'wt');
    for i = 1:num_init_points
        buf = cdata.points_total{frm_idx};
        fprintf(fid,'[%f,%f]\n',buf(i,1),buf(i,2));
    end
end
disp('..... Saving feature points completed!')

%%
% Close
disp('..... Feature extraction & saving is finished!');
pause(0.1);
close(window);
disp('++++++++++++++++++++++++++++++++++++++++++++++');
