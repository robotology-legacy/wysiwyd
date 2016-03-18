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
%
% Image sequence retrieving for post processing (e.g. feature extraction) from YARP ABM
% This code is implemented based on FastImageConversion.m
% ==========================================================================

% clean and make a saving directory
[SUCCESS,MESSAGE,MESSAGEID] = rmdir('ABM','s');
mkdir('ABM/P');
mkdir('ABM/Q');

%%
%-------------------------------------------------------------------------------------------
% Port P
%-------------------------------------------------------------------------------------------
%====================================
% Connecting Ports
%====================================
%creating ports
port2ABM_query_P = Port;          %port for ABM of 1st data

%first close the port just in case
%(this is to try to prevent matlab from beuing unresponsive)
port2ABM_query_P.close;

%open the ports
disp('_____opening ports...');
port2ABM_query_P.open('/matlab/kinematicStructure/1st_data_read');
disp('_____opened port /matlab/kinematicStructure/1st_data_read');
pause(0.5);
disp('_____done.');

disp('Going to open port /matlab/kinematicStructure/1st_data_read');
connection_check = 0;
while(~connection_check)
    Network.connect('/matlab/kinematicStructure/1st_data_read', '/autobiographicalMemory/rpc');
    connection_check = Network.isConnected('/matlab/kinematicStructure/1st_data_read','/autobiographicalMemory/rpc');
    pause(0.5)
    disp('waiting for connection...');
end
disp('1st Port is connected to ABM!')

%%
%====================================
% Receive Data & Save Images
%====================================
% get number of frames: "getImagesInfo"
if strcmp(data_source_P,'left') || strcmp(data_source_P,'right')
    portIncoming_P = BufferedPortImageRgb;
    %     portIncoming_P = Port;
elseif strcmp(data_source_P,'kinect')
    %     portIncoming_P = Bottle;
    portIncoming_P = Port;
end

portIncoming_P.close;

disp('_____opening ports...');
portIncoming_P.open('/matlab/kinematicStructure/1st_datain');
disp('_____opened port /matlab/kinematicStructure/1st_datain');
pause(0.5);
disp('_____done.');

disp('Going to open port /matlab/kinematicStructure/1st_datain');


b_write_P = yarp.Bottle;
b_response_P = yarp.Bottle;

command_P = ['triggerStreaming ', num2str(instance_num_P), ' ("includeAugmented" 0) ("realtime" 1) ("speedMultiplier" 0.1)'];
b_write_P.fromString(command_P);
disp(b_write_P);
port2ABM_query_P.write(b_write_P,b_response_P);
% pause(3);
disp(b_response_P.toString);

b_provide_P = yarp.Bottle;

if strcmp(data_source_P,'left') || strcmp(data_source_P,'right')
    b_provide_P = b_response_P.get(2).asList();
elseif strcmp(data_source_P,'kinect')
    b_provide_P = b_response_P.get(3).asList();
end

disp(b_provide_P.toString);

% for P
for i=0:b_provide_P.size()-1
    str_buf = b_provide_P.get(i).asList().get(0).asString();
    char_buf = char(str_buf);
    
    % for camcalib/left or camcalib/right
    if strcmp(data_source_P,'left') || strcmp(data_source_P,'right')
        
        if strcmp(char_buf(1:end-18), ['/autobiographicalMemory/icub/camcalib/',data_source_P,'/out'])
            %         if strcmp(char_buf(1:end-18), '/autobiographicalMemory/grabber')
            num_images_P = b_provide_P.get(i).asList().get(1).asInt();
            final_portname_P = char_buf;
        end
        % for kinect
    elseif strcmp(data_source_P,'kinect')
        if strcmp(char_buf(1:end), '/autobiographicalMemory/agentDetector/skeleton:o')
            num_images_P = b_provide_P.get(i).asList().get(1).asInt();
            final_portname_P = char_buf;
        end
    end
end

disp(final_portname_P);
disp(num2str(num_images_P));

connection_check = 0;
while(~connection_check)
    Network.connect(final_portname_P, '/matlab/kinematicStructure/1st_datain');
    connection_check = Network.isConnected(final_portname_P, '/matlab/kinematicStructure/1st_datain');
    %     pause(0.5)
    disp('waiting for connection...');
end
disp('1st Port Connected!')

%%
disp('================================');
disp('Start retrieving 1st data!');

bDataMeta_P_buf = cell(num_images_P,1);
bRawImage_P_buf = cell(num_images_P,1);

start_frame_idx_P = 1;
last_frame_idx_P = num_images_P;

reverseStr = '';
% h = waitbar(0,'Initializing progress bar...');

%------------------------------------------
% for camcalib/left or camcalib/right
if strcmp(data_source_P,'left') || strcmp(data_source_P,'right')
    for i=0:ceil(num_images_P*0.9)-1
        
        %                 disp(['receive image ', num2str(i),'/',num2str(num_images_P-5)]);
        percentDone = 100 * (i+1) / (ceil(num_images_P*0.9));
        msg = sprintf('Percent done: %3.1f', percentDone); %Don't forget this semicolon
        fprintf([reverseStr, msg]);
        reverseStr = repmat(sprintf('\b'), 1, length(msg));
        %         waitbar(i/(num_images_P-4),h,sprintf('%d%% along...',floor(i/(num_images_P-4)*100)))
        
        yarpData_P = yarp.ImageRgb;
        yarpData_P = portIncoming_P.read();
        
        if(i+1 >= start_frame_idx_P && i < last_frame_idx_P)
            % save images
            filename = [sprintf('%04d',i) '.png'];
            save_path = ['ABM/P/',filename];
            
            if (sum(size(yarpData_P)) ~= 0) %check size of bottle
                %                 disp('got it..');
                h=yarpData_P.height;
                w=yarpData_P.width;
                pixSize=yarpData_P.getPixelSize();
                tool=YarpImageHelper(h, w);
                IN = tool.getRawImg(yarpData_P); %use leo pape image patch
                TEST = reshape(IN, [h w pixSize]); %need to reshape the matrix from 1D to h w pixelSize
                matlabImage=uint8(zeros(h, w, pixSize)); %create an empty image with the correct dimentions
                r = cast(TEST(:,:,1),'uint8');  % need to cast the image from int16 to uint8
                g = cast(TEST(:,:,2),'uint8');
                b = cast(TEST(:,:,3),'uint8');
                matlabImage(:,:,1)= r; % copy the image to the previoulsy create matrix
                matlabImage(:,:,2)= g;
                matlabImage(:,:,3)= b;
            else
                disp('incorrect image');
            end
            imwrite(matlabImage,save_path);
            env = yarp.Bottle;
            portIncoming_P.getEnvelope(env);
            bDataMeta_P_buf{i+1,1} = env.toString();
            pause(0.01);
        end
    end
    disp('');
    disp('Done receiving images!');
    %     close(h);
    %------------------------------------------
    % for kinect
elseif strcmp(data_source_P,'kinect')
    fileID = fopen('ABM/P/joints.txt','w');
    yarpData_P = yarp.Bottle;
    portIncoming_P.read(yarpData_P);
    
    for i=0:floor(num_images_P/16)-2
        disp(['receive data of frame ', num2str(i)]);
        if(i+1 >= start_frame_idx_P && i < last_frame_idx_P)
            % save data
            if (sum(size(yarpData_P)) ~= 0) %check size of bottle
                for body_idx = 0:15
                    buf = char(yarpData_P.get(0).asList().get(body_idx).toString());
                    readData = str2double(strsplit(' ',buf));
                    x = readData(2);
                    y = readData(3);
                    z = readData(4);
                    fprintf(fileID, '%d\t %d\t %6.6f\t %6.6f\t %6.6f\n', i+1, body_idx, x, y, z);
                end
            else
                disp('incorrect data');
            end
            
            env = yarp.Bottle;
            portIncoming_P.getEnvelope(env);
            bDataMeta_P_buf{i+1,1} = env.toString();
        end
    end
    fclose(fileID);
end

% save buf data
save('ABM/P/DataMeta.mat','bDataMeta_P_buf');

%%
%====================================
% Generate Video
%====================================
%------------------------------------------
% for camcalib/left or camcalib/right
if strcmp(data_source_P,'left') || strcmp(data_source_P,'right')
    imagesFolder_P='ABM/P';
    imageFiles_P = dir(strcat(imagesFolder_P,'/*.png'));
    num_images_P = size(imageFiles_P,1);
    
    fileIdx_P = zeros(num_images_P,1);
    VideoFile_P = strcat(imagesFolder_P,'/video');
    writerObj_P = VideoWriter(VideoFile_P);
    
    fps= 10;
    writerObj_P.FrameRate = fps;
    open(writerObj_P);
    for t = 1:length(imageFiles_P)
        Frame = imread(strcat(imagesFolder_P,'/',imageFiles_P(t).name));
        writeVideo(writerObj_P,im2frame(Frame));
    end
    close(writerObj_P);
    %------------------------------------------
    % for kinect
elseif strcmp(data_source_P,'kinect')
end

%%
%====================================
% Close Ports
%====================================
port2ABM_query_P.close;
portIncoming_P.close;

%%
%-------------------------------------------------------------------------------------------
% Port Q
%-------------------------------------------------------------------------------------------
%====================================
% Connecting Ports
%====================================
%creating ports
port2ABM_query_Q = Port;          %port for ABM of 2nd data

%first close the port just in case
%(this is to try to prevent matlab from beuing unresponsive)
port2ABM_query_Q.close;

%open the ports
disp('_____opening ports...');
port2ABM_query_Q.open('/matlab/kinematicStructure/2nd_data_read');
disp('_____opened port /matlab/kinematicStructure/2nd_data_read');
pause(0.5);
disp('_____done.');

disp('Going to open port /matlab/kinematicStructure/2nd_data_read');
connection_check = 0;
while(~connection_check)
    Network.connect('/matlab/kinematicStructure/2nd_data_read', '/autobiographicalMemory/rpc');
    connection_check = Network.isConnected('/matlab/kinematicStructure/2nd_data_read','/autobiographicalMemory/rpc');
    pause(0.5)
    disp('waiting for connection...');
end
disp('2nd Port is connected to ABM!')

%%
%====================================
% Receive Data & Save Images
%====================================
% get number of frames: "getImagesInfo"
if strcmp(data_source_Q,'left') || strcmp(data_source_Q,'right')
    portIncoming_Q = BufferedPortImageRgb;
    %     portIncoming_Q = Port;
elseif strcmp(data_source_Q,'kinect')
    %     portIncoming_Q = Bottle;
    portIncoming_Q = Port;
end

portIncoming_Q.close;

disp('_____opening ports...');
portIncoming_Q.open('/matlab/kinematicStructure/2nd_datain');
disp('_____opened port /matlab/kinematicStructure/2nd_datain');
pause(0.5);
disp('_____done.');

disp('Going to open port /matlab/kinematicStructure/2nd_datain');

b_write_Q = yarp.Bottle;
b_response_Q = yarp.Bottle;

command_Q = ['triggerStreaming ', num2str(instance_num_Q), ' ("includeAugmented" 0) ("realtime" 1) ("speedMultiplier" 0.1)'];
b_write_Q.fromString(command_Q);
disp(b_write_Q);
port2ABM_query_Q.write(b_write_Q,b_response_Q);
% pause(3);
disp(b_response_Q.toString);

if strcmp(data_source_Q,'left') || strcmp(data_source_Q,'right')
    b_provide_Q = b_response_Q.get(2).asList();
elseif strcmp(data_source_Q,'kinect')
    b_provide_Q = b_response_Q.get(3).asList();
end

disp(b_provide_Q.toString);

% for Q
for i=0:b_provide_Q.size()-1
    str_buf = b_provide_Q.get(i).asList().get(0).asString();
    char_buf = char(str_buf);
    
    % for camcalib/left or camcalib/right
    if strcmp(data_source_Q,'left') || strcmp(data_source_Q,'right')
        if strcmp(char_buf(1:end-18), ['/autobiographicalMemory/icub/camcalib/',data_source_Q,'/out'])
            %         if strcmp(char_buf(1:end-18), '/autobiographicalMemory/grabber')
            num_images_Q = b_provide_Q.get(i).asList().get(1).asInt();
            final_portname_Q = char_buf;
        end
        % for kinect
    elseif strcmp(data_source_Q,'kinect')
        if strcmp(char_buf(1:end), '/autobiographicalMemory/agentDetector/skeleton:o')
            num_images_Q = b_provide_Q.get(i).asList().get(1).asInt();
            final_portname_Q = char_buf;
        end
    end
end

disp(final_portname_P);
disp(num2str(num_images_Q));

connection_check = 0;
while(~connection_check)
    Network.connect(final_portname_Q, '/matlab/kinematicStructure/2nd_datain');
    connection_check = Network.isConnected(final_portname_Q, '/matlab/kinematicStructure/2nd_datain');
    pause(0.5)
    disp('waiting for connection...');
end
disp('2nd Port Connected!')

%%
disp('================================');
disp('Start retrieving 2nd data!');

bDataMeta_Q_buf = cell(num_images_Q,1);
bRawImage_Q_buf = cell(num_images_Q,1);

start_frame_idx_Q = 1;
last_frame_idx_Q = num_images_Q;

reverseStr = '';
% h = waitbar(0,'Initializing progress bar...');

%------------------------------------------
% for camcalib/left or camcalib/right
if strcmp(data_source_Q,'left') || strcmp(data_source_Q,'right')
    for i=0:ceil(num_images_Q*0.9)-1        %         disp(['receive image ', num2str(i),'/',num2str(num_images_Q-2)]);
        %         waitbar(i/(num_images_Q-4),h,sprintf('%d%% along...',floor(i/(num_images_Q-4)*100)))
        percentDone = 100 * (i+1) / (ceil(num_images_Q*0.9));
        msg = sprintf('Percent done: %3.1f', percentDone); %Don't forget this semicolon
        fprintf([reverseStr, msg]);
        reverseStr = repmat(sprintf('\b'), 1, length(msg));
        
        yarpData_Q = yarp.ImageRgb;
        yarpData_Q = portIncoming_Q.read();
        
        if(i+1 >= start_frame_idx_Q && i < last_frame_idx_Q)
            % save images
            filename = [sprintf('%04d',i) '.png'];
            save_path = ['ABM/Q/',filename];
            
            if (sum(size(yarpData_Q)) ~= 0) %check size of bottle
                %disp('got it..');
                h=yarpData_Q.height;
                w=yarpData_Q.width;
                pixSize=yarpData_Q.getPixelSize();
                tool=YarpImageHelper(h, w);
                
                IN = tool.getRawImg(yarpData_Q); %use leo pape image patch
                TEST = reshape(IN, [h w pixSize]); %need to reshape the matrix from 1D to h w pixelSize
                matlabImage=uint8(zeros(h, w, pixSize)); %create an empty image with the correct dimentions
                r = cast(TEST(:,:,1),'uint8');  % need to cast the image from int16 to uint8
                g = cast(TEST(:,:,2),'uint8');
                b = cast(TEST(:,:,3),'uint8');
                matlabImage(:,:,1)= r; % copy the image to the previoulsy create matrix
                matlabImage(:,:,2)= g;
                matlabImage(:,:,3)= b;
            else
                disp('incorrect image');
            end
            
            imwrite(matlabImage,save_path);
            env = yarp.Bottle;
            portIncoming_Q.getEnvelope(env);
            bDataMeta_Q_buf{i+1,1} = env.toString();
        end
    end
    disp('');
    disp('Done receiving images!');
    %     close(h);
    
    %------------------------------------------
    % for kinect
elseif strcmp(data_source_Q,'kinect')
    fileID = fopen('ABM/Q/joints.txt','w');
    yarpData_Q = yarp.Bottle;
    portIncoming_Q.read(yarpData_Q);
    
    for i=0:floor(num_images_Q/16)-2
        disp(['receive data of frame ', num2str(i)]);
        if(i+1 >= start_frame_idx_Q && i < last_frame_idx_Q)
            % save data
            if (sum(size(yarpData_Q)) ~= 0) %check size of bottle
                for body_idx = 0:15
                    buf = char(yarpData_Q.get(0).asList().get(body_idx).toString());
                    readData = str2double(strsplit(' ',buf));
                    x = readData(2);
                    y = readData(3);
                    z = readData(4);
                    fprintf(fileID, '%d\t %d\t %6.6f\t %6.6f\t %6.6f\n', i+1, body_idx, x, y, z);
                end
            else
                disp('incorrect data');
            end
            
            env = yarp.Bottle;
            portIncoming_Q.getEnvelope(env);
            bDataMeta_Q_buf{i+1,1} = env.toString();
        end
    end
    fclose(fileID);
end

% save buf data
save('ABM/Q/DataMeta.mat','bDataMeta_Q_buf');

%%
%====================================
% Generate Video
%====================================
%------------------------------------------
% for camcalib/left or camcalib/right
if strcmp(data_source_Q,'left') || strcmp(data_source_Q,'right')
    imagesFolder_Q='ABM/Q';
    imageFiles_Q = dir(strcat(imagesFolder_Q,'/*.png'));
    num_images_Q = size(imageFiles_Q,1);
    
    fileIdx_Q = zeros(num_images_Q,1);
    VideoFile_Q = strcat(imagesFolder_Q,'/video');
    writerObj_Q = VideoWriter(VideoFile_Q);
    
    fps= 10;
    writerObj_Q.FrameRate = fps;
    open(writerObj_Q);
    for t = 1:length(imageFiles_Q)
        Frame = imread(strcat(imagesFolder_Q,'/',imageFiles_Q(t).name));
        writeVideo(writerObj_Q,im2frame(Frame));
    end
    close(writerObj_Q);
    %------------------------------------------
    % for kinect
elseif strcmp(data_source_Q,'kinect')
end

%%
%====================================
% Close Ports
%====================================
port2ABM_query_Q.close;
portIncoming_Q.close;