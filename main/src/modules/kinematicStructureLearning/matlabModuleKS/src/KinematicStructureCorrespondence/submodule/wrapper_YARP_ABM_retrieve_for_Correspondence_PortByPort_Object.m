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
[SUCCESS,MESSAGE,MESSAGEID] = rmdir('data/KSC/P','s');
[SUCCESS,MESSAGE,MESSAGEID] = rmdir('data/KSC/Q','s');
mkdir('data/KSC/P');
mkdir('data/KSC/Q');

%%
%-------------------------------------------------------------------------------------------
% Port P
%-------------------------------------------------------------------------------------------
%==================================================================================================
% Connecting Ports
%==================================================================================================
%creating ports
port2ABM_query_images_P = Port;          %port for ABM of 1st data

%first close the port just in case
%(this is to try to prevent matlab from beuing unresponsive)
port2ABM_query_images_P.close;

disp(' ')
disp('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++');
disp('+     Image or Kinect data loading for P (1 out of 4)    +');
disp('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++');

%open the ports
port2ABM_query_images_P.open('/matlab/kinematicStructure/data_read_images_P');
disp('_____opened port /matlab/kinematicStructure/data_read_images_P');
pause(0.5);

connection_check = 0;
while(~connection_check)
    Network.connect('/matlab/kinematicStructure/data_read_images_P', '/autobiographicalMemory/rpc');
    connection_check = Network.isConnected('/matlab/kinematicStructure/data_read_images_P','/autobiographicalMemory/rpc');
    pause(0.5)
    disp('waiting for connection...');
end
disp('P Port is connected to ABM!')

%==================================================================================================
% Receive Data & Save Images
%==================================================================================================
if strcmp(data_source_P,'left') || strcmp(data_source_P,'right')
    portIncoming_images_P = BufferedPortImageRgb;
elseif strcmp(data_source_P,'kinect')
    portIncoming_images_P = Port;
end

portIncoming_images_P.close;

portIncoming_images_P.open('/matlab/kinematicStructure/data_in_images_P');
disp('_____opened port /matlab/kinematicStructure/data_in_images_P');
pause(0.5);

b_write_images_P = yarp.Bottle;
b_response_images_P = yarp.Bottle;
b_provide_images_P = yarp.Bottle;

pause(2)

command_P = ['triggerStreaming ', num2str(instance_num_P), ' ("includeAugmented" 0) ("realtime" 1) ("speedMultiplier" 0.1)'];
b_write_images_P.fromString(command_P);
disp(b_write_images_P);
port2ABM_query_images_P.write(b_write_images_P,b_response_images_P);
disp(b_response_images_P.toString);

if strcmp(data_source_P,'left') || strcmp(data_source_P,'right')
    b_provide_images_P = b_response_images_P.get(2).asList();
elseif strcmp(data_source_P,'kinect')
    b_provide_images_P = b_response_images_P.get(3).asList();
end

disp(b_provide_images_P.toString);

% for P
for i=0:b_provide_images_P.size()-1
    str_buf = b_provide_images_P.get(i).asList().get(0).asString();
    char_buf = char(str_buf);
    
    % for camcalib/left or camcalib/right
    if strcmp(data_source_P,'left') || strcmp(data_source_P,'right')
        
        if strcmp(char_buf(1:end-18), ['/autobiographicalMemory/icub/camcalib/',data_source_P,'/out'])
            num_images_P = b_provide_images_P.get(i).asList().get(1).asInt();
            final_portname_images_P = char_buf;
        end
        % for kinect
    elseif strcmp(data_source_P,'kinect')
        if strcmp(char_buf(1:end), '/autobiographicalMemory/agentDetector/kinect/agentLoc:o')
            num_images_P = b_provide_images_P.get(i).asList().get(1).asInt();
            final_portname_images_P = char_buf;
        end
    end
end

disp(final_portname_images_P);
disp(['Total number of data: ', num2str(num_images_P)]);

connection_check = 0;
while(~connection_check)
    Network.connect(final_portname_images_P, '/matlab/kinematicStructure/data_in_images_P');
    connection_check = Network.isConnected(final_portname_images_P, '/matlab/kinematicStructure/data_in_images_P');
    %     pause(0.5)
    disp('waiting for connection...');
end
disp('P Port Connected!')

%==================================================================================================
bDataMeta_images_P_buf = cell(ceil(num_images_P*0.9),1);
bRawImage_images_P_buf = cell(ceil(num_images_P*0.9),1);

start_frame_idx_P = 1;
last_frame_idx_P = num_images_P;

disp(['Actual number of retrieving images: ',num2str(ceil(num_images_P*0.9))]);
reverseStr = '';
waitbar_h = waitbar(0,'Initializing progress bar...');

%------------------------------------------
% for camcalib/left or camcalib/right
if strcmp(data_source_P,'left') || strcmp(data_source_P,'right')
    for i=0:ceil(num_images_P*0.9)-1
        percentDone = 100 * (i+1) / (ceil(num_images_P*0.9));
        msg = sprintf('Percent done: %3.1f', percentDone); %Don't forget this semicolon
        fprintf([reverseStr, msg]);
        reverseStr = repmat(sprintf('\b'), 1, length(msg));
        
        waitbar(percentDone/100,waitbar_h,sprintf('%3.1f%% along ...', percentDone));
        
        yarpData_images_P = yarp.ImageRgb;
        yarpData_images_P = portIncoming_images_P.read();
        
        if(i+1 >= start_frame_idx_P && i < last_frame_idx_P)
            % save images
            filename = [sprintf('%04d',i) '.png'];
            save_path = ['data/KSC/P/',filename];
            
            if (sum(size(yarpData_images_P)) ~= 0) %check size of bottle
                h=yarpData_images_P.height;
                w=yarpData_images_P.width;
                pixSize=yarpData_images_P.getPixelSize();
                tool=YarpImageHelper(h, w);
                IN = tool.getRawImg(yarpData_images_P); %use leo pape image patch
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
            portIncoming_images_P.getEnvelope(env);
            bDataMeta_images_P_buf{i+1,1} = env.toString();
            pause(0.01);
        end
    end
    disp('');
    disp('Done receiving images for P !');
    close(waitbar_h);
    %------------------------------------------
    % for kinect
elseif strcmp(data_source_P,'kinect')
    fileID = fopen('data/KSC/P/joints.txt','w');
    yarpData_images_P = yarp.Bottle;
    
    for i=0:floor(num_images_P*0.9/16)
        
        portIncoming_images_P.read(yarpData_images_P);
        
        percentDone = 100 * (i+1) / (floor(num_images_P*0.9/16));
        msg = sprintf('Percent done: %3.1f', percentDone); %Don't forget this semicolon
        fprintf([reverseStr, msg]);
        reverseStr = repmat(sprintf('\b'), 1, length(msg));
        
        waitbar(percentDone/100,waitbar_h,sprintf('%3.1f%% along ...', percentDone));
        
        if(i+1 >= start_frame_idx_P && i < last_frame_idx_P)
            % save data
            if (sum(size(yarpData_images_P)) ~= 0) %check size of bottle
                if yarpData_images_P.get(0).asList().size() == 16
                    for body_idx = 0:15
                        buf = char(yarpData_images_P.get(0).asList().get(body_idx).asList().get(1).toString());
                        body_name = char(yarpData_images_P.get(0).asList().get(body_idx).asList().get(0).toString());
                        expression = '\s*';
                        matchStr = regexp(buf,expression,'split');
                        readData = str2double(matchStr);
                        x = readData(1);
                        y = readData(2);
                        z = readData(3);
                        %                         fprintf(fileID, '%d\t %s\t %6.6f\t %6.6f\t %6.6f\n', i+1, body_name, x, y, z);
                        fprintf(fileID, '%d\t %d\t %6.6f\t %6.6f\t %6.6f\n', i+1, body_idx, x, y, z);
                    end
                end
            else
                disp('incorrect data');
            end
            
            env = yarp.Bottle;
            portIncoming_images_P.getEnvelope(env);
            bDataMeta_images_P_buf{i+1,1} = env.toString();
        end
        yarpData_images_P.clear();
    end
    fclose(fileID);
    disp('');
    disp('Done receiving kinect data for P !');
    close(waitbar_h);
end

% save buf data
save('data/KSC/P/DataMeta.mat','bDataMeta_images_P_buf');

%==================================================================================================
% Generate Video
%==================================================================================================
%------------------------------------------
% for camcalib/left or camcalib/right
if strcmp(data_source_P,'left') || strcmp(data_source_P,'right')
    imagesFolder_P='data/KSC/P';
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

%==================================================================================================
% Close Ports
%==================================================================================================
port2ABM_query_images_P.close;
portIncoming_images_P.close;

disp('Done receiving images / kinect data for P');
disp('PAUSED for 5 seconds...')
pause(5);

%%
% %==================================================================================================
% % Joint Projection Loading for P only if it is from left / right
% %==================================================================================================
% if strcmp(data_source_P,'left') || strcmp(data_source_P,'right')
%     %====================================
%     % Connecting Ports
%     %====================================
%     %creating ports
%     port2ABM_query_joints_P = Port;          %port for ABM of 1st data
%     
%     %first close the port just in case
%     %(this is to try to prevent matlab from beuing unresponsive)
%     port2ABM_query_joints_P.close;
%     
%     disp(' ')
%     disp('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++');
%     disp('+     Joint projection data loading for P (1.5 out of 4)    +');
%     disp('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++');
%     
%     %open the ports
%     port2ABM_query_joints_P.open('/matlab/kinematicStructure/data_read_joints_P');
%     disp('_____opened port /matlab/kinematicStructure/data_read_joints_P');
%     pause(0.5);
%     
%     connection_check = 0;
%     while(~connection_check)
%         Network.connect('/matlab/kinematicStructure/data_read_joints_P', '/autobiographicalMemory/rpc');
%         connection_check = Network.isConnected('/matlab/kinematicStructure/data_read_joints_P','/autobiographicalMemory/rpc');
%         pause(0.5)
%         disp('waiting for connection...');
%     end
%     disp('P Port is connected to ABM!')
%     
%     %%
%     %==================================================================================================
%     % Receive Data & Save Images
%     %==================================================================================================
%     % get number of frames: "getImagesInfo"
%     if strcmp(data_source_P,'left') || strcmp(data_source_P,'right')
%         portIncoming_joints_P = BufferedPortImageRgb;
%     elseif strcmp(data_source_P,'kinect')
%         portIncoming_joints_P = Port;
%     end
%     disp(1)
%     portIncoming_joints_P.close;
%     
%     portIncoming_joints_P.open('/matlab/kinematicStructure/data_in_joints_P');
%     disp('_____opened port /matlab/kinematicStructure/data_in_joints_P');
%     pause(0.5);
%     disp(2)
%     b_write_joints_P = yarp.Bottle;
%     b_response_joints_P = yarp.Bottle;
%     disp(3)
% %-----------------------------------    
%     command_P = ['triggerStreaming ', num2str(instance_num_P), ' ("includeAugmented" 0) ("realtime" 1) ("speedMultiplier" 0.1)'];
%     b_write_joints_P.fromString(command_P);
%     port2ABM_query_joints_P.write(b_write_joints_P,b_response_joints_P);
%     disp(4)
% %     b_provide_P = b_response_P.get(3).asList().get(1).asList();
%     
%     for i=0:b_response_joints_P.get(3).asList().size()-1
%         str_buf = b_response_joints_P.get(3).asList().get(i).asList().get(0).asString();
%         char_buf = char(str_buf);
% disp(5)
% %         if strcmp(char_buf(1:end), ['/autobiographicalMemory/jointsAwareness/',data_source_P,'_arm/to',data_source_P,'Cam/joints2DProj:o'])
%         if strcmp(char_buf(1:end), '/autobiographicalMemory/jointsAwareness/left_arm/toleftCam/joints2DProj:o')
%             num_data = b_response_joints_P.get(3).asList().get(i).asList().get(1).asInt();
%             final_portname_joints_P = char_buf;
%         end
%     end
%     
%     disp(final_portname_joints_P);
%     
%     connection_check = 0;
%     while(~connection_check)
%         Network.connect(final_portname_joints_P, '/matlab/kinematicStructure/data_in_joints_P');
%         connection_check = Network.isConnected(final_portname_joints_P, '/matlab/kinematicStructure/data_in_joints_P');
%         disp('waiting for connection...');
%     end
%     disp('P Port for Joint Projection data receiving is connected!')
% %----------------    
% % b_write = yarp.Bottle;
% % b_response = yarp.Bottle;
% % command = ['triggerStreaming ', num2str(instance_num_P), ' ("includeAugmented" 0) ("realtime" 1) ("speedMultiplier" 0.1)'];
% % b_write.fromString(command);
% % disp(b_write);
% % port2ABM_read.write(b_write,b_response);
% % 
% % disp(b_response.toString);
% % 
% % b_provide = b_response.get(3).asList().get(1).asList();
% % disp(b_provide.toString);
% % 
% % % for i=0:b_provide.size()-1
% % for i=0:b_response.get(3).asList().size()-1
% % %     str_buf = b_provide.get(i).asList().get(0).asString();
% %     str_buf = b_response.get(3).asList().get(i).asList().get(0).asString();
% %     char_buf = char(str_buf);
% %     
% %     disp(char_buf)
% %     
% % %     if strcmp(char_buf(1:end-18), ['/autobiographicalMemory/icub/camcalib/',camera_selection,'/out'])
% %     if strcmp(char_buf(1:end), '/autobiographicalMemory/jointsAwareness/left_arm/toleftCam/joints2DProj:o')
% %         num_data = b_response.get(3).asList().get(i).asList().get(1).asInt();
% %         final_portname_data = char_buf;
% %     end
% % end
% % disp(['OUT: ',final_portname_data]);
% % Network.connect(final_portname_data, '/matlab/kinematicStructure/data_in_P');    
%     
%     %%
%     start_frame_idx_P = 1;
%     last_frame_idx_P = num_images_P;
%     
%     disp(['Actual number of retrieving joint projection data: ',num2str(ceil(num_images_P*0.9))]);
%     reverseStr = '';
%     waitbar_h = waitbar(0,'Initializing progress bar...');
%     
%     fileID = fopen('data/KSC/P/jointsAwareness.txt','w');
%     yarpData_joint_P = yarp.Bottle;
%     
%     %------------------------------------------
%     % for yarp read
%     for i=0:ceil(num_images_P*0.9)-1
% 
%         percentDone = 100 * (i+1) / (ceil(num_images_P*0.9));
%         msg = sprintf('Percent done: %3.1f', percentDone); %Don't forget this semicolon
%         fprintf([reverseStr, msg]);
%         reverseStr = repmat(sprintf('\b'), 1, length(msg));
% 
%         waitbar(percentDone/100,waitbar_h,sprintf('%3.1f%% along ...', percentDone));
% 
%         if(i+1 >= start_frame_idx_P && i < last_frame_idx_P)
% 
%             portIncoming_joints_P.read(yarpData_joint_P);
%             
%             % save data
%             if (sum(size(yarpData_joint_P)) ~= 0) %check size of bottle
%                 buf = char(yarpData_joint_P.get(0).asList().toString());
%                 readData = str2num(buf);
%                 
%                 for body_idx = 0:6                
%                     x = readData((body_idx)*2+1);
%                     y = readData((body_idx)*2+2);
%                     fprintf(fileID, '%d\t %d\t %6.6f\t %6.6f\n', i+1, body_idx, x, y);
%                 end                
%             else
%                 disp('incorrect image');
%             end
%         end
%     end
%     fclose(fileID);
%     
%     disp('');
%     disp('Done receiving joint projection data for P!');
%     close(waitbar_h);
%     
%     %%
%     %====================================
%     % Close Ports
%     %====================================
%     port2ABM_query_joints_P.close;
%     portIncoming_joints_P.close;
%     
%     disp('PAUSED for 3 seconds...')
%     pause(3);
% end
% 
% %%
% %==================================================================================================
% % OBJECT location information for P
% %==================================================================================================
% %====================================
% % Connecting Ports
% %====================================
% %creating ports
% port2ABM_query_images_P = Port;          %port for ABM of 1st data
% 
% %first close the port just in case
% %(this is to try to prevent matlab from beuing unresponsive)
% port2ABM_query_images_P.close;
% 
% disp(' ')
% disp('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++');
% disp('+     Object location data loading for P (2 out of 4)    +');
% disp('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++');
% 
% %open the ports
% port2ABM_query_images_P.open('/matlab/kinematicStructure/data_read_P');
% disp('_____opened port /matlab/kinematicStructure/data_read_P');
% pause(0.5);
% 
% connection_check = 0;
% while(~connection_check)
%     Network.connect('/matlab/kinematicStructure/data_read_P', '/autobiographicalMemory/rpc');
%     connection_check = Network.isConnected('/matlab/kinematicStructure/data_read_P','/autobiographicalMemory/rpc');
%     pause(0.5)
%     disp('waiting for connection...');
% end
% disp('P Port is connected to ABM!')
% 
% %%
% %==================================================================================================
% % Receive Data & Save Images
% %==================================================================================================
% % get number of frames: "getImagesInfo"
% if strcmp(data_source_P,'left') || strcmp(data_source_P,'right')
%     portIncoming_images_P = BufferedPortImageRgb;
% elseif strcmp(data_source_P,'kinect')
%     portIncoming_images_P = Port;
% end
% 
% portIncoming_images_P.close;
% 
% portIncoming_images_P.open('/matlab/kinematicStructure/data_in_P');
% disp('_____opened port /matlab/kinematicStructure/data_in_P');
% pause(0.5);
% 
% b_write_images_P = yarp.Bottle;
% b_response_images_P = yarp.Bottle;
% 
% pause(3)
% 
% command_P = ['triggerStreaming ', num2str(instance_num_P), ' ("includeAugmented" 0) ("realtime" 1) ("speedMultiplier" 0.1)'];
% b_write_images_P.fromString(command_P);
% port2ABM_query_images_P.write(b_write_images_P,b_response_images_P);
% 
% b_provide_images_P = yarp.Bottle;
% 
% if strcmp(data_source_P,'left') || strcmp(data_source_P,'right')
%     b_provide_images_P = b_response_images_P.get(3).asList();
% elseif strcmp(data_source_P,'kinect')
%     b_provide_images_P = b_response_images_P.get(3).asList();
% end
% 
% % for Object of P
% for i=0:b_provide_images_P.size()-1
%     str_buf = b_provide_images_P.get(i).asList().get(0).asString();
%     char_buf = char(str_buf);
%     
%     % for camcalib/left or camcalib/right
%     if strcmp(data_source_P,'left') || strcmp(data_source_P,'right')        
%         if strcmp(char_buf(1:end), ['/autobiographicalMemory/jointsAwareness/objects/to',data_source_P,'Cam/objects2DProj:o'])
%             num_images_P = b_provide_images_P.get(i).asList().get(1).asInt();
%             final_portname_P_object = char_buf;
%         end
%         % for kinect
%     elseif strcmp(data_source_P,'kinect')
%         if strcmp(char_buf(1:end), '/autobiographicalMemory/iol2opc/objLoc:o')
%             num_images_P = b_provide_images_P.get(i).asList().get(1).asInt();
%             final_portname_P_object = char_buf;
%         end
%     end
% end
% 
% disp(final_portname_P_object);
% disp(['Total number of object data: ', num2str(num_images_P)]);
% 
% connection_check = 0;
% while(~connection_check)
%     Network.connect(final_portname_P_object, '/matlab/kinematicStructure/data_in_P');
%     connection_check = Network.isConnected(final_portname_P_object, '/matlab/kinematicStructure/data_in_P');
%     disp('waiting for connection...');
% end
% disp('P Port Connected!')
% 
% %%
% bDataMeta_images_P_buf = cell(ceil(num_images_P*0.9),1);
% bRawImage_images_P_buf = cell(ceil(num_images_P*0.9),1);
% 
% start_frame_idx_P = 1;
% last_frame_idx_P = num_images_P;
% 
% disp(['Actual number of retrieving images: ',num2str(ceil(num_images_P*0.9))]);
% reverseStr = '';
% waitbar_h = waitbar(0,'Initializing progress bar...');
% 
% %------------------------------------------
% % for camcalib/left or camcalib/right
% if strcmp(data_source_P,'left') || strcmp(data_source_P,'right')
%     fileID = fopen('data/KSC/P/object_locations.txt','w');
%     yarpData_images_P = yarp.Bottle;
%     
%     for i=0:floor(num_images_P*0.9)        
%         portIncoming_images_P.read(yarpData_images_P);
%         
%         percentDone = 100 * (i+1) / (floor(num_images_P*0.9));
%         msg = sprintf('Percent done: %3.1f', percentDone); 
%         fprintf([reverseStr, msg]);
%         reverseStr = repmat(sprintf('\b'), 1, length(msg));
%         
%         waitbar(percentDone/100, waitbar_h, sprintf('%3.1f%% along ...', percentDone));
%         
%         if(i+1 >= start_frame_idx_P && i < last_frame_idx_P)
%             % save data
%             if (sum(size(yarpData_images_P)) ~= 0) %check size of bottle
%                 buf = char(yarpData_images_P.get(0).asList().get(0).asList().get(1).toString());
%                 expression = '\s*';
%                 matchStr = regexp(buf,expression,'split');
%                 readData = str2double(matchStr);
%                 x = readData(1);
%                 y = readData(2);
%                 fprintf(fileID, '%d\t %6.6f\t %6.6f\n', i, x, y);
%             else
%                 disp('incorrect data');
%             end
%             
%             env = yarp.Bottle;
%             portIncoming_images_P.getEnvelope(env);
%             bDataMeta_images_P_buf{i+1,1} = env.toString();
%         end
%         yarpData_images_P.clear();
%     end
%     fclose(fileID);
%     disp('');
%     disp('Done receiving object location data!');
%     close(waitbar_h);
%     %------------------------------------------
%     % for kinect
% elseif strcmp(data_source_P,'kinect')
%     fileID = fopen('data/KSC/P/object_locations.txt','w');
%     yarpData_images_P = yarp.Bottle;
%     
%     for i=0:floor(num_images_P*0.9)        
%         portIncoming_images_P.read(yarpData_images_P);
%         
%         percentDone = 100 * (i+1) / (floor(num_images_P*0.9));
%         msg = sprintf('Percent done: %3.1f', percentDone); 
%         fprintf([reverseStr, msg]);
%         reverseStr = repmat(sprintf('\b'), 1, length(msg));
%         
%         waitbar(percentDone/100, waitbar_h, sprintf('%3.1f%% along ...', percentDone));
%         
%         if(i+1 >= start_frame_idx_P && i < last_frame_idx_P)
%             % save data
%             if (sum(size(yarpData_images_P)) ~= 0) %check size of bottle
%                 buf = char(yarpData_images_P.get(0).asList().get(0).asList().get(1).toString());
%                 expression = '\s*';
%                 matchStr = regexp(buf,expression,'split');
%                 readData = str2double(matchStr);
%                 x = readData(1);
%                 y = readData(2);
%                 z = readData(3);
%                 fprintf(fileID, '%d\t %6.6f\t %6.6f\t %6.6f\n', i, x, y, z);
%             else
%                 disp('incorrect data');
%             end
%             
%             env = yarp.Bottle;
%             portIncoming_images_P.getEnvelope(env);
%             bDataMeta_images_P_buf{i+1,1} = env.toString();
%         end
%         yarpData_images_P.clear();
%     end
%     fclose(fileID);
%     disp('');
%     disp('Done receiving object location data!');
%     close(waitbar_h);
% end
% 
% %%
% %====================================
% % Close Ports
% %====================================
% port2ABM_query_images_P.close;
% portIncoming_images_P.close;
% 
% disp('PAUSED for 3 seconds...')
% pause(3);

%%
%-------------------------------------------------------------------------------------------
% Port Q
%-------------------------------------------------------------------------------------------
%==================================================================================================
% Connecting Ports
%==================================================================================================
%creating ports
port2ABM_query_Q = Port;          %port for ABM of 2nd data

%first close the port just in case
%(this is to try to prevent matlab from beuing unresponsive)
port2ABM_query_Q.close;

disp(' ')
disp('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++');
disp('+     Image or Kinect data loading from Q (3 out of 4)    +');
disp('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++');

%open the ports
port2ABM_query_Q.open('/matlab/kinematicStructure/data_read_Q');
disp('_____opened port /matlab/kinematicStructure/data_read_Q');
pause(0.5);

connection_check = 0;
while(~connection_check)
    Network.connect('/matlab/kinematicStructure/data_read_Q', '/autobiographicalMemory/rpc');
    connection_check = Network.isConnected('/matlab/kinematicStructure/data_read_Q','/autobiographicalMemory/rpc');
    pause(0.5)
    disp('waiting for connection...');
end
disp('Q Port is connected to ABM!')

%==================================================================================================
% Receive Data & Save Images
%==================================================================================================
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
portIncoming_Q.open('/matlab/kinematicStructure/data_in_Q');
disp('_____opened port /matlab/kinematicStructure/data_in_Q');
pause(0.5);

b_write_Q = yarp.Bottle;
b_response_Q = yarp.Bottle;

disp(' ')
disp('+++++++++++++++++++++++++++++++++++++++++++++++++++++');
disp('+  Image or Kinect data loading from Q (3 out of 4) +');
disp('+++++++++++++++++++++++++++++++++++++++++++++++++++++');

command_Q = ['triggerStreaming ', num2str(instance_num_Q), ' ("includeAugmented" 0) ("realtime" 1) ("speedMultiplier" 0.1)'];
b_write_Q.fromString(command_Q);
disp(b_write_Q);
port2ABM_query_Q.write(b_write_Q,b_response_Q);
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
            num_images_Q = b_provide_Q.get(i).asList().get(1).asInt();
            final_portname_Q = char_buf;
        end
        % for kinect
    elseif strcmp(data_source_Q,'kinect')
        if strcmp(char_buf(1:end), '/autobiographicalMemory/agentDetector/kinect/agentLoc:o')
            num_images_Q = b_provide_Q.get(i).asList().get(1).asInt();
            final_portname_Q = char_buf;
        end
    end
end

disp(final_portname_Q);
disp(['Total number of data: ', num2str(num_images_Q)]);

connection_check = 0;
while(~connection_check)
    Network.connect(final_portname_Q, '/matlab/kinematicStructure/data_in_Q');
    connection_check = Network.isConnected(final_portname_Q, '/matlab/kinematicStructure/data_in_Q');
    pause(0.5)
    disp('waiting for connection...');
end
disp('2nd Port Connected!')

%==================================================================================================
bDataMeta_Q_buf = cell(ceil(num_images_Q*0.9),1);
bRawImage_Q_buf = cell(ceil(num_images_Q*0.9),1);

start_frame_idx_Q = 1;
last_frame_idx_Q = num_images_Q;

disp(['Actual number of retrieving images: ',num2str(ceil(num_images_Q*0.9))]);
reverseStr = '';
waitbar_h = waitbar(0,'Initializing progress bar...');

%------------------------------------------
% for camcalib/left or camcalib/right
if strcmp(data_source_Q,'left') || strcmp(data_source_Q,'right')
    for i=0:ceil(num_images_Q*0.9)-1
        yarpData_Q = yarp.ImageRgb;
        yarpData_Q = portIncoming_Q.read();
        
        percentDone = 100 * (i+1) / (ceil(num_images_Q*0.9));
        msg = sprintf('Percent done: %3.1f', percentDone); %Don't forget this semicolon
        fprintf([reverseStr, msg]);
        reverseStr = repmat(sprintf('\b'), 1, length(msg));
        waitbar(percentDone/100,waitbar_h,sprintf('%3.1f%% along ...', percentDone));
        
        if(i+1 >= start_frame_idx_Q && i < last_frame_idx_Q)
            % save images
            filename = [sprintf('%04d',i) '.png'];
            save_path = ['data/KSC/Q/',filename];
            
            if (sum(size(yarpData_Q)) ~= 0) %check size of bottle
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
    disp('Done receiving images for Q !');
    close(waitbar_h);
    
    %------------------------------------------
    % for kinect
elseif strcmp(data_source_Q,'kinect')
    fileID = fopen('data/KSC/Q/joints.txt','w');
    yarpData_Q = yarp.Bottle;
    
    for i=0:floor(num_images_Q*0.9/16)
        
        portIncoming_Q.read(yarpData_Q);
        
        percentDone = 100 * (i) / (floor(num_images_Q*0.9/16));
        msg = sprintf('Percent done: %3.1f', percentDone); %Don't forget this semicolon
        fprintf([reverseStr, msg]);
        reverseStr = repmat(sprintf('\b'), 1, length(msg));
        
        waitbar(percentDone/100,waitbar_h,sprintf('%3.1f%% along ...', percentDone));
        
        if(i+1 >= start_frame_idx_Q && i < last_frame_idx_Q)
            % save data
            if (sum(size(yarpData_Q)) ~= 0) %check size of bottle
                
                if yarpData_Q.get(0).asList().size() == 16
                    for body_idx = 0:15
                        
                        buf = char(yarpData_Q.get(0).asList().get(body_idx).asList().get(1).toString());
                        body_name = char(yarpData_Q.get(0).asList().get(body_idx).asList().get(0).toString());
                        expression = '\s*';
                        matchStr = regexp(buf,expression,'split');
                        readData = str2double(matchStr);
                        
                        x = readData(1);
                        y = readData(2);
                        z = readData(3);
                        
                        %                         fprintf(fileID, '%d\t %s\t %6.6f\t %6.6f\t %6.6f\n', i+1, body_name, x, y, z);
                        fprintf(fileID, '%d\t %d\t %6.6f\t %6.6f\t %6.6f\n', i+1, body_idx, x, y, z);
                    end
                end
            else
                disp('incorrect data');
            end
            
            env = yarp.Bottle;
            portIncoming_Q.getEnvelope(env);
            bDataMeta_Q_buf{i+1,1} = env.toString();
        end
        yarpData_Q.clear();
    end
    
    fclose(fileID);
    disp('');
    disp('Done receiving kinect data for Q !');
    close(waitbar_h);
end

% save buf data
save('data/KSC/Q/DataMeta.mat','bDataMeta_Q_buf');

%==================================================================================================
% Generate Video
%==================================================================================================
%------------------------------------------
% for camcalib/left or camcalib/right
if strcmp(data_source_Q,'left') || strcmp(data_source_Q,'right')
    imagesFolder_Q='data/KSC/Q';
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

%==================================================================================================
% Close Ports
%==================================================================================================
port2ABM_query_Q.close;
portIncoming_Q.close;

disp('PAUSED for 3 seconds...')
pause(3);

% %%
% %==================================================================================================
% % OBJECT location information
% %==================================================================================================
% %====================================
% % Connecting Ports
% %====================================
% %creating ports
% port2ABM_query_Q = Port;          %port for ABM of 1st data
% 
% %first close the port just in case
% %(this is to try to prevent matlab from beuing unresponsive)
% port2ABM_query_Q.close;
% 
% disp(' ')
% disp('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++');
% disp('+     Object location data loading from Q (4 out of 4)    +');
% disp('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++');
% 
% %open the ports
% port2ABM_query_Q.open('/matlab/kinematicStructure/data_read_Q');
% disp('_____opened port /matlab/kinematicStructure/data_read_Q');
% pause(0.5);
% 
% connection_check = 0;
% while(~connection_check)
%     Network.connect('/matlab/kinematicStructure/data_read_Q', '/autobiographicalMemory/rpc');
%     connection_check = Network.isConnected('/matlab/kinematicStructure/data_read_Q','/autobiographicalMemory/rpc');
%     pause(0.5)
%     disp('waiting for connection...');
% end
% disp('Q Port is connected to ABM!')
% 
% %%
% %==================================================================================================
% % Receive Data & Save Images
% %==================================================================================================
% % get number of frames: "getImagesInfo"
% if strcmp(data_source_Q,'left') || strcmp(data_source_Q,'right')
%     portIncoming_Q = BufferedPortImageRgb;
% elseif strcmp(data_source_Q,'kinect')
%     portIncoming_Q = Port;
% end
% 
% portIncoming_Q.close;
% 
% portIncoming_Q.open('/matlab/kinematicStructure/data_in_Q');
% disp('_____opened port /matlab/kinematicStructure/data_in_Q');
% pause(0.5);
% 
% b_write_Q = yarp.Bottle;
% b_response_Q = yarp.Bottle;
% 
% pause(3)
% 
% command_Q = ['triggerStreaming ', num2str(instance_num_Q), ' ("includeAugmented" 0) ("realtime" 1) ("speedMultiplier" 0.1)'];
% b_write_Q.fromString(command_Q);
% port2ABM_query_Q.write(b_write_Q,b_response_Q);
% 
% b_provide_Q = yarp.Bottle;
% 
% if strcmp(data_source_Q,'left') || strcmp(data_source_Q,'right')
%     b_provide_Q = b_response_Q.get(2).asList();
% elseif strcmp(data_source_Q,'kinect')
%     b_provide_Q = b_response_Q.get(3).asList();
% end
% 
% % for Object of Q
% for i=0:b_provide_Q.size()-1
%     str_buf = b_provide_Q.get(i).asList().get(0).asString();
%     char_buf = char(str_buf);
%     
%     % for camcalib/left or camcalib/right
%     if strcmp(data_source_Q,'left') || strcmp(data_source_Q,'right')
%         
%         if strcmp(char_buf(1:end-18), ['/autobiographicalMemory/icub/camcalib/',data_source_Q,'/out'])
%             %         if strcmp(char_buf(1:end-18), '/autobiographicalMemory/grabber')
%             num_images_Q = b_provide_Q.get(i).asList().get(1).asInt();
%             final_portname_Q = char_buf;
%         end
%         % for kinect
%     elseif strcmp(data_source_Q,'kinect')
%         if strcmp(char_buf(1:end), '/autobiographicalMemory/iol2opc/objLoc:o')
%             num_images_Q = b_provide_Q.get(i).asList().get(1).asInt();
%             final_portname_Q_object = char_buf;
%         end
%     end
% end
% 
% disp(final_portname_Q_object);
% disp(['Total number of data: ', num2str(num_images_Q)]);
% 
% connection_check = 0;
% while(~connection_check)
%     Network.connect(final_portname_Q_object, '/matlab/kinematicStructure/data_in_Q');
%     connection_check = Network.isConnected(final_portname_Q_object, '/matlab/kinematicStructure/data_in_Q');
%     disp('waiting for connection...');
% end
% disp('Q Port Connected!')
% 
% %%
% bDataMeta_Q_buf = cell(ceil(num_images_Q*0.9),1);
% bRawImage_Q_buf = cell(ceil(num_images_Q*0.9),1);
% 
% start_frame_idx_Q = 1;
% last_frame_idx_Q = num_images_Q;
% 
% disp(['Actual number of retrieving images: ',num2str(ceil(num_images_Q*0.9))]);
% reverseStr = '';
% waitbar_h = waitbar(0,'Initializing progress bar...');
% 
% %------------------------------------------
% % for camcalib/left or camcalib/right
% if strcmp(data_source_Q,'left') || strcmp(data_source_Q,'right')
%     for i=0:ceil(num_images_Q*0.9)-1
%         
%         %                 disp(['receive image ', num2str(i),'/',num2str(num_images_P-5)]);
%         percentDone = 100 * (i+1) / (ceil(num_images_Q*0.9));
%         msg = sprintf('Percent done: %3.1f', percentDone); %Don't forget this semicolon
%         fprintf([reverseStr, msg]);
%         reverseStr = repmat(sprintf('\b'), 1, length(msg));
%         
%         waitbar(percentDone/100,waitbar_h,sprintf('%3.1f%% along ...', percentDone));
%         
%         yarpData_Q = yarp.ImageRgb;
%         yarpData_Q = portIncoming_Q.read();
%         
%         if(i+1 >= start_frame_idx_Q && i < last_frame_idx_Q)
%             % save images
%             filename = [sprintf('%04d',i) '.png'];
%             save_path = ['data/KSC/Q/',filename];
%             
%             if (sum(size(yarpData_Q)) ~= 0) %check size of bottle
%                 h=yarpData_Q.height;
%                 w=yarpData_Q.width;
%                 pixSize=yarpData_Q.getPixelSize();
%                 tool=YarpImageHelper(h, w);
%                 IN = tool.getRawImg(yarpData_Q); %use leo pape image patch
%                 TEST = reshape(IN, [h w pixSize]); %need to reshape the matrix from 1D to h w pixelSize
%                 matlabImage=uint8(zeros(h, w, pixSize)); %create an empty image with the correct dimentions
%                 r = cast(TEST(:,:,1),'uint8');  % need to cast the image from int16 to uint8
%                 g = cast(TEST(:,:,2),'uint8');
%                 b = cast(TEST(:,:,3),'uint8');
%                 matlabImage(:,:,1)= r; % copy the image to the previoulsy create matrix
%                 matlabImage(:,:,2)= g;
%                 matlabImage(:,:,3)= b;
%             else
%                 disp('incorrect image');
%             end
%             imwrite(matlabImage,save_path);
%             env = yarp.Bottle;
%             portIncoming_Q.getEnvelope(env);
%             bDataMeta_Q_buf{i+1,1} = env.toString();
%             pause(0.01);
%         end
%     end
%     disp('');
%     disp('Done receiving images for Q !');
%     close(waitbar_h);
%     %------------------------------------------
%     % for kinect
% elseif strcmp(data_source_Q,'kinect')
%     fileID = fopen('data/KSC/Q/object_locations.txt','w');
%     yarpData_Q = yarp.Bottle;
%     
%     for i=0:floor(num_images_Q*0.9/16)
%         
%         portIncoming_Q.read(yarpData_Q);
%         
%         percentDone = 100 * (i+1) / (floor(num_images_Q*0.9/16));
%         msg = sprintf('Percent done: %3.1f', percentDone); %Don't forget this semicolon
%         fprintf([reverseStr, msg]);
%         reverseStr = repmat(sprintf('\b'), 1, length(msg));
%         
%         waitbar(percentDone/100, waitbar_h, sprintf('%3.1f%% along ...', percentDone));
%         
%         if(i+1 >= start_frame_idx_Q && i < last_frame_idx_Q)
%             % save data
%             if (sum(size(yarpData_Q)) ~= 0) %check size of bottle
%                 %                 if yarpData_P.get(0).asList().size() == 16
%                 %                     for body_idx = 0:15
%                 %                         disp(yarpData_P)
%                 buf = char(yarpData_Q.get(0).asList().get(0).asList().get(1).toString());
%                 %                         body_name = char(yarpData_P.get(0).asList().get(body_idx).asList().get(0).toString());
%                 expression = '\s*';
%                 matchStr = regexp(buf,expression,'split');
%                 readData = str2double(matchStr);
%                 x = readData(1);
%                 y = readData(2);
%                 z = readData(3);
%                 fprintf(fileID, '%d\t %6.6f\t %6.6f\t %6.6f\n', i, x, y, z);
%                 %                     end
%                 %                 end
%             else
%                 disp('incorrect data');
%             end
%             
%             env = yarp.Bottle;
%             portIncoming_Q.getEnvelope(env);
%             bDataMeta_Q_buf{i+1,1} = env.toString();
%         end
%         yarpData_Q.clear();
%     end
%     fclose(fileID);
%     disp('');
%     disp('Done receiving data for Q !');
%     close(waitbar_h);
% end
% 
% %====================================
% % Close Ports
% %====================================
% port2ABM_query_Q.close;
% portIncoming_Q.close;


pause(10)


%==================================================================================================
% Joint Projection Loading for P only if it is from left / right
%==================================================================================================
if strcmp(data_source_P,'left') || strcmp(data_source_P,'right')
    %====================================
    % Connecting Ports
    %====================================
    %creating ports
    port2ABM_query_joints_P = Port;          %port for ABM of 1st data
    
    %first close the port just in case
    %(this is to try to prevent matlab from beuing unresponsive)
    port2ABM_query_joints_P.close;
    
    disp(' ')
    disp('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++');
    disp('+     Joint projection data loading for P (1.5 out of 4)    +');
    disp('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++');
    
    %open the ports
    port2ABM_query_joints_P.open('/matlab/kinematicStructure/data_read_joints_P');
    disp('_____opened port /matlab/kinematicStructure/data_read_joints_P');
    pause(0.5);
    
    connection_check = 0;
    while(~connection_check)
        Network.connect('/matlab/kinematicStructure/data_read_joints_P', '/autobiographicalMemory/rpc');
        connection_check = Network.isConnected('/matlab/kinematicStructure/data_read_joints_P','/autobiographicalMemory/rpc');
        pause(0.5)
        disp('waiting for connection...');
    end
    disp('P Port is connected to ABM!')
    
    %%
    %==================================================================================================
    % Receive Data & Save Images
    %==================================================================================================
    % get number of frames: "getImagesInfo"
    if strcmp(data_source_P,'left') || strcmp(data_source_P,'right')
        portIncoming_joints_P = BufferedPortImageRgb;
    elseif strcmp(data_source_P,'kinect')
        portIncoming_joints_P = Port;
    end
    disp(1)
    portIncoming_joints_P.close;
    
    portIncoming_joints_P.open('/matlab/kinematicStructure/data_in_joints_P');
    disp('_____opened port /matlab/kinematicStructure/data_in_joints_P');
    pause(0.5);
    disp(2)
    b_write_joints_P = yarp.Bottle;
    b_response_joints_P = yarp.Bottle;
    disp(3)
%-----------------------------------    
    command_P = ['triggerStreaming ', num2str(instance_num_P), ' ("includeAugmented" 0) ("realtime" 1) ("speedMultiplier" 0.1)'];
    b_write_joints_P.fromString(command_P);
    disp(b_write_joints_P);
    port2ABM_query_joints_P.write(b_write_joints_P,b_response_joints_P);
    disp(4)
%     b_provide_P = b_response_P.get(3).asList().get(1).asList();
    
    for i=0:b_response_joints_P.get(3).asList().size()-1
        str_buf = b_response_joints_P.get(3).asList().get(i).asList().get(0).asString();
        char_buf = char(str_buf);
disp(5)
%         if strcmp(char_buf(1:end), ['/autobiographicalMemory/jointsAwareness/',data_source_P,'_arm/to',data_source_P,'Cam/joints2DProj:o'])
        if strcmp(char_buf(1:end), '/autobiographicalMemory/jointsAwareness/left_arm/toleftCam/joints2DProj:o')
            num_joint_data_P = b_response_joints_P.get(3).asList().get(i).asList().get(1).asInt();
            final_portname_joints_P = char_buf;
            disp('>>>>>>>>>>>>>> 6');
        end
    end
    
    disp(final_portname_joints_P);
    disp(['Total number of data: ', num2str(num_joint_data_P)]);
    
    connection_check = 0;
    while(~connection_check)
        Network.connect(final_portname_joints_P, '/matlab/kinematicStructure/data_in_joints_P');
        connection_check = Network.isConnected(final_portname_joints_P, '/matlab/kinematicStructure/data_in_joints_P');
        pause(0.5)
        disp('waiting for connection...');
    end
    disp('P Port for Joint Projection data receiving is connected!')
    
    %%
    start_frame_idx_P = 1;
    last_frame_idx_P = num_joint_data_P;
    
    disp(['Actual number of retrieving joint projection data: ',num2str(ceil(num_joint_data_P*0.9))]);
    reverseStr = '';
    waitbar_h = waitbar(0,'Initializing progress bar...');
    
    fileID = fopen('data/KSC/P/jointsAwareness.txt','w');
    yarpData_joint_P = yarp.Bottle;
    
    %------------------------------------------
    % for yarp read
    for i=0:ceil(num_joint_data_P*0.9)-1

        percentDone = 100 * (i+1) / (ceil(num_joint_data_P*0.9));
        msg = sprintf('Percent done: %3.1f', percentDone); %Don't forget this semicolon
        fprintf([reverseStr, msg]);
        reverseStr = repmat(sprintf('\b'), 1, length(msg));

        waitbar(percentDone/100,waitbar_h,sprintf('%3.1f%% along ...', percentDone));

        if(i+1 >= start_frame_idx_P && i < last_frame_idx_P)

            portIncoming_joints_P.read(yarpData_joint_P);
            
            % save data
            if (sum(size(yarpData_joint_P)) ~= 0) %check size of bottle
                buf = char(yarpData_joint_P.get(0).asList().toString());
                readData = str2num(buf);
                
                for body_idx = 0:6                
                    x = readData((body_idx)*2+1);
                    y = readData((body_idx)*2+2);
                    fprintf(fileID, '%d\t %d\t %6.6f\t %6.6f\n', i+1, body_idx, x, y);
                end                
            else
                disp('incorrect image');
            end
        end
    end
    fclose(fileID);
    
    disp('');
    disp('Done receiving joint projection data for P!');
    close(waitbar_h);
    
    %%
    %====================================
    % Close Ports
    %====================================
    port2ABM_query_joints_P.close;
    portIncoming_joints_P.close;
    
    disp('PAUSED for 3 seconds...')
    pause(3);
end
