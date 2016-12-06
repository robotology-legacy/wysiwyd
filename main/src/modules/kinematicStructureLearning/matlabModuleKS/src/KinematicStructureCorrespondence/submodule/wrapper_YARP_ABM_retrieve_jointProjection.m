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

%%
%====================================
% Connecting Ports
%====================================
%creating ports
port2ABM_read = Port;          %port for ABM
portIncoming_image = BufferedPortImageRgb;
portIncoming_data = Port;

%first close the port just in case
%(this is to try to prevent matlab from beuing unresponsive)
port2ABM_read.close;
portIncoming_image.close;
portIncoming_data.close;

%open the ports
disp('_____opening ports...');
port2ABM_read.open('/matlab/kinematicStructure/ABM_data_read');
disp('_____opened port /matlab/kinematicStructure/ABM_data_read');
pause(0.5);
disp('_____done.');

disp('_____opening ports...');
portIncoming_image.open('/matlab/kinematicStructure/image_in');
disp('_____opened port /matlab/kinematicStructure/image_in');
pause(0.5);
disp('_____done.');
portIncoming_image.setStrict();

disp('_____opening ports...');
portIncoming_data.open('/matlab/kinematicStructure/data_in');
disp('_____opened port /matlab/kinematicStructure/data_in');
pause(0.5);
disp('_____done.');

disp('Going to open port /matlab/kinematicStructure/ABM_data_read');
disp('Going to open port /matlab/kinematicStructure/image_in');
disp('Going to open port /matlab/kinematicStructure/data_in');

connection_check = 0;
while(~connection_check)
    Network.connect('/matlab/kinematicStructure/ABM_data_read', '/autobiographicalMemory/rpc');
    connection_check = Network.isConnected('/matlab/kinematicStructure/ABM_data_read','/autobiographicalMemory/rpc');
    pause(1)
    disp('waiting for connection...');
end
disp('Connected!')

%%
%====================================
% Receive Data & Save Images
%====================================
% clean and make a saving directory
[SUCCESS,MESSAGE,MESSAGEID] = rmdir('data/KS','s');
mkdir('data/KS');

% get number of frames: "getImagesInfo"
b_write = yarp.Bottle;
b_response = yarp.Bottle;

command = ['triggerStreaming ', num2str(instance_num), ' ("includeAugmented" 0) ("realtime" 1) ("speedMultiplier" 0.1)'];
b_write.fromString(command);
disp(b_write);
port2ABM_read.write(b_write,b_response);

disp(b_response.toString);

b_provide = yarp.Bottle;
b_provide = b_response.get(2).asList();
disp(b_provide.toString);

for i=0:b_provide.size()-1
    str_buf = b_provide.get(i).asList().get(0).asString();
    char_buf = char(str_buf);
    
    if strcmp(char_buf(1:end-18), ['/autobiographicalMemory/icub/camcalib/',camera_selection,'/out'])
        num_images = b_provide.get(i).asList().get(1).asInt();
        final_portname_image = char_buf;
    end
end
% disp(num2str(num_images));

Network.connect(final_portname_image, '/matlab/kinematicStructure/image_in');

%%
disp('================================');
disp('Start retrieving images!');

bImageMeta_buf = cell(num_images,1);
bRawImage_buf = cell(num_images,1);

if isequal(last_frame_idx, 'last')
    last_frame_idx = num_images;
end

disp(['Actual retrieving number of images: ',num2str(ceil(num_images*0.9))]);

reverseStr = '';
waitbar_h = waitbar(0,'Initializing progress bar for image read ...');


for i=0:ceil(num_images * 0.9)-1
    percentDone = 100 * (i+1) / (ceil(num_images * 0.9));
    msg = sprintf('Percent done: %3.1f', percentDone); %Don't forget this semicolon
    fprintf([reverseStr, msg]);
    reverseStr = repmat(sprintf('\b'), 1, length(msg));
    
    waitbar(percentDone/100,waitbar_h,sprintf('%3.1f%% along ...', percentDone));
    
    yarpImage = yarp.ImageRgb;
    yarpImage = portIncoming_image.read();
    
    if(i+1 >= start_frame_idx & i < last_frame_idx)
        % save images
        filename=[sprintf('%04d',i) '.png'];
        save_path = ['data/KS/',filename];
        
        if (sum(size(yarpImage)) ~= 0) %check size of bottle
            %disp('got it..');
            h=yarpImage.height;
            w=yarpImage.width;
            pixSize=yarpImage.getPixelSize();
            tool=YarpImageHelper(h, w);
            
            IN = tool.getRawImg(yarpImage); %use leo pape image patch
            TEST = reshape(IN, [h w pixSize]); %need to reshape the matrix from 1D to h w pixelSize
            matlabImage = uint8(zeros(h, w, pixSize)); %create an empty image with the correct dimentions
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
        portIncoming_image.getEnvelope(env);
        bImageMeta_buf{i+1,1} = env.toString();
        pause(0.01);
    end
end

disp(' ');
disp('Done receiving images!');
close(waitbar_h);


% save buf data
save('data/KS/ImageMeta.mat','bImageMeta_buf');









%%
command = ['triggerStreaming ', num2str(instance_num), ' ("includeAugmented" 0) ("realtime" 0) ("speedMultiplier" 0.1)'];
b_write.fromString(command);
disp(b_write);
port2ABM_read.write(b_write,b_response);

disp(b_response.toString);

b_provide = b_response.get(3).asList().get(1).asList();
disp(b_provide.toString);

str_buf = b_provide.get(0).asString();
char_buf = char(str_buf);

if strcmp(char_buf(1:end), '/autobiographicalMemory/jointsAwareness/left_arm/toleftCam/joints2DProj:o')
    num_data = b_provide.get(1).asInt();
    final_portname_data = char_buf;
end

Network.connect(final_portname_data, '/matlab/kinematicStructure/data_in');

%%
disp('================================');
disp('Start retrieving data!');

bDataMeta_P_buf = cell(num_data,1);
bRawImage_P_buf = cell(num_data,1);

disp(['Actual retrieving number of data: ',num2str(num_data)]);

reverseStr = '';
waitbar_h = waitbar(0,'Initializing progress bar for data read ...');

fileID = fopen('data/KS/jointsAwareness.txt','w');
yarpData_joint = yarp.Bottle;

for i=0:floor(num_data/14)-2
    
    percentDone = 100 * (i+1) / (floor(num_data/14)-2);
    msg = sprintf('Percent done: %3.1f', percentDone); %Don't forget this semicolon
    fprintf([reverseStr, msg]);
    reverseStr = repmat(sprintf('\b'), 1, length(msg));
    waitbar(percentDone/100,waitbar_h,sprintf('%3.1f%% along ...', percentDone));

    if(i+1 >= 1 && i < num_data)
        
        portIncoming_data.read(yarpData_joint);
        
        % save data
        if (sum(size(yarpData_joint)) ~= 0) %check size of bottle
            buf = char(yarpData_joint.get(0).asList().toString());
%             disp(buf);
            
            readData = str2num(buf);
            for body_idx = 0:6                
%             for body_idx = 6:-1:0                
                x = readData((body_idx)*2+1);
                y = readData((body_idx)*2+2);
                fprintf(fileID, '%d\t %d\t %6.6f\t %6.6f\n', i+1, body_idx, x, y);
%                 fprintf(fileID, '%d\t %d\t %6.6f\t %6.6f\n', i+1, 6-body_idx, x, y);
            end
        else
            disp('incorrect data');
        end
        
%         env = yarp.Bottle;
%         portIncoming_data.getEnvelope(env);
%         bDataMeta_P_buf{i+1,1} = env.toString();
    end
end
fclose(fileID);

disp(' ');
disp('Done receiving images!');
close(waitbar_h);







%%
%====================================
% Close Ports
%====================================
port2ABM_read.close;
portIncoming_image.close;
portIncoming_data.close;

%%
%====================================
% Generate Video
%====================================
imagesFolder='data/KS';
imageFiles = dir(strcat(imagesFolder,'/*.png'));
num_images = size(imageFiles,1);
fileIdx = zeros(num_images,1);

VideoFile = strcat(imagesFolder,'/video');
writerObj = VideoWriter(VideoFile);

fps= 10;
writerObj.FrameRate = fps;

open(writerObj);

for t = 1:length(imageFiles)
    Frame = imread(strcat(imagesFolder,'/',imageFiles(t).name));
    writeVideo(writerObj,im2frame(Frame));
end

close(writerObj);
