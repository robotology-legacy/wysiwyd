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
portIncoming = BufferedPortImageRgb;

%first close the port just in case
%(this is to try to prevent matlab from beuing unresponsive)
port2ABM_read.close;
portIncoming.close;

%open the ports
disp('_____opening ports...');
port2ABM_read.open('/matlab/kinematicStructure/image_read');
disp('_____opened port /matlab/kinematicStructure/image_read');
pause(0.5);
disp('_____done.');

disp('_____opening ports...');
portIncoming.open('/matlab/kinematicStructure/imagein');
disp('_____opened port /matlab/kinematicStructure/imagein');
pause(0.5);
disp('_____done.');
portIncoming.setStrict();

disp('Going to open port /matlab/kinematicStructure/image_read');
disp('Going to open port /matlab/kinematicStructure/imagein');

connection_check = 0;
while(~connection_check)
    Network.connect('/matlab/kinematicStructure/image_read', '/autobiographicalMemory/rpc');
    connection_check = Network.isConnected('/matlab/kinematicStructure/image_read','/autobiographicalMemory/rpc');
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
b_write=yarp.Bottle;
b_response=yarp.Bottle;

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
        final_portname = char_buf;
    end
end
% disp(num2str(num_images));

Network.connect(final_portname, '/matlab/kinematicStructure/imagein');

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
waitbar_h = waitbar(0,'Initializing progress bar...');


for i=0:ceil(num_images * 0.9)-1   
    percentDone = 100 * (i+1) / (ceil(num_images * 0.9));
    msg = sprintf('Percent done: %3.1f', percentDone); %Don't forget this semicolon
    fprintf([reverseStr, msg]);
    reverseStr = repmat(sprintf('\b'), 1, length(msg));
    
    waitbar(percentDone/100,waitbar_h,sprintf('%3.1f%% along ...', percentDone));
    
    yarpImage = yarp.ImageRgb;
    yarpImage = portIncoming.read();
    
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
        portIncoming.getEnvelope(env);
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
%====================================
% Close Ports
%====================================
port2ABM_read.close;
portIncoming.close;

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
