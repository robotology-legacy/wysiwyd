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
% Parameter Initialisation
%====================================
% initialise YARP
LoadYarp;
import yarp.Port
import yarp.BufferedPortImageRgb
import yarp.Bottle
import yarp.Time
import yarp.ImageRgb
import yarp.Image
import yarp.PixelRgb
import yarp.Network

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
[SUCCESS,MESSAGE,MESSAGEID] = rmdir('ABM_images','s');
mkdir('ABM_images');

% get number of frames: "getImagesInfo"
b_write=yarp.Bottle;
b_response=yarp.Bottle;

% command = ['getImagesInfo ', num2str(instance_num), ' 0'];
% b_write.fromString(command);
% disp(b_write);
% port2ABM_read.write(b_write,b_response);

% disp(b_response.toString);

command = ['triggerStreaming ', num2str(instance_num), ' ("includeAugmented" 0) ("realtime" 0)'];
b_write.fromString(command);
disp(b_write);
port2ABM_read.write(b_write,b_response);

disp(b_response.toString);


% num_images = str2num(b_response.get(0));
% num_image_ports = sum(b_response.get(1).size());

%     Bottle* bListImgProviders = bRespImagesInfo.get(2).asList();
%     for(int i=0; i<bListImgProviders->size(); i++) {
%         if(bListImgProviders->get(i).asList()->get(0).asString()==portName) {
%             numberOfImages = bListImgProviders->get(i).asList()->get(1).asInt();
%         }
%     }
%     yDebug() << "Num images: " << numberOfImages;

b_provide = yarp.Bottle;
b_provide = b_response.get(2).asList();
disp(b_provide.toString);

for i=0:b_provide.size()-1
    if strcmp(b_provide.get(i).asList().get(0).asString(), ['/autobiographicalMemory/icub/camcalib/',camera_selection,'/out'])
        num_images = b_provide.get(i).asList().get(1).asInt();
    end
end
disp(num2str(num_images));

Network.connect(['/autobiographicalMemory/icub/camcalib/',camera_selection,'/out'], '/matlab/kinematicStructure/imagein');

% command = ['storeImageOIDs ', num2str(instance_num)];
% b_write.fromString(command);
% disp(b_write);
% port2ABM_read.write(b_write,b_response);

%%
disp('================================');
disp('Start retrieving images!');

bImageMeta_buf = cell(num_images,1);
bRawImage_buf = cell(num_images,1);

if isequal(last_frame_idx, 'last')
    last_frame_idx = num_images;
end

for i=0:num_images-1
    disp(['receive image ', num2str(i)]);
    yarpImage = yarp.ImageRgb;
    yarpImage = portIncoming.read();
    
    % save images
    filename=[sprintf('%04d',i) '.png'];
    save_path = ['ABM_images/',filename];
    
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
end

% for frm_idx = start_frame_idx-1:last_frame_idx-1
%
%     % get the images: "provideImagesByFrame"
%     command = ['provideImagesByFrame ', num2str(instance_num), ' ', num2str(frm_idx), ' 0', ' /icub/camcalib/',camera_selection,'/out'];
%     b_write.fromString(command);
%     disp(b_write);
%     port2ABM_read.write(b_write,b_response);
%
%     if(b_response.get(0).toString ~= 'ack')
%         disp('Error')
%     else
%         bImagesWithMeta = yarp.Bottle;
%         bImagesWithMeta = b_response.get(1).asList();
%
%         i=0;
%         bSingleImageWithMeta = yarp.Bottle;
%         bSingleImageWithMeta = bImagesWithMeta.get(i).asList();
%
%         % Image Metadata
%         bImageMeta = yarp.Bottle;
%         bImageMeta = bSingleImageWithMeta.get(0).asList();
%         bImageMeta_buf{frm_idx+1,i+1} = bImageMeta.toString();
%
%         % Raw Images
%         bRawImage = yarp.Bottle;
%         bRawImage = bSingleImageWithMeta.get(1).asList();
%
%         % Imamge Conversion
%         image_data_buf = str2num(bRawImage.get(2).toString());
%         pixSize = image_data_buf(1);
%         h = image_data_buf(5);
%         w = image_data_buf(4);
%         img_buf = uint8(zeros(h, w, pixSize));
%
%         img_raw_seq = uint8( sscanf( char( bRawImage.get(3).toString() ), '%d' )' );
%         img_raw_mtx = reshape(img_raw_seq,[pixSize,w,h]);
%         img_buf(:,:,1) = squeeze(img_raw_mtx(1,:,:))';
%         img_buf(:,:,2) = squeeze(img_raw_mtx(2,:,:))';
%         img_buf(:,:,3) = squeeze(img_raw_mtx(3,:,:))';
%
%         % save images
%         filename=[sprintf('%04d',frm_idx) '.png'];
%         save_path = ['ABM_images/',num2str(i),'_',filename];
%         imwrite(img_buf,save_path);
%     end
% end

% save buf data
save('ABM_images/ImageMeta.mat','bImageMeta_buf');
% save('ABM_images/RawImage.mat','bRawImage_buf');

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
imagesFolder='ABM_images';
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
