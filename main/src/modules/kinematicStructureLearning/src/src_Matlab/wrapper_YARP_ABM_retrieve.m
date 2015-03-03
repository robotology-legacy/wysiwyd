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

%first close the port just in case
%(this is to try to prevent matlab from beuing unresponsive)
port2ABM_read.close;

%open the ports
disp('_____opening ports...');
port2ABM_read.open('/matlab/kinematicStructure/image_read');
disp('_____opened port /matlab/kinematicStructure/image_read');
pause(0.5);
disp('_____done.');

disp('Going to open port /matlab/kinematicStructure/image_read');

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

command = ['getImagesInfo ', num2str(instance_num), ' 0'];
b_write.fromString(command);
disp(b_write);
port2ABM_read.write(b_write,b_response);

disp(b_response.toString);

num_images = str2num(b_response.get(0));
num_image_ports = sum(b_response.get(1).size());

command = ['storeImageOIDs ', num2str(instance_num)];
b_write.fromString(command);
disp(b_write);
port2ABM_read.write(b_write,b_response);

%%
disp('================================');
disp('Start retrieving images!');

bImageMeta_buf = cell(num_images,num_image_ports);
bRawImage_buf = cell(num_images,num_image_ports);

if isequal(last_frame_idx, 'last')
    last_frame_idx = num_images;
end

for frm_idx = start_frame_idx-1:last_frame_idx-1
    
    % get the images: "provideImagesByFrame"
    command = ['provideImagesByFrame ', num2str(instance_num), ' ', num2str(frm_idx), ' 0', ' /icub/camcalib/',camera_selection,'/out'];
    b_write.fromString(command);
    disp(b_write);
    port2ABM_read.write(b_write,b_response);
    
    if(b_response.get(0).toString ~= 'ack')
        disp('Error')
    else
        bImagesWithMeta = yarp.Bottle;
        bImagesWithMeta = b_response.get(1).asList();
        
        i=0;
        bSingleImageWithMeta = yarp.Bottle;
        bSingleImageWithMeta = bImagesWithMeta.get(i).asList();
        
        % Image Metadata
        bImageMeta = yarp.Bottle;
        bImageMeta = bSingleImageWithMeta.get(0).asList();
        bImageMeta_buf{frm_idx+1,i+1} = bImageMeta.toString();
        
        % Raw Images
        bRawImage = yarp.Bottle;
        bRawImage = bSingleImageWithMeta.get(1).asList();
        
        % Imamge Conversion
        image_data_buf = str2num(bRawImage.get(2).toString());
        pixSize = image_data_buf(1);
        h = image_data_buf(5);
        w = image_data_buf(4);
        img_buf = uint8(zeros(h, w, pixSize));
        
        img_raw_seq = uint8( sscanf( char( bRawImage.get(3).toString() ), '%d' )' );
        img_raw_mtx = reshape(img_raw_seq,[pixSize,w,h]);
        img_buf(:,:,1) = squeeze(img_raw_mtx(1,:,:))';
        img_buf(:,:,2) = squeeze(img_raw_mtx(2,:,:))';
        img_buf(:,:,3) = squeeze(img_raw_mtx(3,:,:))';

        % save images
        filename=[sprintf('%04d',frm_idx) '.png'];
        save_path = ['ABM_images/',num2str(i),'_',filename];
        imwrite(img_buf,save_path);
    end
end

% save buf data
save('ABM_images/ImageMeta.mat','bImageMeta_buf');
save('ABM_images/RawImage.mat','bRawImage_buf');

%%
%====================================
% Close Ports
%====================================
port2ABM_read.close;

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
