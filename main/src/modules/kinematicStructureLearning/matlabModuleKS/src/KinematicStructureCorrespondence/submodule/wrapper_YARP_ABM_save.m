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

% Sending Image sequence with augmented results to YARP ABM
% This code is implemented based on FastImageConversion.m
% ==========================================================================

%%
%====================================
% Connecting Ports
%====================================
%creating ports
port2ABM_write  = Port;          %port for ABM
portOutgoing = Port;

%first close the port just in case
%(this is to try to prevent matlab from beuing unresponsive)
port2ABM_write.close;
portOutgoing.close;

%open the ports
disp('_____opening ports...');
port2ABM_write.open('/matlab/kinematicStructure/image_write');
disp('_____opened port /matlab/kinematicStructure/image_write');
pause(0.5);
disp('_____done.');

disp('_____opening ports...');
portOutgoing.open('/matlab/kinematicStructure/imageout');
disp('_____opened port /matlab/kinematicStructure/imageout');
pause(0.5);
disp('_____done.');

disp('Going to open port /matlab/kinematicStructure/image_write');
disp('Going to open port /matlab/kinematicStructure/imageout');

connection_check = 0;
while(~connection_check)
    Network.connect('/matlab/kinematicStructure/image_write', '/autobiographicalMemory/rpc');
    connection_check = Network.isConnected('/matlab/kinematicStructure/image_write','/autobiographicalMemory/rpc');
    pause(1)
    disp('waiting for connection...');
end
disp('Connected!')

%%
%====================================
% Load Images
%====================================
load('data/KS/ImageMeta.mat');
augmentedLabel = 'KS';
% augmentedLabel = '__________';

%%
%====================================
% Send images back to YARP ABM
%====================================
%send it back to yarp
%%
disp('================================');
disp('Start sending images!');
tic

connection_check = 0;
while(~connection_check)
    Network.connect('/matlab/kinematicStructure/imageout', '/autobiographicalMemory/augmented:i');
    connection_check = Network.isConnected('/matlab/kinematicStructure/imageout','/autobiographicalMemory/augmented:i');
    pause(1)
    disp('waiting for connection...');
end
disp('Connected!')

reverseStr = '';

for i = 1:num_images
    if ~isempty(bImageMeta_buf{i})
        bAugmentedImageWithMeta = yarp.Bottle;
        bAugmentedImageWithMeta.clear();
        bResponseAugmented = yarp.Bottle;
        bResponseAugmented.clear();
        
        bImageMeta_string = bImageMeta_buf{i}.toString();
        
        image_name = [sprintf('%04d',i) '.png'];
%         fprintf(['Saving ', image_name, '\n']);
        img_mat_org = imread(['result/KS/images/points/',image_name]);
        [h,w,pixSize] = size(img_mat_org);
        tool=YarpImageHelper(h, w);
        
        img = yarp.ImageRgb(); %create a new yarp image to send results to ports
        img.resize(w,h);   %resize it to the desired size
        img.zero();        %set all pixels to black
        img_mat_org = reshape(img_mat_org, [h*w*pixSize 1]); %reshape the matlab image to 1D
        tempImg = cast(img_mat_org ,'int16');   %cast it to int16
        img = tool.setRawImg(tempImg, h, w, pixSize); % pass it to the setRawImg function (returns the full image)
        
        bMetaBottle = yarp.Bottle;
        bMetaBottle.fromString(bImageMeta_string);
        bMetaBottle.addString(augmentedLabel);
        portOutgoing.setEnvelope(bMetaBottle);
        portOutgoing.write(img); %send it off
        
        percentDone = 100 * i / num_images;
        msg = sprintf('Sending completed percentage: %3.1f', percentDone); %Don't forget this semicolon
        fprintf([reverseStr, msg]);
        reverseStr = repmat(sprintf('\b'), 1, length(msg));        
    end
end

disp(' ');
disp('Done sending images!');

Network.disconnect('/matlab/kinematicStructure/imageout', '/autobiographicalMemory/augmented:i');

time = toc;
fprintf('Sending back to yarp took %f seconds \n', time);

%%
%====================================
% Close Ports
%====================================
disp('Closing the port');
port2ABM_write.close;
portOutgoing.close;
