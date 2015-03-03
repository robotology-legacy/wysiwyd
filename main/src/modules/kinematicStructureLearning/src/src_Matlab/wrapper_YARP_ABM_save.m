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

%first close the port just in case
%(this is to try to prevent matlab from beuing unresponsive)
port2ABM_write.close;

%open the ports
disp('_____opening ports...');
port2ABM_write.open('/matlab/kinematicStructure/image_write');
disp('_____opened port /matlab/kinematicStructure/image_write');
pause(0.5);
disp('_____done.');

disp('Going to open port /matlab/kinematicStructure/image_write');

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
load('ABM_images/ImageMeta.mat');
load('ABM_images/RawImage.mat');

augmentedLabel = 'kinematic_structure';

%%
%====================================
% Send images back to YARP ABM
%====================================
%send it back to yarp
%%
disp('================================');
disp('Start sending images!');
tic
frm_idx_buf = 1;

bDeletePrevious = yarp.Bottle;
bDeletePrevious.clear();
bResponseDelete = yarp.Bottle;
% bResponseDelete.clear();
delete_command = ['DELETE FROM visualdata WHERE instance = ', num2str(instance_num), ' AND augmented = ''', augmentedLabel, ''''];
bDeletePrevious.addString('request');
bDeletePrevious.addString(delete_command);
port2ABM_write.write(bDeletePrevious);
% disp(bResponseDelete.toString());

% for frm_idx = 1:10
for frm_idx = 1:num_images
    
    if ~isempty(bImageMeta_buf{frm_idx})
        bAugmentedImageWithMeta = yarp.Bottle;
        bAugmentedImageWithMeta.clear();
        bResponseAugmented = yarp.Bottle;
        bResponseAugmented.clear();
        
        bImageMeta_string = bImageMeta_buf{frm_idx}.toString();
        
        image_name = [sprintf('%04d',frm_idx_buf) '.png'];
        img_mat_org = imread(['result/images/',image_name]);
        [h,w,pixSize] = size(img_mat_org);
        
        img_mat_org_r = reshape(img_mat_org(:,:,1)',[1, w*h]);
        img_mat_org_g = reshape(img_mat_org(:,:,2)',[1, w*h]);
        img_mat_org_b = reshape(img_mat_org(:,:,3)',[1, w*h]);
        img_mat_org_vec = [img_mat_org_r ; img_mat_org_g ; img_mat_org_b];
        img_mat = reshape(img_mat_org_vec,[1, h*w*pixSize]);
        
%         img_mat = reshape(img_mat_org2, [h*w*pixSize 1])'; %reshape the matlab image to 1D
        img_string = ['[mat] [rgb] (',num2str(pixSize),' ',num2str(h*w*pixSize),' 8 ',num2str(w),' ',num2str(h),') {',num2str(img_mat),'}'];   
              
        string_buf = ['saveAugmentedImages ((',char(bImageMeta_string),') ', augmentedLabel, ' (', img_string, '))'];
        bAugmentedImageWithMeta.fromString(string_buf);
        port2ABM_write.write(bAugmentedImageWithMeta, bResponseAugmented);
        
        disp(bResponseAugmented.toString());
        frm_idx_buf = frm_idx_buf + 1;
    end
end

time = toc;
fprintf('Sending back to yarp took %f seconds \n', time);

%%
%====================================
% Close Ports
%====================================
disp('Closing the port');
port2ABM_write.close;
