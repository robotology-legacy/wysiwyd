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
% This is a main code of kinematic structure learning from YARP ABM inputs
% Triggering this module through YARP
% ==========================================================================
clc
close all
clear all

%%
addpath(genpath('../../include/include_Matlab'));

%%
% YARP port
LoadYarp;
import yarp.Port;
import yarp.RpcServer;

shouldClose = false;

portTrigger = Port;
portTrigger.close;

disp('Going to open port /matlab/kinematicStructure/rpc');
portTrigger.open('/matlab/kinematicStructure/rpc');

disp('=================================================================================================');
disp('Enter ''startStructureLearning #num_instance [left or right camera] [first frame] [last frame]''');
disp('The module closes when ''quit'' is received');
disp('=================================================================================================');

bCommand = yarp.Bottle;
bCommand.clear();
bReply = yarp.Bottle;
bReply.clear();

%%
% YARP Trigger
while(~shouldClose)
    %%
    disp(sprintf('\n'))
    disp('Waiting for command...');
    portTrigger.read(bCommand, true);

    disp(bCommand);
    %%
    %----------------------------------------------------------------------
    if (all(isletter(bCommand.get(0))))
        if (strcmp(bCommand.get(0).toString, 'quit') || strcmp(bCommand.get(0).toString, 'close'))
            bReply.addString('close kinematic structure module');
            shouldClose = true;
            
        elseif (strcmp(bCommand.get(0).toString, 'startStructureLearning'))
            instance_num = num2str(char(bCommand.get(1).toString));
            
            % get camera selection (left of right) [default: left]
            camera_selection = char(bCommand.get(2).toString);
            if(isempty(camera_selection))
                camera_selection = 'left';
            end
            
            % get starting frame index[default: 1]
            start_frame_idx = str2num(char(bCommand.get(3).toString));
            if(isempty(start_frame_idx))
                start_frame_idx = 1;
            end
            
            % get last frame index [default: last frame]
            last_frame_idx = str2num(char(bCommand.get(4).toString));
            if(isempty(last_frame_idx))
                last_frame_idx = 'last';
            end                       
            
            disp(['Start structure learning from ABM #',instance_num, ' using ', ...
                camera_selection,' camera, ',...
                'from frame #',num2str(start_frame_idx), ' to #', num2str(last_frame_idx)]);
            disp(sprintf('\n'))

            %%
            % video input selection
            disp('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++');
            videoName = 'YARP';
            disp(['Loading images from ',videoName]);
            
            %%
            % video load
            % -- feature extraction
            % ---- saving feature values
            submodule_video_loading_feature_extraction;
            
            %%
            % converting feature data to y
            [y, W, frames, points] = submodule_cvuKltRead([pwd,'/points/',filename(1:end-4),'/point_seq_%d.txt'], 1, nFrames, 'workspace', points_total);
    
            %%
            % background subtraction
            submodule_bg_removal;
            W = W_fg;
            y = y_fg;
    
            %%
            % main algorithm
            disp('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++');           
            submodule_motion_segmentation;
            %%
            % output display
            % YARP ABM save            
            submodule_display_save_structure;

            bReply.addString('ack');
            bReply.addString(['Structure learning from ABM #',instance_num, ' is completed!']);
           
        else
            disp('Command not understood!');
            bReply.addString('Command not understood!');            
        end
    else
        disp('Wrong input bottle format!');
        bReply.addString('Wrong input bottle format!');
    end
    
    portTrigger.reply(bReply);    
    bReply.clear();
end

%%
% Module close
portTrigger.close;
