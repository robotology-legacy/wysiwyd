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

function [videoName, numOfSegments] = videoInputSelect()

videoMode = input('Which input video source do you want? [0:Webcam, 1:Video, 2:YARP, 3:Images] ');

switch videoMode
    case 0
        videoName = 'webcam';
        disp(['Loading video from ',videoName]);
    case 1
        videoName = 'video';
        disp(['Loading video from ',videoName]);
    case 2
        videoName = 'YARP';
        disp(['Loading video from ',videoName]);
    case 3
        videoName = 'images';
        disp(['Loading video from ',videoName]);
    otherwise
        disp('At least one input should be selected!!');
end

%%
% Number of Segments
numOfSegments = input('Number of Motion Segments? [0:Adaptive, 0<:Manual] ');

switch numOfSegments
    case 0
        disp('Calculate the number of motion segments adaptively!');
    otherwise
        disp(['The number of motion segments: ',numOfSegments]);
end
end
