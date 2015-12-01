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

disp('--------------------------------------------------------------------------------------------------');
videoMode_P = input('Which input video source do you want for the first? [0:Webcam, 1:Video, 2:YARP, 3:Images] ');

switch videoMode_P
    case 0
        videoName_P = 'webcam';
        disp(['Loading video from ',videoName_P]);
    case 1
        videoName_P = 'video';
        disp(['Loading video from ',videoName_P]);
    case 2
        videoName_P = 'YARP';
        disp(['Loading video from ',videoName_P]);
    case 3
        videoName_P = 'images';
        disp(['Loading video from ',videoName_P]);
    otherwise
        disp('At least one input should be selected!!');
end

%%
% Number of Segments
numOfSegments_P = input('Number of Motion Segments for the first video? [0:Adaptive, 0<:Manual] ');

switch numOfSegments_P
    case 0
        disp('Calculate the number of motion segments adaptively!');
    otherwise
        disp(['The number of motion segments: ',numOfSegments_P]);
end

%%
disp('--------------------------------------------------------------------------------------------------');
videoMode_Q = input('Which input video source do you want for the second? [0:Webcam, 1:Video, 2:YARP, 3:Images] ');

switch videoMode_Q
    case 0
        videoName_P = 'webcam';
        disp(['Loading video from ',videoName_P]);
    case 1
        videoName_P = 'video';
        disp(['Loading video from ',videoName_P]);
    case 2
        videoName_P = 'YARP';
        disp(['Loading video from ',videoName_P]);
    case 3
        videoName_P = 'images';
        disp(['Loading video from ',videoName_P]);
    otherwise
        disp('At least one input should be selected!!');
end

%%
% Number of Segments
numOfSegments_Q = input('Number of Motion Segments for the second video? [0:Adaptive, 0<:Manual] ');

switch numOfSegments_Q
    case 0
        disp('Calculate the number of motion segments adaptively!');
    otherwise
        disp(['The number of motion segments: ',numOfSegments_Q]);
end
