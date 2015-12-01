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

% ImagesFolder=uigetdir('/home/human/robot/dataset/DATA_INTE');
ImagesFolder=uigetdir('/home/human/robot/wysiwyd/main/build/bin');

% jpegFiles = dir(strcat(ImagesFolder,'/*.jpg'));
% imageFiles = dir(strcat(ImagesFolder,'/*.ppm'));
imageFiles = dir(strcat(ImagesFolder,'/*.tif'));

numImages = size(imageFiles,1);

% fileNames = zeros(numImages,1);
fileIdx = zeros(numImages,1);

for i=1:numImages
    fileNames = char(imageFiles(i).name)
    fileIdx(i) = str2num(fileNames(5:end-4));
end
[S,S ] =sort(fileIdx);
% S = [imageFiles(:).datenum]; 
% [S,S] = sort(S);
imageFilesS = imageFiles(S);

VideoFile=strcat(pwd,'/data/video');
writerObj = VideoWriter(VideoFile);

fps= 10; 
writerObj.FrameRate = fps;

open(writerObj);

for t= 1:length(imageFilesS)
     Frame=imread(strcat(ImagesFolder,'/',imageFilesS(t).name));
     writeVideo(writerObj,im2frame(Frame));
end

close(writerObj);
