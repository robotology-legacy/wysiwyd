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

% function [y, W, frames, points] = submodule_cvuKltRead(format, start_Frame, end_Frame, data_loading_idx, points_total)
function [y, W, frames, points] = submodule_cvuKltRead(cdata)

format = ['data/KS/points/',cdata.filename(1:end-4),'/point_seq_%d.txt'];
start_Frame = 1;
end_Frame = cdata.nFrames;
data_loading_idx = 'workspace';
points_total = cdata.points_total;

sFrame = start_Frame;
eFrame = end_Frame;

switch data_loading_idx
    case 'file'
        disp('Loading feature points...');
        for f = sFrame:eFrame
            buf = importdata(sprintf(format, f));
            buf2 = [];  

            for p = 1:size(buf,1)
                buf2 = [buf2;str2num(buf{p})];
            end
            W(f,:,:) = buf2;
        end
    case 'workspace'
        disp('generating dataset...');
        for f=sFrame:eFrame
            buf = points_total{f};
            buf2 = [];  

            for p = 1:size(buf,1)
                buf2 = [buf2;buf(p,:)];
            end
            W(f,:,:) = buf2;            
        end
end

U = W(:, :, 1);
V = W(:, :, 2);
W = [U ; V];
W = round(W);
W_zero_idx = find(sum(W,2)==0);
W(W_zero_idx,:) = [];

%%
% duplicated values
disp('Removing duplicated points...');
num_points = size(W,2);
num_frames = size(W,1)/2;
for f = 1:num_frames
    buf = [W(f,:);W(f+num_frames,:)];
    [C,IA,IC] = unique(buf','rows');
    idx_unique = sort(IA);
    idx_dup = setdiff([1:num_points],idx_unique);
    W([f,f+num_frames],idx_dup) = -1;
end

%%
% filtering by mask
disp('Filtering by background mask...');
Wx = W(1,:);
Wy = W(1+num_frames,:);

mask = submodule_bg_removal_mask(cdata);
if ~isempty(mask)
    for i=1:size(Wx,2)
        if Wx(i) > 0 && Wy(i) > 0
            if mask(Wy(i),Wx(i)) == 0
                W(:,i) = -1;
            end
        end
    end
end

%%
% Delete errored tracking points (whose value -1)
W = W(:, sum(W < 0, 1) == 0);

frames = size(W,1)/2;
points = size(W,2); 

disp('Generating y...');
for p=1:points
    for f = 1:frames
        y(1,p,f) = W(f,p);
        y(2,p,f) = W(f+frames,p);
        y(3,p,f) = 1.0;
    end
end
end
