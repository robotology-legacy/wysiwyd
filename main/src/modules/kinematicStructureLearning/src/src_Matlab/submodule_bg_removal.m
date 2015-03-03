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

U = W(1:end/2,:);
V = W(end/2+1:end,:);
std_U = std(U,1);
std_V = std(V,1);

max_dev_U = max(U(2:end,:)-U(1:end-1,:),[],1);
max_dev_V = max(V(2:end,:)-V(1:end-1,:),[],1);

idx_U = ones(1,size(std_U,2));
idx_V = ones(1,size(std_V,2));

threshold_bg_removal = 0.3;
idx_U(find(std_U < threshold_bg_removal)) = 0;
idx_V(find(std_V < threshold_bg_removal)) = 0;

idx_noise_U = ones(1,size(max_dev_U,2));
idx_noise_V = ones(1,size(max_dev_V,2));

threshold_noise_removal = 100;
idx_noise_U(find(max_dev_U > threshold_noise_removal)) = 0;
idx_noise_V(find(max_dev_V > threshold_noise_removal)) = 0;

% background is zero
% idx_bg = idx_U .* idx_V;
idx_bg = idx_U .* idx_V .* idx_noise_U .* idx_noise_V;
% idx_bg = idx_U | idx_V;
seg_bg_idx = cell(2,1);
seg_bg_idx{1} = find(idx_bg == 0);
seg_bg_idx{2} = find(idx_bg == 1);


color_idx = 'rgbcmyk';
marker_idx = '+o*.xsd^v><ph';

h=figure(100);
title('Background & foreground');
for frm_idx=1:nFrames
    clf
    for i=1:2
        plot(y(1,seg_bg_idx{i},frm_idx), height-y(2,seg_bg_idx{i},frm_idx),marker_idx(mod(i,13)+1),'Color',color_idx(mod(i,7)+1));
        hold on
        axis([0, width, 0, height]);
    end
    
    pause(0.001);
end

W_fg = W(:,seg_bg_idx{2});
y_fg = y(:,seg_bg_idx{2},:);
