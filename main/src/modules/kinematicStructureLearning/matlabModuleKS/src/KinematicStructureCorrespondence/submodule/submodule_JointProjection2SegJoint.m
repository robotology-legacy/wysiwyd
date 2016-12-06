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

function KineStruct = submodule_JointProjection2SegJoint(KineStruct)
%%
% Draw structure with points & motor positions
h_points = figure(1002);
color_idx = 'rgbcmy';
marker_idx = '............';

fileID = fopen('data/KS/jointsAwareness.txt','r');
formatSpec = '%d %d %f %f';
sizeA = [4 Inf];
jointsAwareness_points = fscanf(fileID,formatSpec,sizeA);
jointsAwareness_points = jointsAwareness_points';
fclose(fileID);

max_value = max(jointsAwareness_points);
num_jointsAwareness_points = max_value(1);
num_jointsAwareness_joints = max_value(2);
sampling_freq = KineStruct.num_frames / num_jointsAwareness_points;
jntAware_idx = 1;

jointsAwareness_points_selected = cell(num_jointsAwareness_joints+1,1);

for frm_idx = 1:KineStruct.num_frames
    
    curFrame = read(videoObj,frm_idx);
    
    clf
    imshow(curFrame,'Border','tight');
    hold on
    
    % feature points
    for i=1:KineStruct.num_seg
        plot(KineStruct.y(1,KineStruct.seg_idx{i},frm_idx),...
            KineStruct.y(2,KineStruct.seg_idx{i},frm_idx),...
            marker_idx(mod(i,12)+1),...
            'Color', color_idx(mod(i,6)+1),...
            'LineWidth', 3,...
            'MarkerSize', 10);
        hold on
    end
    
    color_value = 0.99;
    
    for m = 1:size(KineStruct.structure_i,1)
        hold on
        joint_pts_buf = KineStruct.joint_center{KineStruct.structure_i(m),KineStruct.structure_j(m)};
        
        % Connection
        plot([KineStruct.seg_center(1,KineStruct.structure_i(m),frm_idx), joint_pts_buf(1,frm_idx)],...
            [KineStruct.seg_center(2,KineStruct.structure_i(m),frm_idx), joint_pts_buf(2,frm_idx)],...
            '-','Color',[color_value,color_value,color_value],'LineWidth',4);
        plot([KineStruct.seg_center(1,KineStruct.structure_j(m),frm_idx), joint_pts_buf(1,frm_idx)],...
            [KineStruct.seg_center(2,KineStruct.structure_j(m),frm_idx), joint_pts_buf(2,frm_idx)],...
            '-','Color',[color_value,color_value,color_value],'LineWidth',4);
        
        % Node
        plot([KineStruct.seg_center(1,KineStruct.structure_i(m),frm_idx)],...
            [KineStruct.seg_center(2,KineStruct.structure_i(m),frm_idx)],'-ws',...
            'LineWidth',3,...
            'MarkerSize',15,...
            'MarkerEdgeColor',[color_value,color_value,color_value],...
            'MarkerFaceColor',[1.0,0.2,0.2]);
        plot([KineStruct.seg_center(1,KineStruct.structure_j(m),frm_idx)],...
            [KineStruct.seg_center(2,KineStruct.structure_j(m),frm_idx)],'-ws',...
            'LineWidth',3,...
            'MarkerSize',15,...
            'MarkerEdgeColor',[color_value,color_value,color_value],...
            'MarkerFaceColor',[1.0,0.2,0.2]);
        
        text(KineStruct.seg_center(1,KineStruct.structure_i(m),frm_idx)+3, KineStruct.seg_center(2,KineStruct.structure_i(m),frm_idx)+3, num2str(KineStruct.structure_i(m)), 'Color',[0.9,0.7,0.9]);
        text(KineStruct.seg_center(1,KineStruct.structure_j(m),frm_idx)+6, KineStruct.seg_center(2,KineStruct.structure_j(m),frm_idx)+6, num2str(KineStruct.structure_j(m)), 'Color',[0.9,0.7,0.9]);
        
        % Joint
        plot(joint_pts_buf(1,frm_idx), joint_pts_buf(2,frm_idx),'wo',...
            'LineWidth',1,...
            'MarkerSize',9,...
            'MarkerEdgeColor',[color_value,color_value,color_value],...
            'MarkerFaceColor',[1.0,0.647,0.0]);
        plot(joint_pts_buf(1,frm_idx), joint_pts_buf(2,frm_idx),'wx',...
            'LineWidth',1,...
            'MarkerSize',9,...
            'MarkerEdgeColor',[color_value,color_value,color_value],...
            'MarkerFaceColor',[1.0,0.647,0.0]);
    end
    
    jntAware_sampling_idx = ceil(frm_idx/sampling_freq);
    
    for jntAware_idx = 0:num_jointsAwareness_joints
        x_buf = jointsAwareness_points((jntAware_sampling_idx-1)*7 + jntAware_idx+1, 3);
        y_buf = jointsAwareness_points((jntAware_sampling_idx-1)*7 + jntAware_idx+1, 4);
        
        if (x_buf < 320 && x_buf > 0)
            if (y_buf < 240 && y_buf > 0)
                jointsAwareness_points_selected{jntAware_idx+1} = [jointsAwareness_points_selected{jntAware_idx+1};[x_buf,y_buf]];
                plot(x_buf, y_buf, 'y*');
                text(x_buf+3+jntAware_idx, y_buf+3+jntAware_idx, num2str(jntAware_idx), 'Color',[0.9,0.7,0.9]);
            end
        end
        
    end
    
    pause(0.003)
    
end

%%
% matching seg centres to motor joints
num_jointProjected = 0;
idx_jointProjected = [];
for jntAware_idx = 0:num_jointsAwareness_joints
    if ~isempty(jointsAwareness_points_selected{jntAware_idx+1})
        num_jointProjected = num_jointProjected+1;
        idx_jointProjected = [idx_jointProjected,jntAware_idx];
    end
end

dist_jntProj2jntSeg = zeros(num_jointProjected, KineStruct.num_seg);

for frm_idx = 1:KineStruct.num_frames
    for idx_jntSeg = 1:KineStruct.num_seg
        for idx_jntProj = 1:num_jointProjected
            x_jntSeg = KineStruct.seg_center(1,idx_jntSeg,frm_idx);
            y_jntSeg = KineStruct.seg_center(2,idx_jntSeg,frm_idx);
            
            x_jntProj = jointsAwareness_points_selected{idx_jointProjected(idx_jntProj)+1}(frm_idx,1);
            y_jntProj = jointsAwareness_points_selected{idx_jointProjected(idx_jntProj)+1}(frm_idx,2);
            
            %             [frm_idx, idx_jntSeg, idx_jntProj, x_jntSeg, y_jntSeg, x_jntProj, y_jntProj]
            dist_buf = sqrt((x_jntSeg-x_jntProj)^2 + (y_jntSeg-y_jntProj)^2);
            dist_jntProj2jntSeg(idx_jntProj,idx_jntSeg) = dist_jntProj2jntSeg(idx_jntProj,idx_jntSeg) + dist_buf;
        end
    end
end

dist_jntProj2jntSeg = dist_jntProj2jntSeg / KineStruct.num_frames;

%%
matching_out = closest_jnt_finding(dist_jntProj2jntSeg);

seg2jnt = zeros(size(matching_out,2),1);
for i=1:size(matching_out,2)
    seg2jnt(i) = find(matching_out(:,i));
end
KineStruct.seg2jnt = seg2jnt;
end