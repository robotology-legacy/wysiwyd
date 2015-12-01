function KineStruct = findingJoint(KineStruct,ctrl_param)

%%
joint_buf = cell(KineStruct.num_seg, KineStruct.num_seg);

K = 5;

for i = 1:KineStruct.num_seg
    for j = 1:KineStruct.num_seg
        if i~=j
            
            I_idx = KineStruct.seg_idx{i};
            J_idx = KineStruct.seg_idx{j};
            
            size_pts_I = length(I_idx);
            size_pts_J = length(J_idx);
            
            dist_I_against_J_buf = zeros(size_pts_I, KineStruct.num_frames);
            dist_J_against_I_buf = zeros(size_pts_J, KineStruct.num_frames);
            
            for frm_idx = 1:KineStruct.num_frames
                query_pt_I = KineStruct.seg_center(:,i,frm_idx);
                query_pt_J = KineStruct.seg_center(:,j,frm_idx);
                ref_pts_I = KineStruct.y(1:2,KineStruct.seg_idx{i},frm_idx);
                ref_pts_J = KineStruct.y(1:2,KineStruct.seg_idx{j},frm_idx);
                
                dist_I_against_J_buf(:,frm_idx) = sqrt(sum((repmat(query_pt_J,[1,size_pts_I]) - ref_pts_I).^2,1))';
                dist_J_against_I_buf(:,frm_idx) = sqrt(sum((repmat(query_pt_I,[1,size_pts_J]) - ref_pts_J).^2,1))';
                
            end
            
            %             median_dist_I = median(dist_I_against_J_buf
            %             mean_dist_I = mean(dist_I_against_J_buf,2);
            %             mean_dist_J = mean(dist_J_against_I_buf,2);
            
            mean_dist_I = median(dist_I_against_J_buf,2);
            mean_dist_J = median(dist_J_against_I_buf,2);
            
            [Y, sort_idx_I_against_J] = sort(mean_dist_I);
            [Y, sort_idx_J_against_I] = sort(mean_dist_J);
            
            %             nn_idx = annquery(ref_pts, query_pts, 5);
            joint_buf_buf = zeros(2,KineStruct.num_frames);
            for frm_idx = 1:KineStruct.num_frames
                joint_buf_buf(:,frm_idx) = mean([KineStruct.y(1:2,I_idx(sort_idx_I_against_J(1:K)),frm_idx), KineStruct.y(1:2,J_idx(sort_idx_J_against_I(1:K)),frm_idx)],2);
            end
            joint_buf{i,j} = joint_buf_buf;
        end
    end
end

KineStruct.joint_center = joint_buf;


%% Validation by Visualisation
if ctrl_param.KineStruct.joint_connect_plot_ON
    color_idx = 'rgbcmyk';
    marker_idx = '+o*.xsd^v><ph';
    
    h=figure;
    for frm_idx=1:KineStruct.num_frames
        clf
        for i=1:KineStruct.num_seg
            %             plot(y(1,seg_idx{i},f), height-y(2,seg_idx{i},f),'.','Color',color_idx(mod(i,7)+1));
            %             plot(y(1,seg_idx{i},frm_idx), height-y(2,seg_idx{i},frm_idx),'Marker',marker_idx(mod(i,13)+1),'Color',color_idx(mod(i,7)+1));
            plot(KineStruct.y(1,KineStruct.seg_idx{i},frm_idx), KineStruct.height-KineStruct.y(2,KineStruct.seg_idx{i},frm_idx),marker_idx(mod(i,13)+1),'Color',color_idx(mod(i,7)+1));
            %             plot(y(1,seg_idx{i},f), height-y(2,seg_idx{i},f),'.','Color',[1-i/num_seg,i/num_seg,1-i/num_seg]);
            hold on
            axis([0, KineStruct.width, 0, KineStruct.height]);
            hold on
            plot(KineStruct.seg_center(1,:,frm_idx),KineStruct.height-KineStruct.seg_center(2,:,frm_idx),'ro',...
                'MarkerSize',15,...
                'MarkerEdgeColor','k',...
                'MarkerFaceColor',[1.0,0.2,0.2]);
            
        end
        
        for idx = 1:length(KineStruct.structure_i)
            joint_pts_buf = KineStruct.joint_center{KineStruct.structure_i(idx),KineStruct.structure_j(idx)};
            plot(joint_pts_buf(1,frm_idx),KineStruct.height-joint_pts_buf(2,frm_idx),'ko');
            
            plot([KineStruct.seg_center(1,KineStruct.structure_i(idx),frm_idx),joint_pts_buf(1,frm_idx),],[KineStruct.height-KineStruct.seg_center(2,KineStruct.structure_i(idx),frm_idx),KineStruct.height-joint_pts_buf(2,frm_idx)],'r-d');
            plot([KineStruct.seg_center(1,KineStruct.structure_j(idx),frm_idx),joint_pts_buf(1,frm_idx),],[KineStruct.height-KineStruct.seg_center(2,KineStruct.structure_j(idx),frm_idx),KineStruct.height-joint_pts_buf(2,frm_idx)],'r-d');
            
        end
        
        pause(0.01);
    end
end

end