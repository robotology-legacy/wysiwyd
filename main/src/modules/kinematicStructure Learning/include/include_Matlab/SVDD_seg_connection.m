skeleton_seg_connection_mean = mean(full_connection_weight,3);
skeleton_seg_connection_mean = skeleton_seg_connection_mean' + skeleton_seg_connection_mean;
skeleton_dist = 1./skeleton_seg_connection_mean;
skeleton_dist(find(skeleton_dist == Inf)) = 0;

skeleton_MST_idx1 = [];
skeleton_MST_idx2 = [];
skeleton_MST_W_buf = [];
skeleton_MST_k_knn = num_seg-1;

for i = 1:num_seg
    [skeleton_MST_Y,skeleton_MST_I] = sort(skeleton_dist(i,:));
    for l = 1:skeleton_MST_k_knn
        skeleton_MST_idx1 = [skeleton_MST_idx1,i];
        skeleton_MST_idx2 = [skeleton_MST_idx2,skeleton_MST_I(l+1)];
        skeleton_MST_W_buf = [skeleton_MST_W_buf,skeleton_dist(i,skeleton_MST_I(l+1))];
    end
end
skeleton_MST_DG = sparse(skeleton_MST_idx1,skeleton_MST_idx2,skeleton_MST_W_buf);
skeleton_MST_UG = tril(skeleton_MST_DG + skeleton_MST_DG');

% Find the minimum spanning tree of UG
[skeleton_MST_ST,skeleton_MST_pred] = graphminspantree(skeleton_MST_UG,'Method','Kruskal');
[skeleton_MST_ii,skeleton_MST_jj,skeleton_MST_ss] = find(skeleton_MST_ST);

%%
% Drawing the connections
if integration_seg_connect_plot_ON
    if result_save_ON
        writerobj = VideoWriter([result_save_folder,'/ms_skeleton_connection.avi']);
        open(writerobj);
    end
    
    h=figure(32);
    
    for frm_idx = 1:num_frames;
        clf
        plot(y(1,:,frm_idx),height-y(2,:,frm_idx),'b.');
        %     plot(y(1,:,frm),y(2,:,frm),'b.');
        hold on
        plot(seg_center(1,:,frm_idx),height-seg_center(2,:,frm_idx),'ro');
        %     plot(seg_y(1,:,frm),seg_y(2,:,frm),'ro');
        for i = 1:num_seg
            text(seg_center(1,i,frm_idx)+3, height-seg_center(2,i,frm_idx)+3, num2str(i), 'Color', 'r');
            %         text(seg_y(1,i,frm)+3, seg_y(2,i,frm)+3, num2str(i), 'Color', 'r');
        end
        axis([0 width 0 height]);
        %     axis equal
        hold on
        for m = 1:size(skeleton_MST_ii,1)
            plot([seg_center(1,skeleton_MST_ii(m),frm_idx),seg_center(1,skeleton_MST_jj(m),frm_idx)],height-[seg_center(2,skeleton_MST_ii(m),frm_idx),seg_center(2,skeleton_MST_jj(m),frm_idx)],'k-');
            %         plot([seg_y(1,ii(m),frm),seg_y(1,jj(m),frm)],[seg_y(2,ii(m),frm),seg_y(2,jj(m),frm)],'k-');
        end
        pause(0.001)
        if result_save_ON
            F = getframe(h);
            writeVideo(writerobj,F);
        end
    end
    if result_save_ON
        close(writerobj);
    end
end