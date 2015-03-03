% integrated_dist = motion_dist .* skeleton_dist;
motion_dist_max = max(max(motion_dist));
skeleton_dist_max = max(max(skeleton_dist));
motion_dist = motion_dist / motion_dist_max;
skeleton_dist = skeleton_dist / skeleton_dist_max;

integrated_dist = motion_dist .* skeleton_dist;

integrated_idx1 = [];
integrated_idx2 = [];
integrated_W_buf = [];
integrated_k_knn = num_seg-1;

for i = 1:num_seg
    [integrated_Y,integrated_I] = sort(integrated_dist(i,:));
    for l = 1:integrated_k_knn
        integrated_idx1 = [integrated_idx1,i];
        integrated_idx2 = [integrated_idx2,integrated_I(l+1)];
        integrated_W_buf = [integrated_W_buf,integrated_dist(i,integrated_I(l+1))];
    end
end
integrated_DG = sparse(integrated_idx1,integrated_idx2,integrated_W_buf);
integrated_UG = tril(integrated_DG + integrated_DG');
% Find the minimum spanning tree of UG
[integrated_ST,integrated_pred] = graphminspantree(integrated_UG,'Method','Kruskal');
[integrated_ii,integrated_jj,integrated_ss] = find(integrated_ST);

%%
% Drawing the connections
if integration_seg_connect_plot_ON
    if result_save_ON
        writerobj = VideoWriter([result_save_folder,'/ms_integration_connection.avi']);
        open(writerobj);
    end
    h=figure(5);
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
        for m = 1:size(integrated_ii,1)
            plot([seg_center(1,integrated_ii(m),frm_idx),seg_center(1,integrated_jj(m),frm_idx)],height-[seg_center(2,integrated_ii(m),frm_idx),seg_center(2,integrated_jj(m),frm_idx)],'k-');
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