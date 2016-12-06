%%
% draw connection with radius
if integration_seg_connect_plot_ON
    if result_save_ON
        writerobj = VideoWriter([result_save_folder,'/ms_integration_radius.avi']);
        open(writerobj);
    end
    
    h = figure(3123);
    
    for frm_idx = 1:num_frames
        
        clf
        %     imshow(flipud(full_skeleton_binary_map(:,:,frm_idx)));
        imshow(full_skeleton_binary_map(:,:,frm_idx));
        hold on
        %     plot(full_exy{frm_idx}(1,:),full_exy{frm_idx}(2,:),'go')
        plot(full_jxy{frm_idx}(1,:),full_jxy{frm_idx}(2,:),'ro')
        
        plot(y(1,:,frm_idx),y(2,:,frm_idx),'b.');
        
        hold on
        plot(seg_center(1,:,frm_idx),seg_center(2,:,frm_idx),'ro');
        
        for i = 1:num_seg
            text(seg_center(1,i,frm_idx)+3, seg_center(2,i,frm_idx)+3, num2str(i), 'Color', 'r');
        end
        axis([0 width 0 height]);
        hold on
        
        for m = 1:size(motion_MST_ii,1)
            plot([seg_center(1,motion_MST_ii(m),frm_idx),seg_center(1,motion_MST_jj(m),frm_idx)],[seg_center(2,motion_MST_ii(m),frm_idx),seg_center(2,motion_MST_jj(m),frm_idx)],'y-', 'LineWidth',3);
        end
        
        for i = 1:num_seg
            circle(seg_center(1,i,frm_idx),seg_center(2,i,frm_idx),seg_center_rad(i),'c');
        end
        
        for i = 1:size(full_exy{frm_idx},2)
            circle(full_exy{frm_idx}(1,i),full_exy{frm_idx}(2,i),round(sqrt(full_radius_map(full_exy{frm_idx}(2,i),full_exy{frm_idx}(1,i),frm_idx))),'g');
        end
        
        pause(0.3)
        if result_save_ON
            F = getframe(h);
            writeVideo(writerobj,F);
        end
    end
    if result_save_ON
        close(writerobj);
    end
end