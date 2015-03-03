%%
% Result visualisation
if motion_seg_plot_ON
    color_idx = 'rgbcmyk';
    marker_idx = '+o*.xsd^v><ph';
    
    if result_save_ON
        writerobj = VideoWriter([result_save_folder,'/motion_segmentation_',num2str(num_seg),'.avi']);
        open(writerobj);
    end
    
    h=figure(1);
    title(['Number of Segments: ',num2str(num_seg)]);
    for frm_idx=1:num_frames
        clf
        for i=1:num_seg
            %             plot(y(1,seg_idx{i},f), height-y(2,seg_idx{i},f),'.','Color',color_idx(mod(i,7)+1));
            %             plot(y(1,seg_idx{i},frm_idx), height-y(2,seg_idx{i},frm_idx),'Marker',marker_idx(mod(i,13)+1),'Color',color_idx(mod(i,7)+1));
            plot(y(1,seg_idx{i},frm_idx), height-y(2,seg_idx{i},frm_idx),marker_idx(mod(i,13)+1),'Color',color_idx(mod(i,7)+1));
            %             plot(y(1,seg_idx{i},f), height-y(2,seg_idx{i},f),'.','Color',[1-i/num_seg,i/num_seg,1-i/num_seg]);
            hold on
            axis([0, width, 0, height]);
        end
        %         plot(seg_center(:,1),height-seg_center(:,2),'ko');
        
        pause(0.001);
        %     pause()
        
        if result_save_ON
            F = getframe(h);
            writeVideo(writerobj,F);
        end
    end
    if result_save_ON
        close(writerobj);
    end
end

