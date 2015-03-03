%%
% Drawing the connections with video
% load init image
if motion_seg_connect_plot_with_video_ON
    if result_save_ON
        writerobj = VideoWriter([result_save_folder,'/ms_connection_image_',num2str(num_seg),'.avi']);
        open(writerobj);
    end
    
    h=figure(10000+num_seg);    
    
    videoFileName = filename(1:end-4);
    xyloObj = VideoReader(['/home/hjchang/OpenCV/test_codes/lk_track/',videoFileName]);
    
    nFrames = xyloObj.NumberOfFrames;
    for frm = 1:num_frames;
        curFrame = read(xyloObj,frm+init_frm-1);
        
        clf
        imshow(curFrame);
        hold on
        %     axis([0 width 0 height]);
        
        % feature points
        for i=1:num_seg
            %         plot(y(1,seg_idx{i},frm), height-y(2,seg_idx{i},frm),'.','Color',color_idx(mod(i,7)+1));
            plot(y(1,seg_idx{i},frm), y(2,seg_idx{i},frm),'y.');
            hold on
        end
        
        
        % drawing connections
        for m = 1:size(motion_MST_ii,1)
            plot([seg_center(1,motion_MST_ii(m),frm),seg_center(1,motion_MST_jj(m),frm)],[seg_center(2,motion_MST_ii(m),frm),seg_center(2,motion_MST_jj(m),frm)],'-Wo',...
                'LineWidth',3,...
                'MarkerSize',10,...
                'MarkerEdgeColor','w',...
                'MarkerFaceColor',[1.0,0.2,0.2]);
        end
        
        % put center number
        for i = 1:num_seg
            text(seg_center(1,i,frm)+8, seg_center(2,i,frm)+8, num2str(i), 'Color', 'g');
        end
        
        pause(0.5)
        if result_save_ON
            F = getframe(h);
            writeVideo(writerobj,F);
        end        
    end
    if result_save_ON
        close(writerobj);
    end    
end