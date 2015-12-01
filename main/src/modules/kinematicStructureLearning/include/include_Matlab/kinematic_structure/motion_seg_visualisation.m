%%
% Result visualisation
if ctrl_param.KineStruct.motion_seg_plot_ON
    color_idx = 'rgbcmyk';
    marker_idx = '+o*.xsd^v><ph';
    
    h=figure;
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
        pause(0.001);
    end
end

