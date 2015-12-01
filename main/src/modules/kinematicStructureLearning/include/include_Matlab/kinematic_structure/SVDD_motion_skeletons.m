
for frm = 1:1:num_frames
    frm
    figure(345)
    clf
    %     plot(y(1,:,frm),y(2,:,frm),'b.');

%     imshow(full_binary_image(:,:,frm_idx));
    imshow(1-full_skeleton_map(:,:,frm));
    hold on        
    
    plot(full_exy{frm}(1,:),full_exy{frm}(2,:),'co')
    plot(full_jxy{frm}(1,:),full_jxy{frm}(2,:),'mo')    
    
    plot(y(1,:,frm),y(2,:,frm),'b.');
      
    
    plot(seg_center(1,:,frm),seg_center(2,:,frm),'ro');
    for i = 1:seg_y_num
        text(seg_center(1,i,frm)+3, seg_center(2,i,frm)+3, num2str(i), 'Color', 'r');
        %         text(seg_y(1,i,frm)+3, seg_y(2,i,frm)+3, num2str(i), 'Color', 'r');
    end
    axis([0 width 0 height]);
    %     axis equal
    hold on
    for m = 1:size(ii,1)
        plot([seg_center(1,ii(m),frm),seg_center(1,jj(m),frm)],[seg_center(2,ii(m),frm),seg_center(2,jj(m),frm)],'g-');
        %         plot([seg_y(1,ii(m),frm),seg_y(1,jj(m),frm)],[seg_y(2,ii(m),frm),seg_y(2,jj(m),frm)],'k-');
    end    
    
    
    pause(1);
end