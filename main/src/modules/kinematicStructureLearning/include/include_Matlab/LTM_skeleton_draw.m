[LTM_ii, LTM_jj, LTM_ss] = find(adjmatT{1});
% Drawing the connections of ChowLiu
for frm = 1:seg_y_frm;
    figure(1001)
    clf
    plot(y(1,:,frm),height-y(2,:,frm),'b.');
%     plot(y(1,:,frm),y(2,:,frm),'b.');    
    hold on
    plot(seg_y(1,:,frm),height-seg_y(2,:,frm),'ro');
%     plot(seg_y(1,:,frm),seg_y(2,:,frm),'ro');
    for i = 1:seg_y_num
        text(seg_y(1,i,frm)+3, height-seg_y(2,i,frm)+3, num2str(i), 'Color', 'r');
%         text(seg_y(1,i,frm)+3, seg_y(2,i,frm)+3, num2str(i), 'Color', 'r');
    end

    axis([0 width 0 height]);
    %     axis equal
    
    hold on
    for m = 1:size(LTM_ii,1)
        plot([seg_y(1,LTM_ii(m),frm),seg_y(1,LTM_jj(m),frm)],height-[seg_y(2,LTM_ii(m),frm),seg_y(2,LTM_jj(m),frm)],'k-');
%         plot([seg_y(1,ii(m),frm),seg_y(1,jj(m),frm)],[seg_y(2,ii(m),frm),seg_y(2,jj(m),frm)],'k-');
    end
    pause(0.001)
end