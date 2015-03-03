frm_idx = 500;

% for frm_idx = 1:num_frames
    frm_idx
    % feature points
    figure(9001)
    clf
%     subplot(2,2,1)
    set(gcf,'color','w');
    plot(y(1,:,frm_idx),height-y(2,:,frm_idx),'k.');
    axis([0 width 0 height]);
    axis off
    
    %%
    % Motion segments
    seg_idx_for_figure1_ms = seg_idx_history{2};
    figure(9002)
    clf
%     subplot(2,2,2)
    color_idx = 'rgbcmyk';
    marker_idx = '+o*.xsd^v><ph';
    
    set(gcf,'color','w');
    for i=1:size(seg_idx_for_figure1_ms,1)
        plot(y(1,seg_idx_for_figure1_ms{i},frm_idx), height-y(2,seg_idx_for_figure1_ms{i},frm_idx),marker_idx(mod(i,13)+1),'Color',color_idx(mod(i,7)+1));
        hold on
    end
    axis([0 width 0 height]);
    axis off
    
    %%
    % Skeleton
    figure(9003)
    set(gcf,'color','w');
    clf
%     subplot(2,2,3)
    buf = (full_radius_map(:,:,frm_idx).^(1/2)/max(max(full_radius_map(:,:,frm_idx).^(1/2))));
    imagesc(buf);
    colormap jet
    % axis image off
    % freezeColors
    
    hold on
    plot(y(1,:,frm_idx),y(2,:,frm_idx),'k.');
    
    [bg_idx_h,bg_idx_w] = find(full_binary_image(:,:,frm_idx) == 0);
    plot(bg_idx_w,bg_idx_h, 'w.');
    
    [sk_idx_h,sk_idx_w] = find(full_skeleton_binary_map(:,:,frm_idx) == 1);
    plot(sk_idx_w,sk_idx_h, 'w.');
    
    
    axis([0 width 0 height]);
    axis off
    
    %%
    % Final result
    figure(9004)
    set(gcf,'color','w');
    clf
%     subplot(2,2,4)
    seg_idx_for_figure1_final = seg_idx_history{5};
    videoFileName = filename(1:end-4);
    xyloObj = VideoReader(['/home/hjchang/OpenCV/test_codes/lk_track/',videoFileName]);
    
    curFrame = read(xyloObj,frm_idx+init_frm-1);
    
    imshow(curFrame);
    hold on
    
    
    % feature points
    for i=1:size(seg_idx_for_figure1_final,1)
        plot(y(1,seg_idx_for_figure1_final{i},frm_idx), y(2,seg_idx_for_figure1_final{i},frm_idx),'y.');
        hold on
    end
    
    
    [bg_idx_h,bg_idx_w] = find(curFrame(:,:,1) <= 30 & curFrame(:,:,2) <= 30 & curFrame(:,:,3) <= 30);
    plot(bg_idx_w,bg_idx_h, 'w.');        
    % drawing connections
    for m = 1:size(motion_MST_ii,1)
        plot([seg_center(1,motion_MST_ii(m),frm_idx),seg_center(1,motion_MST_jj(m),frm_idx)],[seg_center(2,motion_MST_ii(m),frm_idx),seg_center(2,motion_MST_jj(m),frm_idx)],'-ko',...
            'LineWidth',4,...
            'MarkerSize',12,...
            'MarkerEdgeColor','k',...
            'MarkerFaceColor',[1.0,0.2,0.2]);
    end

    
    
    
    axis([0 width 0 height]);
%     pause(1)