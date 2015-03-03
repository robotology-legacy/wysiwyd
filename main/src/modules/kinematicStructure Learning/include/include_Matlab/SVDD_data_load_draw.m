load(['../dataset/SVDD_data/',filename(1:end-4),'_SVDDdata.mat']);

if shape_SVDD_load_plot_ON
    for frm_idx = 1:num_frames
        figure(31)
        clf
        
        subplot(2,2,1)
        imshow(full_binary_image(:,:,frm_idx));
        
        subplot(2,2,2)
        imagesc(full_skeleton_value_map(:,:,frm_idx));
        colormap jet
        axis image off
        freezeColors
        
        
        subplot(2,2,3)
        imagesc(full_radius_map(:,:,frm_idx))
        colormap jet
        axis image off
        freezeColors
        
        subplot(2,2,4)
        imshow(full_skeleton_binary_map(:,:,frm_idx))
        hold on
        plot(full_exy{frm_idx}(1,:),full_exy{frm_idx}(2,:),'go')
        plot(full_jxy{frm_idx}(1,:),full_jxy{frm_idx}(2,:),'ro')
        
        pause(0.03)
        
    end
end