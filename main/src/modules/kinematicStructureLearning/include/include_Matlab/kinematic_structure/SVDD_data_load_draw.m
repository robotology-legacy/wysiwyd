stridx = strfind(filepath,'/');
filename = filepath(stridx(end-1)+1:stridx(end)-1);

load(['../dataset/SVDD_data/',filename,'_SVDDdata.mat']);

if ctrl_param.KineStruct.shape_SVDD_load_plot_ON
    for frm_idx = 1:num_frames
        
%         %=====================================================================
%         % TEMPORARY!!
%         % point density
%         data = [y(1,:,frm_idx);y(2,:,frm_idx)]';
%         [bandwidth,density,X,Y]=kde2d(data);
%         figure(33);
%         surf(X,Y,density,'LineStyle','none'), view([0,70])
%         
%         [Xq,Yq] = meshgrid(1:1:width, 1:1:height);
%         densityq = interp2(X,Y,density,Xq,Yq,'cubic');
%         densityq = densityq / sum(sum(densityq));
%         imagesc(densityq)
%         %=====================================================================
        
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
        
        
%         %=====================================================================
%         combination_buf = full_radius_map(:,:,frm_idx) .* densityq;
%         %         combination_buf = full_radius_map(:,:,frm_idx) + densityq;
%         figure(34)
%         imagesc(combination_buf);
        
    end
end