
full_binary_image = zeros(height, width, num_frames);
full_intensity_map = zeros(height, width, num_frames);
full_skeleton_value_map = zeros(height, width, num_frames);
full_skeleton_binary_map = zeros(height, width, num_frames);
full_radius_map = zeros(height, width, num_frames);

full_exy = cell(num_frames,1);
full_jxy = cell(num_frames,1);

%%
for frm_idx = 1:1:num_frames
%     frm_idx
    
    x_SVDD = x_org_full(:,:,frm_idx);
    y_SVDD = ones(size(x_SVDD,1),1);
    SET = incSVDD(x_SVDD,y_SVDD,C,kernel_type,kernel_param,0);
    
    if SET.S.ndata ~= 0
        
        %%
        % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % image binarisation
        width_buf = repmat([1:width]',[1,height]);
        width_buf2 = reshape(width_buf',[width*height,1]);
        height_buf = [1:height]';
        height_buf2 = repmat(height_buf,[width,1]);
        test_pixel = [width_buf2,height_buf2];
        
        test_result = testSVDD(test_pixel, SET, kernel_param, kernel_type);
        bw_img = uint8(test_result < 0)*255;
        bw_img = reshape(bw_img, [height,width]);
        
        img = flipud(bw_img);
        
        full_binary_image(:,:,frm_idx) = uint8(img);
        
        %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % the standard skeletonization:
        % figure(2)
        %         subplot(2,3,2)
        %         imshow(bwmorph(img,'skel',inf));
        
        % the new method:
        % figure(3)
        %         subplot(2,3,3)
        %         imshow(bwmorph(skeleton(img)>35,'skel',Inf));
        
        % in more detail:
        [skr,rad] = skeleton(img);
        
        % the intensity at each point is proportional to the degree of evidence
        % that this should be a point on the skeleton:
        % figure(4)
        
        full_skeleton_value_map(:,:,frm_idx) = skr;
        
        % skeleton can also return a map of the radius of the largest circle that
        % fits within the foreground at each point:
        % figure(5)
        
        full_intensity_map(:,:,frm_idx) = rad;
        full_radius_map(:,:,frm_idx) = sqrt(rad);
        
        % thresholding the skeleton can return skeletons of thickness 2,
        % so the call to bwmorph completes the thinning to single-pixel width.
        skel = bwmorph(skr > 50,'skel',inf);
        % figure(6)
        
        % try different thresholds besides 35 to see the effects
        
        full_skeleton_binary_map(:,:,frm_idx) = skel;
        
        % anaskel returns the locations of endpoints and junction points
        [dmap,exy,jxy] = anaskel(skel);
        
        
        full_exy{frm_idx} = exy;
        full_jxy{frm_idx} = jxy;
        
        
        %=====================================================================
        % TEMPORARY!!
        % point density 
        data = [y(1,:,frm_idx);y(2,:,frm_idx)]';
        [bandwidth,density,X,Y]=kde2d(data);
        
        [Xq,Yq] = meshgrid(1:1:width, 1:1:height);
        densityq = interp2(X,Y,density,Xq,Yq,'cubic');
        densityq = densityq / sum(sum(densityq));
%         imagesc(densityq)
        %=====================================================================        
        
        %%
        %-----------------------------------------------------------
        % drawing result
        if ctrl_param.KineStruct.shape_SVDD_training_plot_ON
            h = figure(3);
            clf
            incSVDD_drawing(x_SVDD,y_SVDD,SET,kernel_param,kernel_type,time_delay);
            
            subplot(2,3,3)
            imshow(uint8(img));
            title('Shape Binarization');
            
%----------------------------------------------------
% Original
%             subplot(2,3,5)
%             imagesc(skr);
%             colormap jet
%             axis image off
%             freezeColors
%             
%             subplot(2,3,4)
%             imagesc(rad)
%             title('Probability of topological skeleton');
%             colormap jet
%             axis image off
%             freezeColors
%             
%             subplot(2,3,6)
%             imshow(skel)
%             title('Topological Skeleton from Shape');
%             hold on
%             plot(exy(1,:),exy(2,:),'go')
%             plot(jxy(1,:),jxy(2,:),'ro')

%----------------------------------------------------
% density weighted
            subplot(2,3,4)
            imagesc(sqrt(rad))
            title('Skeleton distance map');
            colormap jet
            axis image off
            freezeColors

            subplot(2,3,5)
            imagesc(densityq);
            colormap jet
            axis image off
            freezeColors
            

            combination_buf = sqrt(rad) .* densityq;
            subplot(2,3,6)
            imagesc(combination_buf);
            colormap jet
            axis image off
            freezeColors

%----------------------------------------------------            
            
            
            color_idx = 'rgbcmyk';
            
            if motion_ON
                % drawing connections
                for i=1:num_seg
                    subplot(2,3,1)
                    title('Motion segmentation and skeleton');
                    plot(y(1,seg_idx{i},frm_idx), height-y(2,seg_idx{i},frm_idx),'.','Color',color_idx(mod(i,7)+1));
                    hold on
                    axis([0, width, 0, height]);
                end
                
                for seg_connect_idx_i = 1:num_seg
                    for seg_connect_idx_j = seg_connect_idx_i+1:num_seg
                        plot(seg_center(1,[seg_connect_idx_i,seg_connect_idx_j],frm_idx),...
                            height-seg_center(2,[seg_connect_idx_i,seg_connect_idx_j],frm_idx),'-ko',...
                            'LineWidth',3,...
                            'MarkerSize',10,...
                            'MarkerEdgeColor','k',...
                            'MarkerFaceColor',[1.0,0.2,0.2]);
                    end
                end
            end
            
            pause(0.1)
        end
    end
end
%%
if ctrl_param.KineStruct.motion_ON
    % calculating weights of each connections
    SVDD_skeleton_weight_connection;
end

%%
if ctrl_param.KineStruct.shape_SVDD_result_save_ON
    save(['../dataset/SVDD_data/',filename(1:end-4),'_SVDDdata.mat'],...
        'full_binary_image',...
        'full_radius_map',...
        'full_skeleton_value_map',...
        'full_skeleton_binary_map',...
        'full_intensity_map',...
        'full_exy',...
        'full_jxy',...
        'kernel_param',...
        'kernel_param_buf');
end