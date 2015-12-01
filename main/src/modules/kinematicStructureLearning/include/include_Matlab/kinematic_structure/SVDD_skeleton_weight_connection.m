
full_connection_weight = zeros(num_seg, num_seg, num_frames);

for frm_idx = 1:num_frames
    weight_map = full_radius_map(:,:,frm_idx);
    for seg_connect_idx_i = 1:num_seg
        for seg_connect_idx_j = seg_connect_idx_i+1:num_seg
            
            %check the bresenham path is lower than max depth
            x1 = round(seg_center(1,seg_connect_idx_i,frm_idx));
            y1 = round(seg_center(2,seg_connect_idx_i,frm_idx));
            x2 = round(seg_center(1,seg_connect_idx_j,frm_idx));
            y2 = round(seg_center(2,seg_connect_idx_j,frm_idx));
            
            %%
            %             [x_pix, y_pix] = bresenham(x1,y1,x2,y2);
            [y_pix, x_pix] = bresenham(y1,x1,y2,x2);
            size_x = size(x_pix,1);
            
            %%
            out_of_bounds = 0;
            for k = 1:size_x
                if isnan(y_pix(k)) || weight_map(y_pix(k),x_pix(k)) == 0
                    out_of_bounds = 1;
                end
            end
            
            %             if out_of_bounds > 0
            %                 full_connection_weight(seg_connect_idx_i, seg_connect_idx_j, frm_idx) = 0;
            %             else
            weight_buf = diag(weight_map(y_pix,x_pix));
            
            %             full_connection_weight(seg_connect_idx_i, seg_connect_idx_j, frm_idx) = mean(weight_buf)/size_x;
%                         full_connection_weight(seg_connect_idx_i, seg_connect_idx_j, frm_idx) = mean(weight_buf);
            full_connection_weight(seg_connect_idx_i, seg_connect_idx_j, frm_idx) = mean(log(weight_buf.^2+1))/size_x;
%             full_connection_weight(seg_connect_idx_i, seg_connect_idx_j, frm_idx) = mean(weight_buf)/size_x;
            %                 full_connection_weight(seg_connect_idx_i, seg_connect_idx_j, frm_idx) = max(weight_buf)/size_x;
            %             end
        end
    end
end