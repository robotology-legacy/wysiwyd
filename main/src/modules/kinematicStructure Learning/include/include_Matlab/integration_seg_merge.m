%% iterative clustering

closet_merging = 1;
radius_merging = 1;

[seg_center_rad_sort_value, seg_center_rad_sort_idx] = sort(seg_center_rad,'ascend');

thres_rad = min_rad;
seg_center_rad_sort_value;
%%
% seg center distance
if closet_merging == 1;
    seg_center_mtx_total = zeros(num_seg, num_seg, num_frames);
    for frm_idx = 1:num_frames
        for i = 1:num_seg
            for j = i+1:num_seg
                seg_center_mtx_total(i,j,frm_idx) = sqrt((seg_center(1,i,frm_idx)-seg_center(1,j,frm_idx))^2 ...
                    + (seg_center(2,i,frm_idx)-seg_center(2,j,frm_idx))^2);
            end
        end
    end
    seg_center_mtx = mean(seg_center_mtx_total,3);
    
    closest_seg_idx_i = 0;
    closest_seg_idx_j = 0;
    
    for i = 1:num_seg
        for j = i+1:num_seg
            if seg_center_mtx(i,j) < thres_rad
                closest_seg_idx_i = i;
                closest_seg_idx_j = j;
            end
        end
    end
    
    if closest_seg_idx_i ~= 0 && closest_seg_idx_j ~= 0
        seg_idx{closest_seg_idx_i} = [seg_idx{closest_seg_idx_i};seg_idx{closest_seg_idx_j}];
        seg_idx{closest_seg_idx_j} = [];
        emptyCells = cellfun('isempty', seg_idx);
        seg_idx(emptyCells) = [];
        
        disp(['........ Merging Seg #',num2str(closest_seg_idx_i), ' and Seg #',num2str(closest_seg_idx_j), ' close seg']);
        
        num_seg = num_seg - 1;
        radius_merging = 0;
    else
        radius_merging = 1;
    end
end

%%
% by radius

if radius_merging == 1
    if seg_center_rad_sort_value(1) < thres_rad
        
        min_rad_seg_idx = seg_center_rad_sort_idx(1);
        
        MS_connection_mtx = full(motion_MST_ST);
        MS_connection_mtx = MS_connection_mtx + MS_connection_mtx';
        
        
        min_value_buf = min(nonzeros(MS_connection_mtx(min_rad_seg_idx,:)));
        if isempty(min_value_buf)
            disp('........ Stop merging!');
            do_seg_merge = 0;
        else
            min_connected_seg_idx = find(MS_connection_mtx(min_rad_seg_idx,:) == min_value_buf);
            
            seg_idx{min_rad_seg_idx} = [seg_idx{min_rad_seg_idx};seg_idx{min_connected_seg_idx}];
            seg_idx{min_connected_seg_idx} = [];
            emptyCells = cellfun('isempty', seg_idx);
            seg_idx(emptyCells) = [];
            
            disp(['........ Merging Seg #',num2str(min_rad_seg_idx), ' and Seg #',num2str(min_connected_seg_idx), ' deviated seg']);
            
            num_seg = num_seg - 1;
        end
    else
        disp('........ Stop merging!');
        do_seg_merge = 0;
    end
end