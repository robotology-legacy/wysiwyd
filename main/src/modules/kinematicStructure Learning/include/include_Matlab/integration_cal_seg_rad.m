%%
full_seg_center_rad = zeros(num_seg, num_frames);

for i=1:num_seg
    for f=1:num_frames
        %         full_seg_center_rad(i,f) = full_log_weight_map(round(seg_center(2,i,f)), round(seg_center(1,i,f)),f);
        full_seg_center_rad(i,f) = full_radius_map(round(seg_center(2,i,f)), round(seg_center(1,i,f)),f);
        %         full_seg_center_rad(i,f) = full_skeleton_value_map(round(seg_center(2,i,f)), round(seg_center(1,i,f)),f);
    end
end

seg_center_rad = round(sqrt(mean(full_seg_center_rad,2)))';
% seg_center_rad = round(100./sqrt(mean(full_seg_center_rad,2)))