function [error_by_time] = cal_error_by_time(y, s, seg_idx, seg_center, seg_idx_GT, seg_center_GT)

num_seg = size(seg_idx,1);
num_seg_GT = size(seg_idx_GT,1);
num_points = size(y,2);
num_frames = size(y,3);

% error weight by seg number
weight = (1 + (abs(num_seg_GT-num_seg)/num_seg_GT));
error_total = 0;

error_by_time = zeros(1,num_frames);

for frm_idx = 1:num_frames
    error_total = 0;
    for n = 1:num_seg_GT
        seg_center_GT_buf = repmat(seg_center_GT(:,n,frm_idx),[1,num_seg]);
        dist_vec = sum((seg_center(:,:,frm_idx) - seg_center_GT_buf).^2,1).^(1/2);
        min_dist = min(dist_vec);
        error_total = error_total + min_dist;
    end
    error_by_time(frm_idx) = error_total;
end

% error = error_total/(num_seg_GT*num_frames) * weight;

error_by_time = error_by_time /(num_seg_GT) * weight;

end