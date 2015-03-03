function [seg_idx_GT, seg_center_GT] = cal_seg_GT(s,y)

num_frames = size(y,3);
num_seg_GT = size(unique(s),1);

seg_idx_GT = cell(num_seg_GT,1);

for n = 1:num_seg_GT
    seg_idx_GT{n} = find(s == n);
end

seg_center_GT = zeros(2,num_seg_GT,num_frames);

for frm_idx = 1:num_frames
    for i=1:num_seg_GT
        %                 seg_center(1,i,frm_idx) = median(y(1,seg_idx{i},frm_idx));
        %                 seg_center(2,i,frm_idx) = median(y(2,seg_idx{i},frm_idx));
        seg_center_GT(1,i,frm_idx) = mean(y(1,seg_idx_GT{i},frm_idx));
        seg_center_GT(2,i,frm_idx) = mean(y(2,seg_idx_GT{i},frm_idx));
    end
end

end