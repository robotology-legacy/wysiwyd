seg_center = zeros(2,num_seg,num_frames);

for frm_idx = 1:num_frames
    for i=1:num_seg
        %                 seg_center(1,i,frm_idx) = median(y(1,seg_idx{i},frm_idx));
        %                 seg_center(2,i,frm_idx) = median(y(2,seg_idx{i},frm_idx));
        seg_center(1,i,frm_idx) = mean(y(1,seg_idx{i},frm_idx));
        seg_center(2,i,frm_idx) = mean(y(2,seg_idx{i},frm_idx));
    end
end