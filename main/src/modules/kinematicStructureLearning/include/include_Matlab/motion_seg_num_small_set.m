num_small_seg = 0;

for i=1:num_seg
    if size(seg_idx{i},1) < 8
        num_small_seg = num_small_seg + 1;
    end
end

num_small_seg