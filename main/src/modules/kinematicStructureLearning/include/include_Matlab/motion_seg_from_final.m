seg_idx = cell(num_seg,1);

for i=1:num_seg
    seg_idx{i} = find(final == i);
end