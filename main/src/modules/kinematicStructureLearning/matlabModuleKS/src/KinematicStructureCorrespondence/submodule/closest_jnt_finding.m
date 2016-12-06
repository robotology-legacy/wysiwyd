function matching_out = closest_jnt_finding(dist_map)

[num_jntProj, num_jntSeg] = size(dist_map);

matching_out = zeros(num_jntProj,num_jntSeg);

num_found = 0;

while(1)
    [min_val,idx] = min(dist_map(:));
    [row,col] = ind2sub(size(dist_map),idx);
    matching_out(row,col) = 1;
    dist_map(row,:) = inf;
    dist_map(:,col) = inf;
    num_found = num_found + 1;
    if num_found == num_jntSeg
        break
    end
end

end