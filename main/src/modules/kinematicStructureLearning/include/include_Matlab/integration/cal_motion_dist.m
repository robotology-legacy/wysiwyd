motion_dist_total_buf = zeros(numNode,numNode,numFrame-1);

for frm_idx = 1:numFrame-1
    motion_dist_buf = zeros(numNode,numNode);
    for i = 1:numNode
        for j = i+1:numNode
            motion_dist_buf(i,j) = (norm([nodeLoc(1:2,i,frm_idx)-nodeLoc(1:2,j,frm_idx)],2)) * ...
                (norm([(nodeLoc(1:2,i,frm_idx+1)-nodeLoc(1:2,i,frm_idx))-(nodeLoc(1:2,j,frm_idx+1)-nodeLoc(1:2,j,frm_idx))],2));
        end
    end
    motion_dist_total_buf(:,:,frm_idx) = motion_dist_buf + motion_dist_buf';
end

motion_dist_total_sorted = sort(motion_dist_total_buf,3,'descend');
motion_dist = median(motion_dist_total_sorted(:,:,1:round(numFrame*0.05)),3)/numFrame;