function dist_buf = cal_geodesic_dist_efficient(map_inv,seg_center_current, num_seg)

dist_buf = zeros(num_seg, num_seg);

p = zeros(num_seg,2);
T = cell(num_seg,1);

for i=1:num_seg
    p(i,:) = round(seg_center_current(:,i)');
    T{i} = graydist(map_inv,p(i,1),p(i,2),'cityblock');
%     T{i} = graydist(map_inv,p(i,1),p(i,2),'quasi-euclidean');
end

% T1 = graydist(map_inv,p1(1),p1(2),'quasi-euclidean');
% T2 = graydist(map_inv,p2(1),p2(2),'quasi-euclidean');
% T1 = graydist(map_inv,p1(1),p1(2),'cityblock');
% T2 = graydist(map_inv,p2(1),p2(2),'cityblock');
% 
% T = T1+T2;


for i=1:num_seg
    for j=i+1:num_seg
        T_buf = T{i}+T{j};
        dist_buf(i,j) = T_buf(p(i,2),p(i,1));
    end    
end

% dist = min(min(T));
% dist = T(p1(2),p1(1));
end