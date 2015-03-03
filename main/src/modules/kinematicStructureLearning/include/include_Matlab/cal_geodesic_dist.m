function dist = cal_geodesic_dist(map_inv,p1,p2)
p1 = round(p1);
p2 = round(p2);

% T1 = graydist(map_inv,p1(1),p1(2),'quasi-euclidean');
% T2 = graydist(map_inv,p2(1),p2(2),'quasi-euclidean');

T1 = graydist(map_inv,p1(1),p1(2),'cityblock');
T2 = graydist(map_inv,p2(1),p2(2),'cityblock');

T = T1+T2;

% dist = min(min(T));
dist = T(p1(2),p1(1));
end