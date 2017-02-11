load('/home/hjchang/Research/code/Matlab/cvpr2015/dataset/SVDD_data/iCub_motion.avi_SVDDdata.mat')

height = size(full_radius_map,1);
width = size(full_radius_map,2);

% for i=1:500


p1 = [150,250]';
% p2 = [330,150]
p2 = [300,320]';

c = full_radius_map(:,:,1);

c_inv = 1./c;
% c_inv = max(max(c))-c;

% T1 = graydist(c_inv,p1(1),p1(2),'quasi-euclidean');
% T2 = graydist(c_inv,p2(1),p2(2),'quasi-euclidean');
% T1 = graydist(c_inv,p1(1),p1(2),'chessboard');
% T2 = graydist(c_inv,p2(1),p2(2),'chessboard');
T1 = graydist(c_inv,p1(1),p1(2),'cityblock');
T2 = graydist(c_inv,p2(1),p2(2),'cityblock');
T = T1+T2;

figure(2)
% clf
mesh(T)
hold on

traj = find(T < min(min(T))+0.00000000001);
traj_x = floor(traj/height);
traj_y = mod(traj,height);

plot(traj_x,traj_y,'g-')

T(p1(2),p1(1))
T(p2(2),p2(1))

% end
