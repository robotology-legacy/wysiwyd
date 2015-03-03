load('/home/hjchang/Research/code/Matlab/cvpr2015/dataset/SVDD_data/iCub_motion.avi_SVDDdata.mat')

pathname = '/home/hjchang/Research/code/Matlab/cvpr2015/dataset/';
filename_buf = 'iCub_motion.avi.mat';
load([pathname,filename_buf]);
%%
frm_idx = 1;
height = size(full_radius_map,1);
width = size(full_radius_map,2);

% for i=1:500


p1 = [180,240]';
p2 = [335,150]';
% p2 = [300,320]';

c = full_radius_map(:,:,frm_idx);

c_inv = 1./c;
% c_inv = max(max(c))-c;

T1 = graydist(c_inv,p1(1),p1(2),'quasi-euclidean');
T2 = graydist(c_inv,p2(1),p2(2),'quasi-euclidean');
% T1 = graydist(c_inv,p1(1),p1(2),'chessboard');
% T2 = graydist(c_inv,p2(1),p2(2),'chessboard');
% T1 = graydist(c_inv,p1(1),p1(2),'cityblock');
% T2 = graydist(c_inv,p2(1),p2(2),'cityblock');
T = T1+T2;

figure(2)
clf
mesh(T)
hold on

traj = find(T < min(min(T))+0.00000000000001);
traj_x = floor(traj/height);
traj_y = mod(traj,height);

plot(p1(1),p1(2),'r*');
plot(p1(1),p1(2),'r*');

plot(traj_x,traj_y,'g-')

plot([p1(1),p2(1)],[p1(2),p2(2)],'y-')

T(p1(2),p1(1))
T(p2(2),p2(1))

c = fliplr(c);

%--------------------
figure(112)
set(gcf,'color','w');
clf
c(find(c == 0)) = Inf;
% imagesc(c);
mesh(c)
% colormap jet
colormap summer

colormap(parula(10))
% colormap(map)
% colormap winter

axis image off
% freezeColors
hold on

for i=1:size(y,2)
plot(width-y(1,i,frm_idx),y(2,i,frm_idx),'Marker','.','Color',[0.3,0.3,0.3]);
end

% imshow(full_skeleton_binary_map(:,:,frm_idx));
[sk_idx_h,sk_idx_w] = find(fliplr(full_skeleton_binary_map(:,:,frm_idx)) == 1);
plot(sk_idx_w,sk_idx_h, 'k.');

plot(width-p1(1),p1(2),'Marker','o',...
    'MarkerSize',15,...
    'MarkerEdgeColor','k',...
    'MarkerFaceColor',[1.0,0.2,0.2]);
plot(width-p2(1),p2(2),'Marker','o',...
    'MarkerSize',15,...
    'MarkerEdgeColor','k',...
    'MarkerFaceColor',[1.0,0.2,0.2]);

plot(width-traj_x,traj_y,'w--',...
    'LineWidth',4);

plot([width-p1(1),width-p2(1)],[p1(2),p2(2)],'g-',...
    'LineWidth',4);

view(0,-90);

%%
% Skeleton
figure(9003)
set(gcf,'color','w');
clf
%     subplot(2,2,3)
buf = (full_radius_map(:,:,frm_idx).^(1/2)/max(max(full_radius_map(:,:,frm_idx).^(1/2))));
imagesc(buf);
colormap jet
% axis image off
% freezeColors

hold on
plot(y(1,:,frm_idx),y(2,:,frm_idx),'k.');

[bg_idx_h,bg_idx_w] = find(full_binary_image(:,:,frm_idx) == 0);
plot(bg_idx_w,bg_idx_h, 'w.');

[sk_idx_h,sk_idx_w] = find(full_skeleton_binary_map(:,:,frm_idx) == 1);
plot(sk_idx_w,sk_idx_h, 'w.');


axis([0 width 0 height]);
axis off

