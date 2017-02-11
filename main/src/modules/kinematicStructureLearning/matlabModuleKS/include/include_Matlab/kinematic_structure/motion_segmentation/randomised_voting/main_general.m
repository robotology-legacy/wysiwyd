addpath 'f_estimation'
addpath 'fast_Kmeans'
addpath 'spectral_clutering'

%%
% videoFileName = 'toy.avi';

% videoFileName = 'baxter_cut.mp4';
% init_frm = 70;
% end_frm = 514;

videoFileName = 'toy.avi';
init_frm = 1;
end_frm = 69;

% videoFileName = 'YanPollefeys.mp4'; width = 718; height = 480;
% init_frm = 1;
% end_frm = 60;

% videoFileName = 'iCub_hand2.avi'; width = 320; height = 240;
% init_frm = 51;
% end_frm = 330;

% videoFileName = 'iCub_motion.avi'; width = 720; height = 480;
% init_frm = 31;
% end_frm = 280;
tic
% [y, W, frames, points] = mycvuKltRead(['/home/hjchang/OpenCV/test_codes/lk_track/points/',videoFileName,'/point_seq_%d.txt'], init_frm, end_frm);
[y, W, frames, points] = mycvuKltRead(['/home/hjchang/fast_data/research/code/lk_track/points/',videoFileName,'/point_seq_%d.txt'], init_frm, end_frm);
loading_time = toc
%% parameter setting
%         c = max(s);
% c = 11;
% c = 7;
c = floor(size(y,2)/(16));
% c = floor(size(y,2)/(8));
% c = 7;
T_i = 200; % the number of iteration     % 150
T = 10; % the number of trial
T_c = 20; % convergence test, noise free : 15, noise : 5
T_r = 15; % reinitialization at T_r
alpha = 0.9; % decay parameter
lambda = 2; % voting strength

d = 1;
converge_check2 = 0;

max_dist = ones(1,c);
best_overlap = 0;

% y is trajectory, 3 x (the number of points) x (the number of frames)
% T, T_i, T_c, T_r, alpha, lambda : tuning parameters
% c : the number of group
% s : ground truth, (the number of points) x 1

%         final = motion_segmentation(y, T, T_i, T_c, T_r, alpha, lambda, c, s);
final = my_motion_segmentation(y, T, T_i, T_c, T_r, alpha, lambda, c);

%%
% Result visualisation

color_idx = 'rgbcmyk';
num_seg = size(unique(final),1);
seg_idx = cell(num_seg,1);

for i=1:num_seg
    seg_idx{i} = find(final == i);
end

mean_y = zeros(num_seg,2);

h = figure(1);
hold on
for f=1:size(y,3)
    clf
    %     for p=1:size(y,2)
    %         title('Motion segmentation and skeleton');
    %         plot(y(1,p,f), height-y(2,p,f),'.','Color',color_idx(mod(final(p),7)+1));
    %         axis([0, width, 0, height]);
    %         hold on
    %     end
    
    for i=1:num_seg        
        title('Motion segmentation and skeleton');
        plot(y(1,seg_idx{i},f), height-y(2,seg_idx{i},f),'.','Color',color_idx(mod(i,7)+1));
%         plot(y(1,seg_idx{i},f), height-y(2,seg_idx{i},f),'.','Color',[1-i/num_seg,i/num_seg,1-i/num_seg]);
%         [i/num_seg,i/num_seg,i/num_seg]
        hold on
        axis([0, width, 0, height]);
        mean_y(i,1) = median(y(1,seg_idx{i},f));
        mean_y(i,2) = median(y(2,seg_idx{i},f));
        %         mean_y(i,1) = mean(y(1,seg_idx{i},f));
        %         mean_y(i,2) = mean(y(2,seg_idx{i},f));
    end
    plot(mean_y(:,1),height-mean_y(:,2),'ko');
    
    %     F = getframe(h);
    %     writeVideo(writerobj,F);
%     pause(0.001);
    pause()
end

% close(writerobj);
%%

%% connecting means
seg_y = zeros(2,num_seg,size(y,3));

for f = 1:size(y,3)
    for i=1:num_seg
        seg_y(1,i,f) = median(y(1,seg_idx{i},f));
        seg_y(2,i,f) = median(y(2,seg_idx{i},f));
%         seg_y(1,i,f) = mean(y(1,seg_idx{i},f));
%         seg_y(2,i,f) = mean(y(2,seg_idx{i},f));
    end
end

[seg_y_dim,seg_y_num,seg_y_frm] = size(seg_y);

% Choice of neighborhood structure
w1 = 0.9;   % spatial
w2 = 1-w1;  % dynamic

dist_total_buf = zeros(seg_y_num,seg_y_num,seg_y_frm-1);

for f = 1:seg_y_frm-1
    for i = 1:seg_y_num
        for j = 1:seg_y_num
            dist_total_buf(i,j,f) = w1*(norm([seg_y(1:2,i,f)-seg_y(1:2,j,f)],2)) + ...
                w2*(norm([(seg_y(1:2,i,f+1)-seg_y(1:2,i,f))-(seg_y(1:2,j,f+1)-seg_y(1:2,j,f))],2));
        end
    end
end

dist_total_sorted = sort(dist_total_buf,3,'descend');
dist_final = median(dist_total_sorted(:,:,1:round(seg_y_frm*0.05)),3)/seg_y_frm;

idx1 = [];
idx2 = [];
W_buf = [];
k_knn = c-1;
% k_knn = 30;
for i = 1:seg_y_num
    [Y,I] = sort(dist_final(i,:));
    for l = 1:k_knn
        idx1 = [idx1,i];
        idx2 = [idx2,I(l+1)];
        W_buf = [W_buf,dist_final(i,I(l+1))];
    end
end

DG = sparse(idx1,idx2,W_buf);
UG = tril(DG + DG');

% Find the minimum spanning tree of UG
[ST,pred] = graphminspantree(UG);
[ii,jj,ss] = find(ST);

% Drawing the connections
for frm = 1:seg_y_frm;
    figure(2)
    clf
    plot(y(1,:,frm),height-y(2,:,frm),'b.');
%     plot(y(1,:,frm),y(2,:,frm),'b.');    
    hold on
    plot(seg_y(1,:,frm),height-seg_y(2,:,frm),'ro');
%     plot(seg_y(1,:,frm),seg_y(2,:,frm),'ro');
    for i = 1:seg_y_num
        text(seg_y(1,i,frm)+3, height-seg_y(2,i,frm)+3, num2str(i), 'Color', 'r');
%         text(seg_y(1,i,frm)+3, seg_y(2,i,frm)+3, num2str(i), 'Color', 'r');
    end

    axis([0 width 0 height]);
    %     axis equal
    
    hold on
    for m = 1:size(ii,1)
        plot([seg_y(1,ii(m),frm),seg_y(1,jj(m),frm)],height-[seg_y(2,ii(m),frm),seg_y(2,jj(m),frm)],'k-');
%         plot([seg_y(1,ii(m),frm),seg_y(1,jj(m),frm)],[seg_y(2,ii(m),frm),seg_y(2,jj(m),frm)],'k-');
    end
    pause(0.001)
end