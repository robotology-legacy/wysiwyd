addpath 'f_estimation'
addpath 'fast_Kmeans'
addpath 'spectral_clutering'

%% parameter setting
%         c = max(s);
c = 11;
% c = 6;
T_i = 200; % the number of iteration     % 150
T = 100; % the number of trial
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
%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Construct a multimedia reader object associated with file 'xylophone.mpg' with
% user tag set to 'myreader1'.
% writerobj = VideoWriter('result_video.avi');
% open(writerobj);
% % writerobj.FrameRate = 30;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
addpath 'f_estimation'
addpath 'fast_Kmeans'
addpath 'spectral_clutering'
addpath /home/hjchang/Research/dataset/motion_segmentation/Hopkins155WithVideosPart4/giraffe
conversion_to_y;
width = 180; height = 120;

color_idx = 'rgbcmykrbgcmykrgbcmyk';
num_seg = size(unique(final),1);
seg_idx = cell(num_seg,1);

for i=1:num_seg
    seg_idx{i} = find(final == i);
end

mean_y = zeros(num_seg,2);

h = figure(1);
for f=1:size(y,3)
    for p=1:size(y,2)
        subplot(1,2,1)
        title('Feature points of Giraffe');
        plot(y(1,p,f), y(2,p,f),'k.');
        axis([0, 100, 0, 60]);
        hold on
    end
    
    for p=1:size(y,2)
        subplot(1,2,2)
        title('Motion segmentation and skeleton');
        plot(y(1,p,f), y(2,p,f),'.','Color',color_idx(final(p)));
        axis([0, 100, 0, 60]);
        hold on
    end
    
    for i=1:num_seg
%         mean_y(i,1) = median(y(1,seg_idx{i},f));
%         mean_y(i,2) = median(y(2,seg_idx{i},f));
        mean_y(i,1) = mean(y(1,seg_idx{i},f));
        mean_y(i,2) = mean(y(2,seg_idx{i},f));
    end
    subplot(1,2,2)
    plot(mean_y(:,1),mean_y(:,2),'ko', 'MarkerSize',7);

    plot([mean_y(1,1),mean_y(11,1)], [mean_y(1,2),mean_y(11,2)],'k-');
    plot([mean_y(11,1),mean_y(3,1)], [mean_y(11,2),mean_y(3,2)],'k-');
%     
    plot([mean_y(3,1),mean_y(4,1)], [mean_y(3,2),mean_y(4,2)],'k-');
    plot([mean_y(3,1),mean_y(10,1)], [mean_y(3,2),mean_y(10,2)],'k-');    
    
    plot([mean_y(11,1),mean_y(7,1)], [mean_y(11,2),mean_y(7,2)],'k-');     
    plot([mean_y(11,1),mean_y(9,1)], [mean_y(11,2),mean_y(9,2)],'k-');
%     
    plot([mean_y(4,1),mean_y(6,1)], [mean_y(4,2),mean_y(6,2)],'k-');    
    plot([mean_y(9,1),mean_y(8,1)], [mean_y(9,2),mean_y(8,2)],'k-');
    plot([mean_y(10,1),mean_y(5,1)], [mean_y(10,2),mean_y(5,2)],'k-');
    plot([mean_y(7,1),mean_y(2,1)], [mean_y(7,2),mean_y(2,2)],'k-');
% 
%     
%     F = getframe(h);
%     writeVideo(writerobj,F);    
    pause(0.01);
    clf
end

% close(writerobj);
%%

