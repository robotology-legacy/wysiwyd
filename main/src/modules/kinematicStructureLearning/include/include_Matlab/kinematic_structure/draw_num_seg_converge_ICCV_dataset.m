% close all

% filename_buf = {'arm.avi.mat','toy.avi.mat','YanPollefeys.mp4.mat','iCub_motion.avi.mat','iCub_hand.avi.mat','robot_arm.mp4.mat','baxter_cut.mp4.mat','hand1.mp4.mat'};
pathname = '/home/hjchang/Research/code/Matlab/cvpr2015/dataset/iccv_dataset_processed/';
% filename_buf = {'toy_truck/toy_truck.mat'};
filename_buf = {'puppet/puppet.mat'};
% filename_buf = {'head/head.mat'};
% filename_buf = {'dancing/dancing.mat'};
% filename_buf = {'yellow_crane/yellow_crane.mat'};

for video_idx = 1:size(filename_buf,2)
    
    filename = filename_buf{video_idx};
    %     iter_num = 15;
    
    load([pathname,filename]);
%     filename = 'toy_truck.mat';
%     filename = 'head.mat';
    filename = 'puppet.mat';
%     filename = 'dancing.mat';
%     filename = 'yellow_crane.mat';
    
    GT_seg_num = length(unique(s));
    iter_num = 10;
%     switch(video_idx)
%         case{1}
%             GT_seg_num = 2;
%             iter_num = 10;
%         case{2}
%             GT_seg_num = 3;
%             iter_num = 10;
%         case{3}
%             GT_seg_num = 6;
%         case{4}
%             GT_seg_num = 8;
%             iter_num = 20;
%         case{5}
%             GT_seg_num = 7;
%         case{6}
%             GT_seg_num = 8;
%         case{7}
%             GT_seg_num = 11;
%             iter_num = 18;
%         case{8}
%             GT_seg_num = 20;
%             
%     end
    
    iter_mean = zeros(iter_num,1);
    iter_std = zeros(iter_num,1);
    iter_idx = [1:iter_num];
    
    
    for iter = 1:iter_num
        buf = [];
        for i=1:size(num_seg_total,1)
            num_seg_hist_buf = num_seg_total(i,video_idx);
            
            if iter <= size(num_seg_hist_buf{1},1)
                buf = [buf,num_seg_hist_buf{1}(iter)];
            else
                buf = [buf,num_seg_hist_buf{1}(end)];
            end
        end
        iter_mean(iter) = mean(buf);
        iter_std(iter) = std(buf);
    end
    
    
    h=figure(video_idx);
    clf
    hold on;
    set(gcf,'color','w');
    
    % axis([0, iter_num, 0, ceil(max(iter_mean))+1])
    % axis([0 width 0 height])
    H2 = shadedErrorBar(iter_idx, iter_mean, iter_std, '-g', 0);
    H1 = plot(iter_idx, iter_mean, 'Color', 'r', 'LineWidth', 3);
    
    H3 = plot(iter_idx, ones(iter_num,1)*GT_seg_num,'b--','LineWidth', 3);
    
    legend([H1(1), H2.patch, H3(1)], ...
        'Mean', 'Std.', ['Gound Truth: ',num2str(GT_seg_num)], ...
        'Location', 'Northeast');
    
    ylabel('number of segment');
    xlabel('number of iteration');
    axis([0 iter_num 0 12]);
    % [0, iter_num, 0, ceil(max(iter_mean))+1]
    % axis([1 20 1 100]);
%     tightfig
    
%     save_path = '/home/hjchang/Dropbox/paper_writing/[2015][Journal][TPAMI] Kinematic Structure Learning/paper/figs/fig_seg_num_converge/';
%     saveas(h, [save_path, filename(1:end-4),'.pdf'], 'pdf')
end
