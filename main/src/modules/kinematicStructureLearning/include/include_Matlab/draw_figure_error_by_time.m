% mean_error_by_time = mean(error_by_time_buf(2:9,:),1);   
% std_error_by_time = std(error_by_time_buf(2:9,:),1);
mean_error_by_time = mean(error_by_time_buf,1);   
std_error_by_time = std(error_by_time_buf,1);

h=figure(112);
    clf
    hold on;
    set(gcf,'color','w');
    
    % axis([0, iter_num, 0, ceil(max(iter_mean))+1])
    % axis([0 width 0 height])
    H2 = shadedErrorBar([1:num_frames], mean_error_by_time, std_error_by_time, '-g', 0);
    H1 = plot([1:num_frames], mean_error_by_time, 'Color', 'g', 'LineWidth', 3);
    H3 = plot([1:num_frames], error_by_time, 'b--', 'LineWidth', 3);
    
%     H3 = plot(iter_idx, ones(iter_num,1)*GT_seg_num,'b--','LineWidth', 3);
    
    legend([H1(1), H2.patch, H3(1)], ...
        'Mean', 'Std.', 'Fayad et.al.', ...
        'Location', 'Northeast');
    
    xlabel('frame');
    ylabel('error');
    