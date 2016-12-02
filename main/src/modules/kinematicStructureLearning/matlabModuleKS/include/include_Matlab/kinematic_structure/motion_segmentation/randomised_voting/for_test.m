
ht_ave = mean(ht,3);
% ht_ave = ht(:,:,1);

[Y,final] = max(ht_ave,[],2);
[Y2, sort_idx] = sort(ht_ave,2);

first_idx = sort_idx(:,c);
close_idx = sort_idx(:,c-1);

final = close_idx;

%%
closest_seg_idx_h = cell(num_seg,1);

for i=1:num_seg
    closest_seg_idx_h{i} = close_idx(find(final == i));
end

%%
% Result visualisation
%
width = 718; height = 480;
figure_num = 1;
color_idx = 'rgbcmykrbgcmykrgbcmyk';
num_seg = size(unique(final),1);
seg_idx = cell(num_seg,1);

for i=1:num_seg
    seg_idx{i} = find(final == i);
end

mean_y = zeros(num_seg,2);

h = figure(figure_num);
for f=1:1
    clf  
    for p=1:size(y,2)
        title('Motion segmentation and skeleton');
        plot(y(1,p,f), height-y(2,p,f),'.','Color',color_idx(final(p)));
        text(y(1,p,f),height-y(2,p,f),['[',num2str(first_idx(p)),', ',num2str(close_idx(p)),']'],...
                                       'VerticalAlignment','middle',...
                                       'HorizontalAlignment','left',...
                                       'FontSize',6);        
        axis([0, width, 0, height]);
        hold on
    end
    
    for i=1:num_seg
%         mean_y(i,1) = median(y(1,seg_idx{i},f));
%         mean_y(i,2) = median(y(2,seg_idx{i},f));
        mean_y(i,1) = mean(y(1,seg_idx{i},f));
        mean_y(i,2) = mean(y(2,seg_idx{i},f));
    end
    plot(mean_y(:,1),height-mean_y(:,2),'ko');

%     plot([mean_y(1,1),mean_y(11,1)], [mean_y(1,2),mean_y(11,2)],'k-');
%     plot([mean_y(11,1),mean_y(3,1)], [mean_y(11,2),mean_y(3,2)],'k-');
%     
%     plot([mean_y(11,1),mean_y(4,1)], [mean_y(11,2),mean_y(4,2)],'k-');
%     plot([mean_y(11,1),mean_y(5,1)], [mean_y(11,2),mean_y(5,2)],'k-');
%     
%     plot([mean_y(3,1),mean_y(10,1)], [mean_y(3,2),mean_y(10,2)],'k-');    
%     plot([mean_y(3,1),mean_y(8,1)], [mean_y(3,2),mean_y(8,2)],'k-');
%     
%     plot([mean_y(4,1),mean_y(2,1)], [mean_y(4,2),mean_y(2,2)],'k-');    
%     plot([mean_y(5,1),mean_y(9,1)], [mean_y(5,2),mean_y(9,2)],'k-');
%     plot([mean_y(10,1),mean_y(7,1)], [mean_y(10,2),mean_y(7,2)],'k-');
%     plot([mean_y(8,1),mean_y(6,1)], [mean_y(8,2),mean_y(6,2)],'k-');
% 
%     
%     F = getframe(h);
%     writeVideo(writerobj,F);    

    pause(0.001);
end