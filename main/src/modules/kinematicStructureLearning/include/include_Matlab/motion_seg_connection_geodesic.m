%% connecting means
% Choice of neighborhood structure
% w1 = 0.5;   % spatial
% w2 = 1-w1;  % dynamic

motion_dist_total_buf = zeros(num_seg,num_seg,num_frames-1);

% tic
% parfor frm_idx = 1:num_frames-1
for frm_idx = 1:num_frames-1
    %     frm_idx
    map_inv = 1./sqrt(full_radius_map(:,:,frm_idx));
    
    %     motion_dist_buf = zeros(num_seg,num_seg);
    %     for i = 1:num_seg
    %         for j = i+1:num_seg
    %             %             dist_total_buf(i,j,f) = w1*(norm([seg_center(1:2,i,f)-seg_center(1:2,j,f)],2)) + ...
    %             %                 w2*(norm([(seg_center(1:2,i,f+1)-seg_center(1:2,i,f))-(seg_center(1:2,j,f+1)-seg_center(1:2,j,f))],2));
    %
    % %             motion_dist_total_buf(i,j,frm_idx) = (norm([seg_center(1:2,i,frm_idx)-seg_center(1:2,j,frm_idx)],2)) * ...
    % %                 (norm([(seg_center(1:2,i,frm_idx+1)-seg_center(1:2,i,frm_idx))-(seg_center(1:2,j,frm_idx+1)-seg_center(1:2,j,frm_idx))],2));
    %
    %             motion_dist_buf(i,j) = cal_geodesic_dist(map_inv,seg_center(1:2,i,frm_idx),seg_center(1:2,j,frm_idx)) * ...
    %                 (norm([(seg_center(1:2,i,frm_idx+1)-seg_center(1:2,i,frm_idx))-(seg_center(1:2,j,frm_idx+1)-seg_center(1:2,j,frm_idx))],2));
    %         end
    %     end
    motion_dist_geodesic_buf = cal_geodesic_dist_efficient(map_inv,seg_center(1:2,:,frm_idx), num_seg);
    motion_dist_motion_buf = zeros(num_seg, num_seg);
    for i = 1:num_seg
        for j = i+1:num_seg
            motion_dist_motion_buf(i,j) = (norm([(seg_center(1:2,i,frm_idx+1)-seg_center(1:2,i,frm_idx))-(seg_center(1:2,j,frm_idx+1)-seg_center(1:2,j,frm_idx))],2));
        end
    end
    motion_dist_buf = motion_dist_geodesic_buf .* motion_dist_motion_buf;
    motion_dist_total_buf(:,:,frm_idx) = motion_dist_buf + motion_dist_buf';
end
% time = toc

% motion_dist_total_sorted = sort(motion_dist_total_buf,3,'descend');
% motion_dist = median(motion_dist_total_sorted(:,:,1:round(num_frames*0.05)),3)/num_frames;
motion_dist = median(motion_dist_total_buf,3);

%%
% Do MST
motion_MST_idx1 = [];
motion_MST_idx2 = [];
motion_MST_W_buf = [];
motion_MST_k_knn = num_seg-1;

for i = 1:num_seg
    [motion_MST_Y,motion_MST_I] = sort(motion_dist(i,:));
    for l = 1:motion_MST_k_knn
        motion_MST_idx1 = [motion_MST_idx1,i];
        motion_MST_idx2 = [motion_MST_idx2,motion_MST_I(l+1)];
        motion_MST_W_buf = [motion_MST_W_buf,motion_dist(i,motion_MST_I(l+1))];
    end
end

motion_MST_DG = sparse(motion_MST_idx1,motion_MST_idx2,motion_MST_W_buf);
motion_MST_UG = tril(motion_MST_DG + motion_MST_DG');

% Find the minimum spanning tree of UG
[motion_MST_ST,motion_MST_pred] = graphminspantree(motion_MST_UG,'Method','Kruskal');
[motion_MST_ii,motion_MST_jj,motion_MST_ss] = find(motion_MST_ST);

%%
% Drawing the connections
if motion_seg_connect_plot_ON
    if result_save_ON
        writerobj = VideoWriter([result_save_folder,'/ms_connection_',num2str(num_seg),'.avi']);
        open(writerobj);
    end
    
    h=figure(2000+num_seg);
    
    for frm = 1:num_frames;
        clf
        plot(y(1,:,frm),height-y(2,:,frm),'b.');
        %     plot(y(1,:,frm),y(2,:,frm),'b.');
        hold on
        plot(seg_center(1,:,frm),height-seg_center(2,:,frm),'ro');
        %     plot(seg_y(1,:,frm),seg_y(2,:,frm),'ro');
        for i = 1:num_seg
            text(seg_center(1,i,frm)+3, height-seg_center(2,i,frm)+3, num2str(i), 'Color', 'r');
        end
        axis([0 width 0 height]);
        hold on
        for m = 1:size(motion_MST_ii,1)
            plot([seg_center(1,motion_MST_ii(m),frm),seg_center(1,motion_MST_jj(m),frm)],height-[seg_center(2,motion_MST_ii(m),frm),seg_center(2,motion_MST_jj(m),frm)],'k-');
            %         plot([seg_y(1,ii(m),frm),seg_y(1,jj(m),frm)],[seg_y(2,ii(m),frm),seg_y(2,jj(m),frm)],'k-');
        end
        pause(0.001)
        if result_save_ON
            F = getframe(h);
            writeVideo(writerobj,F);
        end
    end
    if result_save_ON
        close(writerobj);
    end
end

%%
% Drawing the connections with color segments
color_idx = 'rgbcmky';
marker_idx = '+o*xsd^v><ph';

if motion_seg_connect_color_plot_ON
    h=figure(5000+num_seg);
    
    for frm_idx = 1:1;
        set(gcf,'color','w');
        clf
        for i=1:num_seg
            %     plot(y(1,seg_idx{i},frm_idx), y(2,seg_idx{i},frm_idx),marker_idx(mod(i,13)+1),'Color',color_idx(mod(i,7)+1));
%             plot(y(1,seg_idx{i},frm_idx), height-y(2,seg_idx{i},frm_idx),marker_idx(mod(i,12)+1),'Color',color_idx(mod(i,7)+1));
            plot(y(1,seg_idx{i},frm_idx), y(2,seg_idx{i},frm_idx),marker_idx(mod(i,12)+1),'Color',color_idx(mod(i,7)+1));
            hold on
        end
        axis([0 width 0 height]);
        axis off
        
        % drawing connections
        for m = 1:size(motion_MST_ii,1)
%             plot([seg_center(1,motion_MST_ii(m),frm_idx),seg_center(1,motion_MST_jj(m),frm_idx)],...
%                 [height-seg_center(2,motion_MST_ii(m),frm_idx),height-seg_center(2,motion_MST_jj(m),frm_idx)],'-ko',...
%                 'LineWidth',4,...
%                 'MarkerSize',12,...
%                 'MarkerEdgeColor','k',...
%                 'MarkerFaceColor',[1.0,0.2,0.2]);
            plot([seg_center(1,motion_MST_ii(m),frm_idx),seg_center(1,motion_MST_jj(m),frm_idx)],...
                [seg_center(2,motion_MST_ii(m),frm_idx),seg_center(2,motion_MST_jj(m),frm_idx)],'-ko',...
                'LineWidth',4,...
                'MarkerSize',12,...
                'MarkerEdgeColor','k',...
                'MarkerFaceColor',[1.0,0.2,0.2]);
                        
        end
        pause(0.1)
    end
end
