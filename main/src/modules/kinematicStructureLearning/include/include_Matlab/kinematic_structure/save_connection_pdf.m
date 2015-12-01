%%
frm_idx_draw = 1;

color_idx = 'rgbcmky';
marker_idx = '+o*xsd^v><ph';

figure(0705)
set(gcf,'color','w');
clf

% videoFileName = filename(1:end-4);
% xyloObj = VideoReader(['/home/hjchang/OpenCV/test_codes/lk_track/',videoFileName]);
%
switch(filename(1:end-4))
    
    case {'arm.avi'}
        y(2,:,:) = height - y(2,:,:);
        seg_center(2,:,:) = height - seg_center(2,:,:);
        
end
%
% imshow(curFrame);
hold on


% % segmentation
% num_seg_GT = max(s);
% seg_idx_GT = cell(num_seg_GT,1);
% seg_center_GT = zeros(2,num_seg_GT,num_frames);
% for i=1:num_seg_GT
%     seg_idx_GT{i} = find(s==i);
% end
%
% for i=1:num_seg_GT
%     seg_center_GT(1,i,frm_idx) = mean(y(1,seg_idx_GT{i},frm_idx));
%     seg_center_GT(2,i,frm_idx) = mean(y(2,seg_idx_GT{i},frm_idx));
% end

num_joint = size(seg_center,2);
num_seg = size(seg_idx,1);

% feature points
for i=1:num_seg
    %     plot(y(1,seg_idx{i},frm_idx), y(2,seg_idx{i},frm_idx),marker_idx(mod(i,13)+1),'Color',color_idx(mod(i,7)+1));
    plot(y(1,seg_idx{i},frm_idx_draw), height-y(2,seg_idx{i},frm_idx_draw),marker_idx(mod(i,12)+1),'Color',color_idx(mod(i,7)+1));
    hold on
end
axis([0 width 0 height]);
axis off

% drawing connections
for m = 1:size(motion_MST_ii,1)
    plot([seg_center(1,motion_MST_ii(m),frm_idx_draw),seg_center(1,motion_MST_jj(m),frm_idx_draw)],...
        [height-seg_center(2,motion_MST_ii(m),frm_idx_draw),height-seg_center(2,motion_MST_jj(m),frm_idx_draw)],'-ko',...
        'LineWidth',4,...
        'MarkerSize',12,...
        'MarkerEdgeColor','k',...
        'MarkerFaceColor',[1.0,0.2,0.2]);
    
end

% save_path = '/home/hjchang/Dropbox/paper writing/[2015][Conference][CVPR] Latent ASfM/paper/figs/fig_qualitative_result/iteration/';
% save_path = '/home/hjchang/Dropbox/paper writing/[2015][Conference][CVPR] Latent ASfM/paper/figs/fig_qualitative_result/iteration_Geodesic/';
save_path = '/home/hjchang/Dropbox/paper_writing/[2015][Journal][TPAMI] Kinematic Structure Learning/paper/figs/fig_qualitative_result/iteration_Geodesic/';

saveas(gcf, [save_path, 'Proposed_G_center_3_',filename,'_iter_',num2str(iter),'.pdf'], 'pdf')
