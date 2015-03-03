%%
%=========================================================
% data load
%=========================================================
%-----------------------
% from mat files
%-----------------------

pathname = '/home/hjchang/Research/code/Matlab/cvpr2015/dataset/';
% filename_buf = {'arm.avi.mat','toy.avi.mat','YanPollefeys.mp4.mat','iCub_motion.avi.mat','iCub_hand.avi.mat','robot_arm.mp4.mat','baxter_cut.mp4.mat','hand1.mp4.mat'};
filename_buf = {'hand1.mp4.mat'};
% [filename, pathname, filterindex] = uigetfile('../dataset/*.mat');

for filename_idx = 1:size(filename_buf,2)
    
    filename = filename_buf{filename_idx};
    
    load([pathname,filename]);
    num_frames = size(y,3);
    num_points = size(y,2);
    %-------------------------------------
    W = zeros(2*num_frames, num_points);
    
    for f = 1:num_frames
        W(f,:) = y(1,:,f);
        W(f+num_frames,:) = y(2,:,f);
    end
    
    frm_idx = 1;
    
    color_idx = 'rgbcmky';
    marker_idx = '+o*xsd^v><ph';
    
    figure(678)
    set(gcf,'color','w');
    clf
    
    videoFileName = filename(1:end-4);
    xyloObj = VideoReader(['/home/hjchang/OpenCV/test_codes/lk_track/',videoFileName]);
    
    switch(videoFileName)
        case {'iCub_hand.avi'}
            curFrame = read(xyloObj,frm_idx);
            
        case {'arm.avi'}
            curFrame = read(xyloObj,frm_idx);
            y(2,:,:) = height - y(2,:,:);
            
        otherwise
            curFrame = read(xyloObj,frm_idx+init_frm-1);
    end
    
    imshow(curFrame);
    hold on
    
    
    % segmentation
    num_seg_GT = max(s);
    seg_idx_GT = cell(num_seg_GT,1);
    seg_center_GT = zeros(2,num_seg_GT,num_frames);
    for i=1:num_seg_GT
        seg_idx_GT{i} = find(s==i);
    end
    
    for i=1:num_seg_GT
        seg_center_GT(1,i,frm_idx) = mean(y(1,seg_idx_GT{i},frm_idx));
        seg_center_GT(2,i,frm_idx) = mean(y(2,seg_idx_GT{i},frm_idx));
    end
    
    % feature points
    for i=1:num_seg_GT
        plot(y(1,seg_idx_GT{i},frm_idx), y(2,seg_idx_GT{i},frm_idx),marker_idx(mod(i,12)+1),'Color',color_idx(mod(i,7)+1));
        %     plot(y(1,seg_idx_GT{i},frm_idx), height-y(2,seg_idx_GT{i},frm_idx),marker_idx(mod(i,13)+1),'Color',color_idx(mod(i,7)+1));
        hold on
    end
    axis([0 width 0 height]);
    axis off
    
    for i = 1:num_seg_GT
        text(seg_center_GT(1,i,frm_idx)+5, seg_center_GT(2,i,frm_idx)+5, num2str(i), 'Color', 'w');
%         text(seg_center_GT(1,i,frm_idx)+3, height-seg_center_GT(2,i,frm_idx)+3, num2str(i), 'Color', 'w');
    end
    
    %%
    % connections
    % baxter
    switch(videoFileName)
        case {'baxter_cut.mp4'}
            connection_idx = [1,2 ; 2,3 ; 3,4 ; 4,5 ; 5,6 ; 5,7 ; 5,11 ; 7,8 ; 8,9 ; 9,10];
        case {'YanPollefeys.mp4'}
            connection_idx = [1,2 ; 2,3 ; 3,4 ; 4,5 ; 3,6];
        case {'toy.avi'}
            connection_idx = [1,2 ; 2,3];
        case {'robot_arm.mp4'}
            connection_idx = [1,2 ; 2,3 ; 3,4 ; 4,5 ; 5,7 ; 7,6 ; 8,6];
        case {'hand1.mp4'}
            %         connection_idx = [1,2 ; 2,3 ; 3,4 ; 4,5 ; 4,7 ; 7,8 ; 7,6 ; 8,9 ; 9,10 ; 10,11 ; 9,12 ; 12,13 ; 13,14 ; 12,15 ; 15,16 ; 16,17 ; 15,18 ; 18,19 ; 19,20];
%             connection_idx = [1,2 ; 2,3 ; 3,4 ; 4,5 ; 4,7 ; 7,8 ; 18,6 ; 8,9 ; 9,10 ; 10,11 ; 9,12 ; 12,13 ; 13,14 ; 12,15 ; 15,16 ; 16,17 ; 15,18 ; 18,19 ; 19,20];
            connection_idx = [1,2 ; 2,7 ; 3,4 ; 4,5 ; 4,7 ; 7,8 ; 7,6 ; 8,9 ; 9,10 ; 10,11 ; 9,12 ; 12,13 ; 13,14 ; 12,15 ; 15,16 ; 16,17 ; 15,18 ; 18,19 ; 19,20];
        case {'iCub_hand.avi'}
%             connection_idx = [1,2 ; 2,7 ; 7,8 ; 2,3 ; 3,4 ; 3,5 ; 5,6 ];
            connection_idx = [1,2 ; 2,7 ; 7,8 ; 2,5 ; 3,4 ; 3,5 ; 5,6 ];
        case {'iCub_motion.avi'}
            connection_idx = [1,2 ; 2,3 ; 3,4 ; 4,5 ; 3,6 ; 3,7];
        case {'arm.avi'}
            connection_idx = [1,2];
    end
    
    % drawing connections
    for m = 1:size(connection_idx,1)
        plot([seg_center_GT(1,connection_idx(m,1),frm_idx),seg_center_GT(1,connection_idx(m,2),frm_idx)],[seg_center_GT(2,connection_idx(m,1),frm_idx),seg_center_GT(2,connection_idx(m,2),frm_idx)],'-wo',...
            'LineWidth',4,...
            'MarkerSize',12,...
            'MarkerEdgeColor','w',...
            'MarkerFaceColor',[1.0,0.2,0.2]);
        %     plot([seg_center_GT(1,connection_idx(m,1),frm_idx),seg_center_GT(1,connection_idx(m,2),frm_idx)],[height - seg_center_GT(2,connection_idx(m,1),frm_idx),height - seg_center_GT(2,connection_idx(m,2),frm_idx)],'-wo',...
        %         'LineWidth',4,...
        %         'MarkerSize',12,...
        %         'MarkerEdgeColor','w',...
        %         'MarkerFaceColor',[1.0,0.2,0.2]);
    end
    
    save_path = '/home/hjchang/Dropbox/paper writing/[2015][Conference][CVPR] Latent ASfM/paper/figs/fig_qualitative_result/';
    saveas(gcf, [save_path, 'GT_', videoFileName(1:end-4),'.pdf'], 'pdf')
end
