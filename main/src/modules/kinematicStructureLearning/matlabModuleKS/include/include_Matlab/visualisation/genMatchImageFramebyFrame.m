function img_output = genMatchImageFramebyFrame(img_acc_P, img_acc_Q, KineStruct_P, KineStruct_Q, X, method, pathname_P, pathname_Q)

nFrm = 6;
frm_idx_set_P = [floor(KineStruct_P.num_frames/nFrm):floor((KineStruct_P.num_frames-1)/nFrm):KineStruct_P.num_frames-1];
frm_idx_set_Q = [floor(KineStruct_Q.num_frames/nFrm):floor((KineStruct_Q.num_frames-1)/nFrm):KineStruct_Q.num_frames-1];

for idx = 1:nFrm
    frm_idx_P = frm_idx_set_P(idx);
    frm_idx_Q = frm_idx_set_Q(idx);
    
    %%
    % load video
    xyloObj_P = VideoReader([pathname_P,KineStruct_P.videoFileName]);
    xyloObj_Q = VideoReader([pathname_Q,KineStruct_Q.videoFileName]);
    
    width_P = KineStruct_P.width;
    width_Q = KineStruct_Q.width;
    height_P = KineStruct_P.height;    
    height_Q = KineStruct_Q.height;
    
%     img_acc_P = uint8(rgb2gray(read(xyloObj_P, frm_idx_P)));
%     img_acc_Q = uint8(rgb2gray(read(xyloObj_Q, frm_idx_Q)));    
    img_acc_P = uint8(read(xyloObj_P, frm_idx_P));
    img_acc_Q = uint8(read(xyloObj_Q, frm_idx_Q));    
    
    %%
    height_normalised = 240;
    width_normalised = 320;
    
    % Combined image
    img_combined = zeros(height_normalised, 2*width_normalised);
    
    %---------------------------------------------------------------
    img_acc_normalised_P = imresize(img_acc_P, [height_normalised, NaN]);
    width_norm_P = size(img_acc_normalised_P,2);
    img_combined(:,1:width_norm_P,1) = img_acc_normalised_P(:,:,1);
    img_combined(:,1:width_norm_P,2) = img_acc_normalised_P(:,:,2);
    img_combined(:,1:width_norm_P,3) = img_acc_normalised_P(:,:,3);
    
    ratio_height_P = height_normalised/ KineStruct_P.height;
    ratio_width_P = width_norm_P / KineStruct_P.width;
    
    %---------------------------------------------------------------
    img_acc_normalised_Q = imresize(img_acc_Q, [height_normalised, NaN]);
    width_norm_Q = size(img_acc_normalised_Q,2);
    img_combined(:,width_norm_P+1:width_norm_P+width_norm_Q,:) = img_acc_normalised_Q;
    
    ratio_height_Q = height_normalised / KineStruct_Q.height;
    ratio_width_Q = width_norm_Q/ KineStruct_Q.width;
    
    shifted_dist = width_norm_P;
    
    %---------------------------------------------------------------
%     img_combined = uint8(img_combined);
    
    img_combined = uint8(ones(size(img_combined,1), size(img_combined,2),3)*255);
    
    %%
    % draw image
    % figure(1011)
    h_result = figure(342);
    clf
    iptsetpref('ImshowBorder','tight')
    imshow(img_combined);
    
    % Drawing the connections with color segments
    color_idx = 'rgbcmy';
    % marker_idx = '+o*xsd^v><ph';
    marker_idx = '............';
    
%     color_value = 0.99;
    color_value = 0.0;
    
%     % for Graph P
%     for i=1:KineStruct_P.num_seg
%         hold on
%         plot(KineStruct_P.y(1,KineStruct_P.seg_idx{i},frm_idx_P) * ratio_width_P, KineStruct_P.y(2,KineStruct_P.seg_idx{i},frm_idx_P) * ratio_height_P,marker_idx(mod(i,12)+1),'Color', color_idx(mod(i,6)+1),'MarkerSize',10, 'LineWidth',3);
%     end
%     % for Graph Q
%     for i=1:KineStruct_Q.num_seg
%         hold on
%         plot(shifted_dist + KineStruct_Q.y(1,KineStruct_Q.seg_idx{i},frm_idx_Q) * ratio_width_Q, KineStruct_Q.y(2,KineStruct_Q.seg_idx{i},frm_idx_Q) * ratio_height_Q, marker_idx(mod(i,12)+1),'Color',color_idx(mod(i,6)+1),'MarkerSize',10, 'LineWidth',3);
%     end
    
    % drawing connections for Graph P
    for m = 1:size(KineStruct_P.structure_i,1)
        hold on
        
        joint_pts_buf = KineStruct_P.joint_center{KineStruct_P.structure_i(m),KineStruct_P.structure_j(m)};
        
        % Connection
        plot([KineStruct_P.seg_center(1,KineStruct_P.structure_i(m),frm_idx_P) * ratio_width_P, joint_pts_buf(1,frm_idx_P) * ratio_width_P],[KineStruct_P.seg_center(2,KineStruct_P.structure_i(m),frm_idx_P) * ratio_height_P, joint_pts_buf(2,frm_idx_P) * ratio_height_P],'-','Color',[color_value,color_value,color_value],'LineWidth',4);
        plot([KineStruct_P.seg_center(1,KineStruct_P.structure_j(m),frm_idx_P) * ratio_width_P, joint_pts_buf(1,frm_idx_P) * ratio_width_P],[KineStruct_P.seg_center(2,KineStruct_P.structure_j(m),frm_idx_P) * ratio_height_P, joint_pts_buf(2,frm_idx_P) * ratio_height_P],'-','Color',[color_value,color_value,color_value],'LineWidth',4);
        
        % Node
        plot([KineStruct_P.seg_center(1,KineStruct_P.structure_i(m),frm_idx_P) * ratio_width_P],[KineStruct_P.seg_center(2,KineStruct_P.structure_i(m),frm_idx_P) * ratio_height_P],'-ws',...
            'LineWidth',3,...
            'MarkerSize',15,...
            'MarkerEdgeColor',[color_value,color_value,color_value],...
            'MarkerFaceColor',[1.0,0.2,0.2]);
        plot([KineStruct_P.seg_center(1,KineStruct_P.structure_j(m),frm_idx_P) * ratio_width_P],[KineStruct_P.seg_center(2,KineStruct_P.structure_j(m),frm_idx_P) * ratio_height_P],'-ws',...
            'LineWidth',3,...
            'MarkerSize',15,...
            'MarkerEdgeColor',[color_value,color_value,color_value],...
            'MarkerFaceColor',[1.0,0.2,0.2]);
        
        % Joint
        plot(joint_pts_buf(1,frm_idx_P) * ratio_width_P, joint_pts_buf(2,frm_idx_P) * ratio_height_P,'wo',...
            'LineWidth',1,...
            'MarkerSize',9,...
            'MarkerEdgeColor',[color_value,color_value,color_value],...
            'MarkerFaceColor',[1.0,0.647,0.0]);
        plot(joint_pts_buf(1,frm_idx_P) * ratio_width_P, joint_pts_buf(2,frm_idx_P) * ratio_height_P,'wx',...
            'LineWidth',1,...
            'MarkerSize',9,...
            'MarkerEdgeColor',[color_value,color_value,color_value],...
            'MarkerFaceColor',[1.0,0.647,0.0]);
    end
    % drawing connections for Graph Q
    for m = 1:size(KineStruct_Q.structure_i,1)
        hold on
        
        joint_pts_buf = KineStruct_Q.joint_center{KineStruct_Q.structure_i(m),KineStruct_Q.structure_j(m)};
        
        % Connection
        plot([shifted_dist + KineStruct_Q.seg_center(1,KineStruct_Q.structure_i(m),frm_idx_Q) * ratio_width_Q, shifted_dist + joint_pts_buf(1,frm_idx_Q) * ratio_width_Q],[KineStruct_Q.seg_center(2,KineStruct_Q.structure_i(m),frm_idx_Q) * ratio_height_Q, joint_pts_buf(2,frm_idx_Q) * ratio_height_Q],'-','Color',[color_value,color_value,color_value],'LineWidth',4);
        plot([shifted_dist + KineStruct_Q.seg_center(1,KineStruct_Q.structure_j(m),frm_idx_Q) * ratio_width_Q, shifted_dist + joint_pts_buf(1,frm_idx_Q) * ratio_width_Q],[KineStruct_Q.seg_center(2,KineStruct_Q.structure_j(m),frm_idx_Q) * ratio_height_Q, joint_pts_buf(2,frm_idx_Q) * ratio_height_Q],'-','Color',[color_value,color_value,color_value],'LineWidth',4);
        
        % Node
        plot([shifted_dist + KineStruct_Q.seg_center(1,KineStruct_Q.structure_i(m),frm_idx_Q) * ratio_width_Q],[KineStruct_Q.seg_center(2,KineStruct_Q.structure_i(m),frm_idx_Q) * ratio_height_Q],'-ws',...
            'LineWidth',3,...
            'MarkerSize',15,...
            'MarkerEdgeColor',[color_value,color_value,color_value],...
            'MarkerFaceColor',[1.0,0.2,0.2]);
        plot([shifted_dist + KineStruct_Q.seg_center(1,KineStruct_Q.structure_j(m),frm_idx_Q) * ratio_width_Q],[KineStruct_Q.seg_center(2,KineStruct_Q.structure_j(m),frm_idx_Q) * ratio_height_Q],'-ws',...
            'LineWidth',3,...
            'MarkerSize',15,...
            'MarkerEdgeColor',[color_value,color_value,color_value],...
            'MarkerFaceColor',[1.0,0.2,0.2]);
        
        % Joint
        plot(shifted_dist + joint_pts_buf(1,frm_idx_Q) * ratio_width_Q, joint_pts_buf(2,frm_idx_Q) * ratio_height_Q,'wo',...
            'LineWidth',1,...
            'MarkerSize',9,...
            'MarkerEdgeColor',[color_value,color_value,color_value],...
            'MarkerFaceColor',[1.0,0.647,0.0]);
        plot(shifted_dist + joint_pts_buf(1,frm_idx_Q) * ratio_width_Q, joint_pts_buf(2,frm_idx_Q) * ratio_height_Q,'wx',...
            'LineWidth',1,...
            'MarkerSize',9,...
            'MarkerEdgeColor',[color_value,color_value,color_value],...
            'MarkerFaceColor',[1.0,0.647,0.0]);
        
    end
    
    %% Draw matches
    color_value = 0;
    for p = 1:size(X,1)
        for q = 1:size(X,2)
            if X(p,q) > 0
                hold on
                plot([KineStruct_P.seg_center(1,p,frm_idx_P) * ratio_width_P, shifted_dist + KineStruct_Q.seg_center(1,q,frm_idx_Q) * ratio_width_Q],...
                    [KineStruct_P.seg_center(2,p,frm_idx_P) * ratio_height_P, KineStruct_Q.seg_center(2,q,frm_idx_Q) * ratio_height_Q],'-','Color',[color_value,color_value,color_value],...
                    'LineWidth',3,...
                    'MarkerSize',10,...
                    'MarkerEdgeColor',[color_value,color_value,color_value],...
                    'MarkerFaceColor',[1.0,0.2,0.2]);
                plot([KineStruct_P.seg_center(1,p,frm_idx_P) * ratio_width_P, shifted_dist + KineStruct_Q.seg_center(1,q,frm_idx_Q) * ratio_width_Q],...
                    [KineStruct_P.seg_center(2,p,frm_idx_P) * ratio_height_P, KineStruct_Q.seg_center(2,q,frm_idx_Q) * ratio_height_Q],'-y.',...
                    'LineWidth',2,...
                    'MarkerSize',10,...
                    'MarkerEdgeColor','y',...
                    'MarkerFaceColor',[1.0,0.2,0.2]);
            end
        end
    end
    
    %% Save result
    iptsetpref('ImshowBorder','tight')
    fig_save_name = ['result/realSeq/',KineStruct_P.videoFileName(1:end-4),'-',KineStruct_Q.videoFileName(1:end-4),'/',KineStruct_P.videoFileName(1:end-4),'-',KineStruct_Q.videoFileName(1:end-4),'-',num2str(idx),'-',method,'-skeleton'];
%     fig_save_name = ['result/realSeq/',KineStruct_P.videoFileName(1:end-4),'-',KineStruct_Q.videoFileName(1:end-4),'/',KineStruct_P.videoFileName(1:end-4),'-',KineStruct_Q.videoFileName(1:end-4),'-',num2str(idx),'-',method];
    export_fig(fig_save_name,'-pdf');
    %%
    img_output = getimage;
    
end
end