function img_output = genMatchImages(cdata_P,cdata_Q,KineStruct_P, KineStruct_Q, data_source_P, data_source_Q, X, method)

% height_normalised = 240;
% width_normalised = 320;
height_normalised = 480;
width_normalised = 640;

img_combined = zeros(height_normalised, 2*width_normalised);

%% load video
if (strcmp(data_source_P,'left') || strcmp(data_source_P,'right'))
    xyloObj_P = VideoReader([cdata_P.pathname,KineStruct_P.videoFileName]);
    mov(1).cdata = read(xyloObj_P, 1);
    img_P = rgb2gray(mov(1).cdata);
elseif strcmp(data_source_P,'kinect')
    img_P = uint8(ones(height_normalised, width_normalised)*0);
end

if (strcmp(data_source_Q,'left') || strcmp(data_source_Q,'right'))
    xyloObj_Q = VideoReader([cdata_Q.pathname,KineStruct_Q.videoFileName]);
    mov(1).cdata = read(xyloObj_Q, 1);
    img_Q = rgb2gray(mov(1).cdata);
elseif strcmp(data_source_Q,'kinect')
    img_Q = uint8(ones(height_normalised, width_normalised)*0);
end

%% Combined image
%---------------------------------------------------------------
img_acc_normalised_P = imresize(img_P, [height_normalised, NaN]);
width_norm_P = size(img_acc_normalised_P,2);
img_combined(:,1:width_norm_P) = img_acc_normalised_P;

ratio_height_P = height_normalised/ KineStruct_P.height;
ratio_width_P = width_norm_P / KineStruct_P.width;

%---------------------------------------------------------------
img_acc_normalised_Q = imresize(img_Q, [height_normalised, NaN]);
width_norm_Q = size(img_acc_normalised_Q,2);
img_combined(:,width_norm_P+1:width_norm_P+width_norm_Q) = img_acc_normalised_Q;

ratio_height_Q = height_normalised / KineStruct_Q.height;
ratio_width_Q = width_norm_Q/ KineStruct_Q.width;

shifted_dist = width_norm_P;

%---------------------------------------------------------------
img_combined = uint8(img_combined);

%%
% draw image
% figure(1011)
h_result = figure;
iptsetpref('ImshowBorder','tight')
imshow(img_combined);

% Drawing the connections with color segments
color_idx = 'rgbcmy';
% marker_idx = '+o*xsd^v><ph';
marker_idx = '............';

color_value = 0.99;

frm_idx_P = KineStruct_P.num_frames;
frm_idx_Q = KineStruct_Q.num_frames;

% for Graph P
if (strcmp(data_source_P,'left') || strcmp(data_source_P,'right'))
    for i=1:KineStruct_P.num_seg
        hold on
        plot(KineStruct_P.y(1,KineStruct_P.seg_idx{i},frm_idx_P) * ratio_width_P, KineStruct_P.y(2,KineStruct_P.seg_idx{i},frm_idx_P) * ratio_height_P,marker_idx(mod(i,12)+1),'Color', color_idx(mod(i,6)+1),'MarkerSize',10, 'LineWidth',3);
%             text(KineStruct_P.seg_center(1,i,frm_idx_P) * ratio_width_P + 10, KineStruct_P.seg_center(2,i,frm_idx_P) * ratio_height_P - 10, num2str(i), 'Color', 'r');
    end
end

% for Graph Q
if (strcmp(data_source_Q,'left') || strcmp(data_source_Q,'right'))
    for i=1:KineStruct_Q.num_seg
        hold on
        plot(shifted_dist + KineStruct_Q.y(1,KineStruct_Q.seg_idx{i},frm_idx_Q) * ratio_width_Q, height_normalised - KineStruct_Q.y(2,KineStruct_Q.seg_idx{i},frm_idx_Q) * ratio_height_Q, marker_idx(mod(i,12)+1),'Color',color_idx(mod(i,6)+1),'MarkerSize',10, 'LineWidth',3);
        %     text(shifted_dist + KineStruct_Q.seg_center(1,i,frm_idx_Q) * ratio_width_Q + 6, KineStruct_Q.seg_center(2,i,frm_idx_Q) * ratio_height_Q - 6, num2str(i), 'Color', 'r');
    end
end

%%
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
    plot([shifted_dist + KineStruct_Q.seg_center(1,KineStruct_Q.structure_i(m),frm_idx_Q) * ratio_width_Q, shifted_dist + joint_pts_buf(1,frm_idx_Q) * ratio_width_Q],height_normalised-[KineStruct_Q.seg_center(2,KineStruct_Q.structure_i(m),frm_idx_Q) * ratio_height_Q, joint_pts_buf(2,frm_idx_Q) * ratio_height_Q],'-','Color',[color_value,color_value,color_value],'LineWidth',4);
    plot([shifted_dist + KineStruct_Q.seg_center(1,KineStruct_Q.structure_j(m),frm_idx_Q) * ratio_width_Q, shifted_dist + joint_pts_buf(1,frm_idx_Q) * ratio_width_Q],height_normalised-[KineStruct_Q.seg_center(2,KineStruct_Q.structure_j(m),frm_idx_Q) * ratio_height_Q, joint_pts_buf(2,frm_idx_Q) * ratio_height_Q],'-','Color',[color_value,color_value,color_value],'LineWidth',4);
    
    % Node
    plot([shifted_dist + KineStruct_Q.seg_center(1,KineStruct_Q.structure_i(m),frm_idx_Q) * ratio_width_Q],[height_normalised-KineStruct_Q.seg_center(2,KineStruct_Q.structure_i(m),frm_idx_Q) * ratio_height_Q],'-ws',...
        'LineWidth',3,...
        'MarkerSize',15,...
        'MarkerEdgeColor',[color_value,color_value,color_value],...
        'MarkerFaceColor',[1.0,0.2,0.2]);
    plot([shifted_dist + KineStruct_Q.seg_center(1,KineStruct_Q.structure_j(m),frm_idx_Q) * ratio_width_Q],[height_normalised-KineStruct_Q.seg_center(2,KineStruct_Q.structure_j(m),frm_idx_Q) * ratio_height_Q],'-ws',...
        'LineWidth',3,...
        'MarkerSize',15,...
        'MarkerEdgeColor',[color_value,color_value,color_value],...
        'MarkerFaceColor',[1.0,0.2,0.2]);
    
    % Joint
    plot(shifted_dist + joint_pts_buf(1,frm_idx_Q) * ratio_width_Q, height_normalised-joint_pts_buf(2,frm_idx_Q) * ratio_height_Q,'wo',...
        'LineWidth',1,...
        'MarkerSize',9,...
        'MarkerEdgeColor',[color_value,color_value,color_value],...
        'MarkerFaceColor',[1.0,0.647,0.0]);
    plot(shifted_dist + joint_pts_buf(1,frm_idx_Q) * ratio_width_Q, height_normalised-joint_pts_buf(2,frm_idx_Q) * ratio_height_Q,'wx',...
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
                [KineStruct_P.seg_center(2,p,frm_idx_P) * ratio_height_P, height_normalised-KineStruct_Q.seg_center(2,q,frm_idx_Q) * ratio_height_Q],'-','Color',[color_value,color_value,color_value],...
                'LineWidth',3,...
                'MarkerSize',10,...
                'MarkerEdgeColor',[color_value,color_value,color_value],...
                'MarkerFaceColor',[1.0,0.2,0.2]);
            plot([KineStruct_P.seg_center(1,p,frm_idx_P) * ratio_width_P, shifted_dist + KineStruct_Q.seg_center(1,q,frm_idx_Q) * ratio_width_Q],...
                [KineStruct_P.seg_center(2,p,frm_idx_P) * ratio_height_P, height_normalised-KineStruct_Q.seg_center(2,q,frm_idx_Q) * ratio_height_Q],'-y.',...
                'LineWidth',2,...
                'MarkerSize',10,...
                'MarkerEdgeColor','y',...
                'MarkerFaceColor',[1.0,0.2,0.2]);
        end
    end
end

%% Save result
iptsetpref('ImshowBorder','tight');
[SUCCESS,MESSAGE,MESSAGEID] = rmdir('result','s');
mkdir('result/images/correspondences');
result_save_folder_images_correspondences = 'result/images/correspondences/';
fig_save_name = [result_save_folder_images_correspondences,'output'];
% fig_save_name = ['result/realSeq/',KineStruct_P.videoFileName(1:end-4),'-',KineStruct_Q.videoFileName(1:end-4),'/',KineStruct_P.videoFileName(1:end-4),'-',KineStruct_Q.videoFileName(1:end-4),'-',method];
export_fig(fig_save_name,'-png');
%%
img_output = getimage;

end