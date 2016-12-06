function img_output = genColorImageStructure(pathname_P, KineStruct_P, pathname_Q, KineStruct_Q)
%%
% load video
xyloObj_P = VideoReader([pathname_P,KineStruct_P.videoFileName]);
xyloObj_Q = VideoReader([pathname_Q,KineStruct_Q.videoFileName]);

frm_idx_P = round(KineStruct_P.num_frames/2);
frm_idx_Q = round(KineStruct_Q.num_frames/2);

img_P = read(xyloObj_P, frm_idx_P);
img_Q = read(xyloObj_Q, frm_idx_Q);


%% draw image
% P
h_color = figure(335);
imshow(img_P);
iptsetpref('ImshowBorder','tight');

% Drawing the connections with color segments
color_idx = 'rgbcmy';
% marker_idx = '+o*xsd^v><ph';
marker_idx = '............';

color_value = 0.99;

% for Graph P

for i=1:KineStruct_P.num_seg
    hold on
    plot(KineStruct_P.y(1,KineStruct_P.seg_idx{i},frm_idx_P), KineStruct_P.y(2,KineStruct_P.seg_idx{i},frm_idx_P),marker_idx(mod(i,12)+1),'Color', color_idx(mod(i,6)+1),'MarkerSize',10, 'LineWidth',3);
end


% drawing connections for Graph P
for m = 1:size(KineStruct_P.structure_i,1)
    hold on
    
    joint_pts_buf = KineStruct_P.joint_center{KineStruct_P.structure_i(m),KineStruct_P.structure_j(m)};

    % Connection
    plot([KineStruct_P.seg_center(1,KineStruct_P.structure_i(m),frm_idx_P), joint_pts_buf(1,frm_idx_P)],[KineStruct_P.seg_center(2,KineStruct_P.structure_i(m),frm_idx_P), joint_pts_buf(2,frm_idx_P)],'-','Color',[color_value,color_value,color_value],'LineWidth',4);
    plot([KineStruct_P.seg_center(1,KineStruct_P.structure_j(m),frm_idx_P), joint_pts_buf(1,frm_idx_P)],[KineStruct_P.seg_center(2,KineStruct_P.structure_j(m),frm_idx_P), joint_pts_buf(2,frm_idx_P)],'-','Color',[color_value,color_value,color_value],'LineWidth',4);
    
    % Node
    plot([KineStruct_P.seg_center(1,KineStruct_P.structure_i(m),frm_idx_P)],[KineStruct_P.seg_center(2,KineStruct_P.structure_i(m),frm_idx_P)],'-ws',...
        'LineWidth',3,...
        'MarkerSize',15,...
        'MarkerEdgeColor',[color_value,color_value,color_value],...
        'MarkerFaceColor',[1.0,0.2,0.2]);
    plot([KineStruct_P.seg_center(1,KineStruct_P.structure_j(m),frm_idx_P)],[KineStruct_P.seg_center(2,KineStruct_P.structure_j(m),frm_idx_P)],'-ws',...
        'LineWidth',3,...
        'MarkerSize',15,...
        'MarkerEdgeColor',[color_value,color_value,color_value],...
        'MarkerFaceColor',[1.0,0.2,0.2]);
    
    % Joint
    plot(joint_pts_buf(1,frm_idx_P), joint_pts_buf(2,frm_idx_P),'wo',...
        'LineWidth',1,...
        'MarkerSize',9,...
        'MarkerEdgeColor',[color_value,color_value,color_value],...
        'MarkerFaceColor',[1.0,0.647,0.0]);
    plot(joint_pts_buf(1,frm_idx_P), joint_pts_buf(2,frm_idx_P),'wx',...
        'LineWidth',1,...
        'MarkerSize',9,...
        'MarkerEdgeColor',[color_value,color_value,color_value],...
        'MarkerFaceColor',[1.0,0.647,0.0]);
        
    
end
%% P
fig_save_name = ['result/realSeq/',KineStruct_P.videoFileName(1:end-4),'-',KineStruct_Q.videoFileName(1:end-4),'/',KineStruct_P.videoFileName(1:end-4),'-structure'];
% saveas(h_color,fig_save_name);
% print('-depsc', fig_save_name);
export_fig(fig_save_name,'-pdf');

%%
% Q
h_color = figure(336);
imshow(img_Q);
iptsetpref('ImshowBorder','tight');

% for Graph Q
for i=1:KineStruct_Q.num_seg
    hold on
    plot(KineStruct_Q.y(1,KineStruct_Q.seg_idx{i},frm_idx_Q), KineStruct_Q.y(2,KineStruct_Q.seg_idx{i},frm_idx_Q), marker_idx(mod(i,12)+1),'Color',color_idx(mod(i,6)+1),'MarkerSize',10, 'LineWidth',3);
end

% drawing connections for Graph Q
for m = 1:size(KineStruct_Q.structure_i,1)
    hold on
    
    joint_pts_buf = KineStruct_Q.joint_center{KineStruct_Q.structure_i(m),KineStruct_Q.structure_j(m)};
  
    % Connection
    plot([KineStruct_Q.seg_center(1,KineStruct_Q.structure_i(m),frm_idx_Q), joint_pts_buf(1,frm_idx_Q)],[KineStruct_Q.seg_center(2,KineStruct_Q.structure_i(m),frm_idx_Q), joint_pts_buf(2,frm_idx_Q)],'-','Color',[color_value,color_value,color_value],'LineWidth',4);
    plot([KineStruct_Q.seg_center(1,KineStruct_Q.structure_j(m),frm_idx_Q), joint_pts_buf(1,frm_idx_Q)],[KineStruct_Q.seg_center(2,KineStruct_Q.structure_j(m),frm_idx_Q), joint_pts_buf(2,frm_idx_Q)],'-','Color',[color_value,color_value,color_value],'LineWidth',4);
    
    % Node
    plot([KineStruct_Q.seg_center(1,KineStruct_Q.structure_i(m),frm_idx_Q)],[KineStruct_Q.seg_center(2,KineStruct_Q.structure_i(m),frm_idx_Q)],'-ws',...
        'LineWidth',3,...
        'MarkerSize',15,...
        'MarkerEdgeColor',[color_value,color_value,color_value],...
        'MarkerFaceColor',[1.0,0.2,0.2]);
    plot([KineStruct_Q.seg_center(1,KineStruct_Q.structure_j(m),frm_idx_Q)],[KineStruct_Q.seg_center(2,KineStruct_Q.structure_j(m),frm_idx_Q)],'-ws',...
        'LineWidth',3,...
        'MarkerSize',15,...
        'MarkerEdgeColor',[color_value,color_value,color_value],...
        'MarkerFaceColor',[1.0,0.2,0.2]);
    
    % Joint
    plot(joint_pts_buf(1,frm_idx_Q), joint_pts_buf(2,frm_idx_Q),'wo',...
        'LineWidth',1,...
        'MarkerSize',9,...
        'MarkerEdgeColor',[color_value,color_value,color_value],...
        'MarkerFaceColor',[1.0,0.647,0.0]);
    plot(joint_pts_buf(1,frm_idx_Q), joint_pts_buf(2,frm_idx_Q),'wx',...
        'LineWidth',1,...
        'MarkerSize',9,...
        'MarkerEdgeColor',[color_value,color_value,color_value],...
        'MarkerFaceColor',[1.0,0.647,0.0]);    
    
end


%% P
fig_save_name = ['result/realSeq/',KineStruct_P.videoFileName(1:end-4),'-',KineStruct_Q.videoFileName(1:end-4),'/',KineStruct_Q.videoFileName(1:end-4),'-structure'];
% saveas(h_color,fig_save_name);
% print('-depsc', fig_save_name);
export_fig(fig_save_name,'-pdf');
%%
img_output = img_P;