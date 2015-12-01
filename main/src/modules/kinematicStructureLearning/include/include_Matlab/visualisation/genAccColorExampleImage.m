function img_output = genAccColorExampleImage(pathname_P, KineStruct_P, pathname_Q, KineStruct_Q, step_size)

height_normalised = 240;
width_normalised = 320;

%%
% load video
xyloObj_P = VideoReader([pathname_P,KineStruct_P.videoFileName]);
xyloObj_Q = VideoReader([pathname_Q,KineStruct_Q.videoFileName]);

img_P = read(xyloObj_P, 1);
img_Q = read(xyloObj_Q, 1);

accu_img_P_buf = double(img_P);
accu_img_Q_buf = double(img_Q);
weight = 0;

step_size_P = floor(KineStruct_P.num_frames / step_size);
step_size_Q = floor(KineStruct_Q.num_frames / step_size);

%%
for frm_idx = 1:step_size_P:KineStruct_P.num_frames-1
    mov(frm_idx).cdata = read(xyloObj_P, frm_idx);
    accu_img_P_buf = ((weight+1)*accu_img_P_buf + double(mov(frm_idx).cdata)) / (2+weight);
end

    mov(frm_idx).cdata = read(xyloObj_P, KineStruct_P.num_frames-1);
    accu_img_P_buf = ((weight+1)*accu_img_P_buf + double(mov(frm_idx).cdata)) / (2+weight);


img_P = uint8(accu_img_P_buf);

%%
for frm_idx = 1:step_size_Q:KineStruct_Q.num_frames-1
    mov(frm_idx).cdata = read(xyloObj_Q, frm_idx);
    accu_img_Q_buf = ((weight+1)*accu_img_Q_buf + double(mov(frm_idx).cdata)) / (2+weight);
end
frm_idx = KineStruct_Q.num_frames-1;
mov(frm_idx).cdata = read(xyloObj_Q, frm_idx);
accu_img_Q_buf = ((weight+1)*accu_img_Q_buf + double(mov(frm_idx).cdata)) / (2+weight);


img_Q = uint8(accu_img_Q_buf);

%% Combined image
img_combined = zeros(height_normalised, 2*width_normalised, 3);

img_normalised_P = imresize(img_P, [height_normalised, NaN]);
width_norm_P = size(img_normalised_P,2);
img_combined(:,1:width_norm_P,:) = img_normalised_P;

%---------------------------------------------------------------
img_normalised_Q = imresize(img_Q, [height_normalised, NaN]);
width_norm_Q = size(img_normalised_Q,2);
img_combined(:,width_norm_P+1:width_norm_P+width_norm_Q,:) = img_normalised_Q;

%---------------------------------------------------------------
img_combined = uint8(img_combined);

%% draw image
h_color = figure(334);
imshow(img_combined);
iptsetpref('ImshowBorder','tight');
mkdir('result/realSeq',[KineStruct_P.videoFileName(1:end-4),'-',KineStruct_Q.videoFileName(1:end-4)]);
fig_save_name = ['result/realSeq/',KineStruct_P.videoFileName(1:end-4),'-',KineStruct_Q.videoFileName(1:end-4),'/',KineStruct_P.videoFileName(1:end-4),'-',KineStruct_Q.videoFileName(1:end-4),'-accumulate'];
% saveas(h_color,fig_save_name);
% print('-depsc', fig_save_name);
export_fig(fig_save_name,'-pdf');
%%
img_output = img_combined;