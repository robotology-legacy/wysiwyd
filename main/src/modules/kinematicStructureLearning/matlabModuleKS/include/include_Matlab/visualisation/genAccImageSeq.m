function img_output = genAccImageSeq(pathname, KineStruct, step)

% disp(sprintf('\n'));
% disp('============================');
% disp('Accumulated Image Generation');
% disp('============================');

%%
% load video
xyloObj = VideoReader([pathname,KineStruct.videoFileName]);

width = KineStruct.width;
height = KineStruct.height;

% for k = 1 : KineStruct.num_frames-1
%     mov(k).cdata = read(xyloObj, k);
% end

mov(1).cdata = read(xyloObj, 1);
accu_figure_buf = double(rgb2gray(mov(1).cdata));
weight = 0;

step_size = floor(KineStruct.num_frames / step);

% for frm_idx = 1:step:KineStruct.num_frames
for frm_idx = 1:step_size:KineStruct.num_frames-1
    mov(frm_idx).cdata = read(xyloObj, frm_idx);
    accu_figure_buf = ((weight+1)*accu_figure_buf + double(rgb2gray(mov(frm_idx).cdata))) / (2+weight);
end
for frm_idx = KineStruct.num_frames-1
    mov(frm_idx).cdata = read(xyloObj, frm_idx);
    accu_figure_buf = ((weight+1)*accu_figure_buf + double(rgb2gray(mov(frm_idx).cdata))) / (2+weight);
end
% frm_idx = KineStruct.num_frames;
% mov(frm_idx).cdata = read(xyloObj, frm_idx);
% accu_figure_buf = ((weight+1)*accu_figure_buf + double(rgb2gray(mov(frm_idx).cdata))) / (2+weight);

img_output_buf = uint8(accu_figure_buf);
img_output = img_output_buf;

end