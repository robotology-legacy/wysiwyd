function img_output = genColorExampleImage(pathname_P, KineStruct_P, pathname_Q, KineStruct_Q)

height_normalised = 240;
width_normalised = 320;

% height_normalised = KineStruct_P.height;
% width_normalised = KineStruct_P.width;

%%
% load video
xyloObj_P = VideoReader([pathname_P,KineStruct_P.videoFileName]);
xyloObj_Q = VideoReader([pathname_Q,KineStruct_Q.videoFileName]);

img_P = read(xyloObj_P, 1);
img_Q = read(xyloObj_Q, 1);

% Combined image
img_combined = zeros(height_normalised, width_normalised, 3);

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
h_color = figure(333);
imshow(img_combined);
iptsetpref('ImshowBorder','tight');
mkdir('result/realSeq',[KineStruct_P.videoFileName(1:end-4),'-',KineStruct_Q.videoFileName(1:end-4)]);
fig_save_name = ['result/realSeq/',KineStruct_P.videoFileName(1:end-4),'-',KineStruct_Q.videoFileName(1:end-4),'/',KineStruct_P.videoFileName(1:end-4),'-',KineStruct_Q.videoFileName(1:end-4)];
% saveas(h_color,fig_save_name);
% print('-depsc', fig_save_name);
export_fig(fig_save_name,'-pdf');

%%
img_output = img_combined;