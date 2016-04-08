function mask = submodule_bg_removal_mask(cdata)

% img_total_buf = cell(num_img,1);
% for i = 1:num_img
%     img_total_buf{i} = double(imread(['data/KS/',sprintf('%04d',i-1),'.png']));
% end

% [height, width, color] = size(img_total_buf{1});

img_total_buf = zeros(cdata.height, cdata.width, 3, cdata.nFrames);
for i = 1:cdata.nFrames
    img_total_buf(:,:,:,i) = double(imread(['data/KS/',sprintf('%04d',i-1),'.png']));
end

mask = ones(cdata.height, cdata.width);
img_mean = zeros(cdata.height, cdata.width);
img_std = zeros(cdata.height, cdata.width);

for h = 1:cdata.height
    for w = 1:cdata.width
%         img_mean(h,w) = (mean(img_total_buf(h,w,1,:)) + mean(img_total_buf(h,w,2,:)) + mean(img_total_buf(h,w,3,:)) ) / 3;
        img_std(h,w) = (std(img_total_buf(h,w,1,:)) + std(img_total_buf(h,w,2,:)) + std(img_total_buf(h,w,3,:)) ) / 3;
    end
end

mask(img_std < 10) = 0;

figure(343)
mesh(img_std)

figure(344)
imagesc(mask)
end
