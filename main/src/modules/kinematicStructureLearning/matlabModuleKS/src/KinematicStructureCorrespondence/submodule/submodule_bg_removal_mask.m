function mask = submodule_bg_removal_mask(cdata)

img_total_buf = zeros(cdata.height, cdata.width, 3, cdata.nFrames);
for i = 1:cdata.nFrames
    img_total_buf(:,:,:,i) = double(imread([cdata.pathname,sprintf('%04d',i-1),'.png']));
end

mask = ones(cdata.height, cdata.width);
img_mean = zeros(cdata.height, cdata.width);
img_std = zeros(cdata.height, cdata.width);

for h = 1:cdata.height
    for w = 1:cdata.width
        img_std(h,w) = (std(img_total_buf(h,w,1,:)) + std(img_total_buf(h,w,2,:)) + std(img_total_buf(h,w,3,:)) ) / 3;
    end
end

mask(img_std < 20) = 0;

figure(343)
mesh(img_std)

figure(344)
imagesc(mask)

se_dilate = strel('disk',7);
mask = imdilate(mask,se_dilate);
se_erode = strel('disk',14);
mask = imerode(mask,se_erode);

figure(345)
imagesc(mask)

end
