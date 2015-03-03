close all

% read in a sample image -- also see letters.png, bagel.png
% img = imread('mushroom.png');
img = imread('dog-5.gif');
se = strel('ball',3,3);
img = imdilate(img,se);
% img = bwmorph(img,'open');
% img = 255-img;
% img = imread('face.bmp');
% img = imresize(img,0.5);
% img = rgb2gray(img);
figure(1)
imshow(img);

% the standard skeletonization:
figure(2)
imshow(bwmorph(img,'skel',inf));

% the new method:
figure(3)
imshow(bwmorph(skeleton(img)>35,'skel',Inf));

% in more detail:
[skr,rad] = skeleton(img);

% the intensity at each point is proportional to the degree of evidence
% that this should be a point on the skeleton:
figure(4)
imagesc(skr);
colormap jet
axis image off

% skeleton can also return a map of the radius of the largest circle that
% fits within the foreground at each point:
figure(5)
imagesc(rad)
colormap jet
axis image off

% thresholding the skeleton can return skeletons of thickness 2,
% so the call to bwmorph completes the thinning to single-pixel width.
skel = bwmorph(skr > 35,'skel',inf);
figure(6)
imshow(skel)
% try different thresholds besides 35 to see the effects

% anaskel returns the locations of endpoints and junction points
[dmap,exy,jxy] = anaskel(skel);
hold on
plot(exy(1,:),exy(2,:),'go')
plot(jxy(1,:),jxy(2,:),'ro')
