
%-----------------------------------------
% Points & Boundary
figure(1001)
set(gcf,'color','w');
imshow(255-uint8(img))
hold on
plot(y(1,:,frm_idx), y(2,:,frm_idx),'y.');

%%
%-----------------------------------------
% Skeleton distance map
figure(1002)
set(gcf,'color','w');
sdm = sqrt(rad);
sdm = fliplr(sdm);
sdm(find(sdm == 0)) = Inf;
mesh(sdm);
axis image off
view(-180,90);

%%
%-----------------------------------------
% Density map
figure(1003)
set(gcf,'color','w');
dm = densityq;
dm = fliplr(dm);
dm(find(dm < 0.00001)) = Inf;
mesh(dm);
axis image off
view(-180,89);

%%
%-----------------------------------------
% Density weighted skeleton map
figure(1004)
set(gcf,'color','w');
dwsm = combination_buf;
dwsm = fliplr(dwsm);
dwsm(find(dwsm < 20)) = Inf;
mesh(dwsm);
axis image off
view(-180,90);

%%
%-----------------------------------------
% Density weighted skeleton map with points
figure(1005)
set(gcf,'color','w');
dwsm = combination_buf;
dwsm = fliplr(dwsm);
dwsm(find(dwsm < 20)) = Inf;
hold on
plot3(width-y(1,:,frm_idx), y(2,:,frm_idx), ones(1,450)*150,'k.');
mesh(dwsm);
for ii = 1:size(skel,1)
    for jj = 1:size(skel,2)
        if skel(ii,jj) ~= 0
            plot3(width - jj, ii, 200,'w.');
        end
    end
end
% imshow(skel)
axis image off
view(-180,90);