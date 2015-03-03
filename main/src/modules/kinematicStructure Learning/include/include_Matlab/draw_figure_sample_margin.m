
pathname = '/home/hjchang/Research/code/Matlab/cvpr2015/dataset/';
% filename_buf = {'arm.avi.mat','toy.avi.mat','YanPollefeys.mp4.mat','iCub_motion.avi.mat','iCub_hand.avi.mat','robot_arm.mp4.mat','baxter_cut.mp4.mat','hand1.mp4.mat'};
filename_buf = 'baxter_cut.mp4.mat';
load([pathname,filename_buf]);

%%
[y_dim,num_points,num_frames] = size(y);

x_org_full = zeros(num_points,2,num_frames);

for f = 1:num_frames
    for n = 1:num_points
        x_org_full(n,1,f) = y(1,n,f);
        x_org_full(n,2,f) = height - y(2,n,f);
    end
end

dim = 2;
x_org = x_org_full(:,:,1);

ndata = size(x_org,1);
y_org = ones(ndata,1);

%----------------------------------------------------

global C
global kernel_param
global kernel_type
global eps
global alpha
global beta
global gamma
global SV
global SV_class_idx
global a
global R2
global mu
global learning_type;
global num_class;
global SET
global g

global Qxx
global Q
global Sidx
global Eidx
global Oidx
global cidx

%---------------- Variable Setting ------------------
C = 1;                             % C value of SVDD
% kernel_param = 30;                  % kernel parameter
kernel_type = 'erbf';          % kernel types
%  - 'gaussian'
%  - 'linear'
%  - 'polynomial'
%  - 'sigmoid'
%  - 'erbf'
prec = 1e-4;
eps = 1e-16;

learning_type = 'batch';
% learning_type = 'incremental';

time_delay = 0.01;
num_class = 1;

SET = struct('S',[],'E',[],'O',[]);
SET.S = struct('x',[],'y',[],'alpha',[],'g',[],'ndata',[]);
SET.S.x = [];
SET.S.y = [];
SET.S.alpha = [];
SET.S.g = [];
SET.S.ndata = 0;
SET.E = struct('x',[],'y',[],'alpha',[],'g',[],'ndata',[]);
SET.E.x = [];
SET.E.y = [];
SET.E.alpha = [];
SET.E.g = [];
SET.E.ndata = 0;
SET.O = struct('x',[],'y',[],'alpha',[],'g',[],'ndata',[]);
SET.O.x = [];
SET.O.y = [];
SET.O.alpha = [];
SET.O.g = [];
SET.O.ndata = 0;
%----------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%% Learning ���� %%%%%%%%%%%%%%%%%%%%%%%%%%%%
num_class = size(unique(y_org),1);

x_SVDD = [];
y_SVDD = [];

sm_entropy = [];

tic
kernel_max = width / 2;
kernel_min = width / 50;
kernel_step = width / 100;

kernel_param_buf = kernel_min:kernel_step:kernel_max;

num_kernel_param = size(kernel_param_buf,2);

param_counter = 0;

sample_margin_buf = cell(num_kernel_param,1);
iter = 0;
for kernel_param = kernel_param_buf
    %     kernel_param
    clc;
    iter = iter + 1;
    param_counter = param_counter+1;
    disp('Searching for an optimal kernal parameter');
    buf = sprintf('%d %% completed.',ceil(param_counter/num_kernel_param*100));
    disp(buf);
    switch learning_type
        case 'batch'
            x_SVDD = x_org;
            y_SVDD = y_org;
            SET = incSVDD(x_SVDD,y_SVDD,C,kernel_type,kernel_param,0);
            
            sample_margin = CalSampleMargin(SET.O.x,SET.O.y,SET,kernel_param,kernel_type);  % calculate the sample margin
            sm_max = max(sample_margin);
            sm_min = min(sample_margin);
            sm_entropy = [sm_entropy,entropy(sample_margin)];
            
            sample_margin_buf{iter} = sample_margin;
            
        case 'incremental'
            for incdata_idx=1:ndata
                disp(['incdata_idx = ',num2str(incdata_idx)]);
                
                num_data = 20;
                
                if incdata_idx < num_data
                    x_SVDD = [x_SVDD;x_org(incdata_idx,:)];
                    y_SVDD = [y_SVDD;y_org(incdata_idx)];
                elseif incdata_idx == num_data
                    SET = incSVDD(x_SVDD,y_SVDD,C,kernel_type,kernel_param,0);
                    incSVDD_drawing(x_SVDD,y_SVDD,SET,kernel_param,kernel_type,time_delay);
                else
                    SET = incSVDD(x_org(incdata_idx,:),y_org(incdata_idx,:),C,kernel_type,kernel_param,1,SET);
                    incSVDD_drawing(x_SVDD,y_SVDD,SET,kernel_param,kernel_type,time_delay);
                end
            end
    end
end
training_time = toc
% incSVDD_drawing(x,y,SET,kernel_param,kernel_type,time_delay);

figure(99)
plot(sm_entropy)

disp('Entropy');
[value, optimal_kern_param_entropy] = max(sm_entropy);
kernel_param = floor(kernel_param_buf(optimal_kern_param_entropy))
SET = incSVDD(x_SVDD,y_SVDD,C,kernel_type,kernel_param,0);
% incSVDD_drawing(x_SVDD,y_SVDD,SET,kernel_param,kernel_type,time_delay);


%%
frm_idx = 1;
small_idx = optimal_kern_param_entropy-15;
mid_idx = optimal_kern_param_entropy;
big_idx = optimal_kern_param_entropy+20;

handles = zeros(3,3);

h = figure(345)
% clf
handles(1,1) = subplot(3,3,1)
% handles(2,1) = subplot(3,3,4)
kernel_param = floor(kernel_param_buf(small_idx))
SET = incSVDD(x_SVDD,y_SVDD,C,kernel_type,kernel_param,0);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% image binarisation
width_buf = repmat([1:width]',[1,height]);
width_buf2 = reshape(width_buf',[width*height,1]);
height_buf = [1:height]';
height_buf2 = repmat(height_buf,[width,1]);
test_pixel = [width_buf2,height_buf2];

test_result = testSVDD(test_pixel, SET, kernel_param, kernel_type);
bw_img = uint8(test_result < 0)*255;
bw_img = reshape(bw_img, [height,width]);

%         img = flipud(bw_img);
img = bw_img;

small_binary_image = uint8(img);
%         imshow(small_binary_image);
contour(small_binary_image,1);
colormap([0,0,0])

hold on
set(gcf,'color','w');
%     plot(y(1,:,frm_idx),y(2,:,frm_idx),'b.');
for i=1:size(y,2)
    plot(y(1,i,frm_idx),height-y(2,i,frm_idx),'Marker','.','Color',[0.5,0.5,0.5],'MarkerSize',4.5);
end
contour(small_binary_image);
axis([0 width 0 height]);
axis off
title_text = sprintf('Kernel parameter = %d',kernel_param);
title(title_text);


handles(1,2) = subplot(3,3,2)
% handles(2,2) = subplot(3,3,5)
kernel_param = floor(kernel_param_buf(mid_idx))
SET = incSVDD(x_SVDD,y_SVDD,C,kernel_type,kernel_param,0);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% image binarisation
width_buf = repmat([1:width]',[1,height]);
width_buf2 = reshape(width_buf',[width*height,1]);
height_buf = [1:height]';
height_buf2 = repmat(height_buf,[width,1]);
test_pixel = [width_buf2,height_buf2];

test_result = testSVDD(test_pixel, SET, kernel_param, kernel_type);
bw_img = uint8(test_result < 0)*255;
bw_img = reshape(bw_img, [height,width]);

%         img = flipud(bw_img);
img = bw_img;

mid_binary_image = uint8(img);
%         imshow(mid_binary_image);
contour(mid_binary_image,1);

hold on
set(gcf,'color','w');
%     plot(y(1,:,frm_idx),y(2,:,frm_idx),'b.');
for i=1:size(y,2)
    plot(y(1,i,frm_idx),height-y(2,i,frm_idx),'Marker','.','Color',[0.5,0.5,0.5],'MarkerSize',4.5);
end
contour(mid_binary_image);
axis([0 width 0 height]);
axis off
title_text = sprintf('Kernel parameter = %d',kernel_param);
title(title_text);

handles(1,3) = subplot(3,3,3)
% handles(2,3) = subplot(3,3,6)
kernel_param = floor(kernel_param_buf(big_idx))
SET = incSVDD(x_SVDD,y_SVDD,C,kernel_type,kernel_param,0);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% image binarisation
width_buf = repmat([1:width]',[1,height]);
width_buf2 = reshape(width_buf',[width*height,1]);
height_buf = [1:height]';
height_buf2 = repmat(height_buf,[width,1]);
test_pixel = [width_buf2,height_buf2];

test_result = testSVDD(test_pixel, SET, kernel_param, kernel_type);
bw_img = uint8(test_result < 0)*255;
bw_img = reshape(bw_img, [height,width]);

%         img = flipud(bw_img);
img = bw_img;

big_binary_image = uint8(img);
%         imshow(big_binary_image);
contour(big_binary_image,1);

hold on
set(gcf,'color','w');
%     plot(y(1,:,frm_idx),y(2,:,frm_idx),'b.');
for i=1:size(y,2)
    plot(y(1,i,frm_idx),height-y(2,i,frm_idx),'Marker','.','Color',[0.5,0.5,0.5],'MarkerSize',4.5);
end
contour(big_binary_image);
axis([0 width 0 height]);
axis off
title_text = sprintf('Kernel parameter = %d',kernel_param);
title(title_text);

%-------------------------------------------------
handles(2,1) = subplot(3,3,4)
% handles(3,1) = subplot(3,3,7)
sm = sample_margin_buf{small_idx};
hist(sm,100)

xlabel('sample margin $\gamma$','Interpreter','latex');
ylabel('Probability');
xlim([min(sm),max(sm)])

N = hist(sm,100);
sm_size = size(sm,2);
y_label_value = N/sm_size;
max_y_value = max(y_label_value);
% max_y_value = 0.1;
y_labels = [0:max_y_value/3:max_y_value]';
y_labels = round(y_labels * 100)/100;
set(gca,'YTickLabel',y_labels);
%-------------------------------------------------
handles(2,2) = subplot(3,3,5)
% handles(3,2) = subplot(3,3,8)
sm = sample_margin_buf{mid_idx+3};
hist(sm,100)

xlabel('sample margin $\gamma$','Interpreter','latex');
ylabel('Probability');
xlim([min(sm),max(sm)])

N = hist(sm,100);
sm_size = size(sm,2);
y_label_value = N/sm_size;
max_y_value = max(y_label_value);
% max_y_value = 0.1;
y_labels = [0:max_y_value/3:max_y_value]';
y_labels = round(y_labels * 100)/100;
set(gca,'YTickLabel',y_labels);

%-------------------------------------------------
handles(2,3) = subplot(3,3,6)
% handles(3,3) = subplot(3,3,9)
sm = sample_margin_buf{big_idx};
hist(sm,100)

xlabel('sample margin $\gamma$','Interpreter','latex');
ylabel('Probability');
xlim([min(sm),max(sm)-0.003])

N = hist(sm,100);
sm_size = size(sm,2);
y_label_value = N/sm_size;
max_y_value = max(y_label_value);
% max_y_value = 0.1;
y_labels = [0:max_y_value/3:max_y_value]';
y_labels = round(y_labels * 100)/100;
set(gca,'YTickLabel',y_labels);

%-------------------------------------------------
handles(3,1) = subplot(3,3,[7:9])
% subplot(3,3,[1:3])
plot(sm_entropy)
ylabel('Entropy H(X)');
xlabel('Kernel parameter $\sigma$','Interpreter','latex');
x_labels = [];
for i=1:size(kernel_param_buf,2)
    x_labels = [x_labels,floor(kernel_param_buf(i))];
end
x_labels = [(floor(kernel_param_buf(1))):(floor(kernel_param_buf(size(kernel_param_buf,2)))-floor(kernel_param_buf(1)))/10:(floor(kernel_param_buf(size(kernel_param_buf,2))))]
set(gca,'XTickLabel',x_labels);
% tightfig;
% squeeze_axes(handles)