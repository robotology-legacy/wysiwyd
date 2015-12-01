% close all
% clear all
% clc

addpath /home/hjchang/Research/code/Matlab/SVDD_package_v1.0/
addpath /home/hjchang/Research/code/Matlab/motion_segmentation/dataset/
addpath /home/hjchang/Research/code/Matlab/Skeleton/Skeleton
addpath /home/hjchang/Research/code/Matlab/motion_segmentation/randomized_voting
%%
%------------------ Data loading --------------------
% load arm_truth.mat
% load head_truth.mat
% load two_cranes_truth.mat
% load toy.mat
% width = 364; height = 244;

load YanPollefeys.mat
width = 718; height = 480;

% load dance_seg_data.mat

% load articulated_truth.mat

% addpath /home/hjchang/Research/dataset/motion_segmentation/Hopkins155WithVideosPart4/giraffe
% conversion_to_y;
% width = 180; height = 120;
% load giraffe_motion_seg_data.mat

% load iCub_hand.mat

%-----------------------------------------------------
%%
[y_dim,y_num,y_frm] = size(y);

x_org_full = zeros(y_num,2,y_frm);

for f = 1:y_frm
    for n = 1:y_num
        x_org_full(n,1,f) = y(1,n,f);
        x_org_full(n,2,f) = height - y(2,n,f);
        %         x_org_full(n,2,f) = y(2,n,f);
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

x = [];
y = [];

sm_mean = [];
sm_var = [];
sm_skewness = [];
sm_entropy = [];

tic
% kernel_param_buf = 10:5:100;
kernel_max = width / 5;
kernel_min = width / 50;
kernel_step = width / 100;

% kernel_param_buf = 10:5:100;
kernel_param_buf = kernel_min:kernel_step:kernel_max;
% kernel_param_buf = 1:1:10;

num_kernel_param = size(kernel_param_buf,2);

param_counter = 0;

for kernel_param = kernel_param_buf
    %     kernel_param
    clc;
    param_counter = param_counter+1;
    disp('Searching for an optimal kernal parameter');
    buf = sprintf('%d %% completed.',ceil(param_counter/num_kernel_param*100));
    disp(buf);
    switch learning_type
        case 'batch'
            x = x_org;
            y = y_org;
            SET = incSVDD(x,y,C,kernel_type,kernel_param,0);
            
            sample_margin = CalSampleMargin(SET.O.x,SET.O.y,SET,kernel_param,kernel_type);  % calculate the sample margin
            sm_max = max(sample_margin);
            sm_min = min(sample_margin);
            sm_mean = [sm_mean,mean(sample_margin)];
            sm_var = [sm_var,var(sample_margin)];
            sm_skewness = [sm_skewness,skewness(sample_margin)];
            sm_entropy = [sm_entropy,entropy(sample_margin)];
            
        case 'incremental'
            for incdata_idx=1:ndata
                disp(['incdata_idx = ',num2str(incdata_idx)]);
                
                num_data = 20;
                
                if incdata_idx < num_data
                    x = [x;x_org(incdata_idx,:)];
                    y = [y;y_org(incdata_idx)];
                elseif incdata_idx == num_data
                    SET = incSVDD(x,y,C,kernel_type,kernel_param,0);
                    incSVDD_drawing(x,y,SET,kernel_param,kernel_type,time_delay);
                else
                    SET = incSVDD(x_org(incdata_idx,:),y_org(incdata_idx,:),C,kernel_type,kernel_param,1,SET);
                    incSVDD_drawing(x,y,SET,kernel_param,kernel_type,time_delay);
                end
            end
    end
end
training_time = toc
% incSVDD_drawing(x,y,SET,kernel_param,kernel_type,time_delay);

figure(99)
subplot(3,1,1);
plot(sm_var)
subplot(3,1,2);
plot(sm_skewness)
subplot(3,1,3);
plot(sm_entropy)

% disp('Variance');
% [value,optimal_kern_param_var] = max(sm_var);
% kernel_param = kernel_param_buf(optimal_kern_param_var)
% SET = incSVDD(x,y,C,kernel_type,kernel_param,0);
% % incSVDD_drawing(x,y,SET,kernel_param,kernel_type,time_delay);
%
% pause(1)

disp('Skewness');
[value, optimal_kern_param_skewness] = min(abs(sm_skewness));
kernel_param = floor(kernel_param_buf(optimal_kern_param_skewness))
SET = incSVDD(x,y,C,kernel_type,kernel_param,0);
% incSVDD_drawing(x,y,SET,kernel_param,kernel_type,time_delay);

% pause(1)

disp('Entropy');
[value, optimal_kern_param_entropy] = max(sm_entropy);
kernel_param = floor(kernel_param_buf(optimal_kern_param_entropy))-20
SET = incSVDD(x,y,C,kernel_type,kernel_param,0);
% incSVDD_drawing(x,y,SET,kernel_param,kernel_type,time_delay);


%%
color_idx = 'rgbcmykrbgcmykrgbcmyk';
num_seg = size(unique(final),1);
seg_idx = cell(num_seg,1);

for i=1:num_seg
    seg_idx{i} = find(final == i);
end

mean_y = zeros(num_seg,2);


%%
% writerobj = VideoWriter('result_video.avi');
% open(writerobj);

h = figure(1);

%%
for frm_idx = 1:y_frm
    clf
    x = x_org_full(:,:,frm_idx);
    SET = incSVDD(x,y,C,kernel_type,kernel_param,0);
    
    if SET.S.ndata ~= 0
        incSVDD_drawing(x,y,SET,kernel_param,kernel_type,time_delay);
        %%
        % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % image binarisation
        tic
        width_buf = repmat([1:width]',[1,height]);
        width_buf2 = reshape(width_buf',[width*height,1]);
        height_buf = [1:height]';
        height_buf2 = repmat(height_buf,[width,1]);
        test_pixel = [width_buf2,height_buf2];
        
        test_result = testSVDD(test_pixel, SET, kernel_param, kernel_type);
        bw_img = uint8(test_result < 0)*255;
        bw_img = reshape(bw_img, [height,width]);
        
        test_time = toc;
        [frm_idx, test_time]
        
        img = flipud(bw_img);
        
        
        subplot(2,3,1)
        for p=1:y_num
            title('Input Feature Points');
            plot(x(p,1), x(p,2),'k.');
            axis([0, width, 0, height]);
            axis equal
            hold on
        end
        
        subplot(2,3,4)
        imshow(uint8(img));
        title('Shape Binarization');
        
        
        
        
        %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % the standard skeletonization:
        % figure(2)
        %         subplot(2,3,2)
        %         imshow(bwmorph(img,'skel',inf));
        
        % the new method:
        % figure(3)
        %         subplot(2,3,3)
        %         imshow(bwmorph(skeleton(img)>35,'skel',Inf));
        
        % in more detail:
        [skr,rad] = skeleton(img);
        
        % the intensity at each point is proportional to the degree of evidence
        % that this should be a point on the skeleton:
        % figure(4)
        %         subplot(2,3,4)
        %         imagesc(skr);
        %         colormap jet
        %         axis image off
        %         freezeColors
        
        % skeleton can also return a map of the radius of the largest circle that
        % fits within the foreground at each point:
        % figure(5)
        subplot(2,3,5)
        imagesc(rad)
        title('Probability of topological skeleton');
        colormap jet
        axis image off
        freezeColors
        
        % thresholding the skeleton can return skeletons of thickness 2,
        % so the call to bwmorph completes the thinning to single-pixel width.
        skel = bwmorph(skr > 35,'skel',inf);
        % figure(6)
        subplot(2,3,6)
        imshow(skel)
        title('Topological Skeleton from Shape');
        % try different thresholds besides 35 to see the effects
        
        % anaskel returns the locations of endpoints and junction points
        [dmap,exy,jxy] = anaskel(skel);
        hold on
        plot(exy(1,:),exy(2,:),'go')
        plot(jxy(1,:),jxy(2,:),'ro')
        
        %%
        %-----------------------------------------------------------
        % Motion segmentation
        for p=1:y_num
            subplot(2,3,3)
            title('Motion segmentation');
%             plot(x(p,1), x(p,2),'.','Color',color_idx(final(p)));
            plot(x(p,1), x(p,2),'.','Color','b');
            axis([0, width, 0, height]);
            hold on
        end
        
%         for i=1:num_seg
%             %         mean_y(i,1) = median(y(1,seg_idx{i},f));
%             %         mean_y(i,2) = median(y(2,seg_idx{i},f));
%             mean_y(i,1) = mean(x(seg_idx{i},1));
%             mean_y(i,2) = mean(x(seg_idx{i},2));
%         end
%         subplot(2,3,3)
%         plot(mean_y(:,1),mean_y(:,2),'ko', 'MarkerSize',7);
        
        
        %         plot([mean_y(1,1),mean_y(11,1)], [mean_y(1,2),mean_y(11,2)],'k-');
        %         plot([mean_y(11,1),mean_y(3,1)], [mean_y(11,2),mean_y(3,2)],'k-');
        %         %
        %         plot([mean_y(3,1),mean_y(4,1)], [mean_y(3,2),mean_y(4,2)],'k-');
        %         plot([mean_y(3,1),mean_y(10,1)], [mean_y(3,2),mean_y(10,2)],'k-');
        %
        %         plot([mean_y(11,1),mean_y(7,1)], [mean_y(11,2),mean_y(7,2)],'k-');
        %         plot([mean_y(11,1),mean_y(9,1)], [mean_y(11,2),mean_y(9,2)],'k-');
        %         %
        %         plot([mean_y(4,1),mean_y(6,1)], [mean_y(4,2),mean_y(6,2)],'k-');
        %         plot([mean_y(9,1),mean_y(8,1)], [mean_y(9,2),mean_y(8,2)],'k-');
        %         plot([mean_y(10,1),mean_y(5,1)], [mean_y(10,2),mean_y(5,2)],'k-');
        %         plot([mean_y(7,1),mean_y(2,1)], [mean_y(7,2),mean_y(2,2)],'k-');
        
        
        
%         F = getframe(h);
%         writeVideo(writerobj,F);
        pause()
    end
end

% close(writerobj);