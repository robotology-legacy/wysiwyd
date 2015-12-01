%%
[y_dim,num_points,num_frames] = size(y);

x_org_full = zeros(num_points,2,num_frames);

for f = 1:num_frames
    for n = 1:num_points
        x_org_full(n,1,f) = y(1,n,f);
        x_org_full(n,2,f) = height - y(2,n,f);
    end
end

sm_entropy_frames = [];

for frm_idx = 1:10:num_frames
    frm_idx
    dim = 2;
    x_org = x_org_full(:,:,frm_idx);
    
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
    
    % tic
    % kernel_max = width / 2;
    % kernel_min = width / 50;
    % kernel_step = width / 100;
    
    kernel_max = width / 2;
    kernel_min = width / 50;
    kernel_step = width / 100;
    
%     kernel_max = width;
%     kernel_min = width / 100;
%     kernel_step = width / 200;
    
    
    kernel_param_buf = kernel_min:kernel_step:kernel_max;
    
    num_kernel_param = size(kernel_param_buf,2);
    
    param_counter = 0;
    
    %%
    % Sample margin
    sample_margin_buf = cell(num_kernel_param,1);
    idx = 1;
    sm_entropy = [];
    sm_min = [];
    sm_max = [];
    
    % Landgrebe
    TPr_buf = [];
    FPr_buf = [];
    
    tic
    for kernel_param = kernel_param_buf
        %     kernel_param
%         clc;
%         param_counter = param_counter+1;
%         disp('Searching for an optimal kernal parameter');
%         buf = sprintf('%d %% completed.',ceil(param_counter/num_kernel_param*100));
%         disp(buf);
        switch learning_type
            case 'batch'
                x_SVDD = x_org;
                y_SVDD = y_org;
                SET = incSVDD(x_SVDD,y_SVDD,C,kernel_type,kernel_param,0);
                
                sample_margin = CalSampleMargin(SET.O.x,SET.O.y,SET,kernel_param,kernel_type);  % calculate the sample margin
                sample_margin_buf{idx} = sample_margin;
                idx = idx + 1;
                sm_max = [sm_max,max(sample_margin)];
                sm_min = [sm_min,min(sample_margin)];
                sm_entropy = [sm_entropy,entropy(sample_margin)];
                
%                 SVDD_param_selection_Landgrebe;
%                 TPr_buf = [TPr_buf, TPr];
%                 FPr_buf = [FPr_buf, FPr];
                
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
    
    disp('Entropy');
    [value, optimal_kern_param_entropy] = max(sm_entropy);
    kernel_param = floor(kernel_param_buf(optimal_kern_param_entropy))
    SET = incSVDD(x_SVDD,y_SVDD,C,kernel_type,kernel_param,0);
    % incSVDD_drawing(x_SVDD,y_SVDD,SET,kernel_param,kernel_type,time_delay);    
    
    sm_entropy_frames = [sm_entropy_frames ; sm_entropy];
end
%%
% result drawing
Figure1 = figure(1);
set(Figure1,'defaulttextinterpreter','latex');
plot(kernel_param_buf,sm_entropy)
xlabel('kernel parameter ($\sigma$)');
ylabel('entropy');
axis([kernel_param_buf(1),kernel_param_buf(end),0,max(sm_entropy)+10]);


% Figure2 = figure(2);
% set(Figure2,'defaulttextinterpreter','latex');
% plot(kernel_param_buf,sm_max,'r-');
% xlabel('kernel parameter ($\sigma$)');
% hold on
% plot(kernel_param_buf,sm_min,'b-');
% xlabel('kernel parameter ($\sigma$)');
% ylabel('sample margin value');
% axis([kernel_param_buf(1),kernel_param_buf(end),0,1]);
% 
% Figure3 = figure(3);
% set(Figure3,'defaulttextinterpreter','latex');
% subplot(1,3,1)
% plot(sample_margin_buf{1},'b.')
% axis([0,length(sample_margin_buf{1}), min(sample_margin_buf{1}), max(sample_margin_buf{1})]);
% ylabel('sample margin value');
% 
% subplot(1,3,2)
% plot(sample_margin_buf{optimal_kern_param_entropy},'b.')
% axis([0,length(sample_margin_buf{optimal_kern_param_entropy}), min(sample_margin_buf{optimal_kern_param_entropy}), max(sample_margin_buf{optimal_kern_param_entropy})]);
% ylabel('sample margin value');
% 
% subplot(1,3,3)
% plot(sample_margin_buf{num_kernel_param},'b.')
% axis([0,length(sample_margin_buf{num_kernel_param}), min(sample_margin_buf{num_kernel_param}), max(sample_margin_buf{num_kernel_param})]);
% ylabel('sample margin value');

%%
Figure4 = figure(4);
plot(FPr_buf, TPr_buf, 'r-');
xlabel('false positive rate (FPr)');
ylabel('true positive rate (TPr)');
axis([0,1,0,1]);

%%
% Figure for Baxter
FIG5 = figure(5);
set(FIG5,'defaulttextinterpreter','latex');
surf(sm_entropy_frames(1:10:end,:));
xlabel('kernel parameter ($\sigma$)');
ylabel('frame')
zlabel('entropy');
set(gca,'XTickLabel',[0:68.3:273.3]);
set(gca,'YTickLabel',[0:90:450]);

save_path = '/home/hjchang/Dropbox/paper_writing/[2015][Journal][TPAMI] Kinematic Structure Learning/paper/figs/fig_kernel_param_select';
saveas(Figure5, [save_path, '/entropy_frame_Baxter.pdf'], 'pdf')

% Figure for human hand
FIG5 = figure(5);
set(FIG5,'defaulttextinterpreter','latex');
surf(sm_entropy_frames);
xlabel('kernel parameter ($\sigma$)');
ylabel('frame')
zlabel('entropy');
axis([0,49,0,64,0,150]);
set(gca,'XTickLabel',[0:85.4:427]);
set(gca,'YTickLabel',[0:210:630]);

save_path = '/home/hjchang/Dropbox/paper_writing/[2015][Journal][TPAMI] Kinematic Structure Learning/paper/figs/fig_kernel_param_select';
saveas(Figure5, [save_path, '/entropy_frame_humanhand.pdf'], 'pdf')


