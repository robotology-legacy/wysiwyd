%%

time_total = [];
param_total = [];

for niter = 1:10
    
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
    
    % tic
    % kernel_max = width / 2;
    % kernel_min = width / 50;
    % kernel_step = width / 100;
    
    % kernel_max = width / 3;
    % kernel_min = width / 50;
    % kernel_step = width / 50;
    
    % kernel_max = width;
    % kernel_min = width / 100;
    % kernel_step = width / 200;
    
    kernel_max = width / 4;
    kernel_min = width / 100;
    kernel_step = width / 200;
    
    
    kernel_param_buf = kernel_min:kernel_step:kernel_max;
    % kernel_param_buf = kernel_max:-kernel_step:kernel_min;
    
    num_kernel_param = size(kernel_param_buf,2);
    
    param_counter = 0;
    
    %%
    % Sample margin
    sample_margin_buf = cell(num_kernel_param,1);
    idx = 1;
    sm_skewness_total = [];
    sm_min = [];
    sm_max = [];
    
    % Landgrebe
    TPr_buf = [];
    FPr_buf = [];
    
    train_time_buf = [];
    Landgrebe_gen_outlier_time_buf = [];
    selection_time_buf = [];
    test_time_buf = [];
    Landgrebe_overall_time_buf = [];
    Kim_overall_time_buf = [];
    
    for kernel_param = kernel_param_buf
        clc;
        disp(['outside: ',num2str(kernel_param)]);
        
        param_counter = param_counter+1;
        disp('Searching for an optimal kernal parameter');
        buf = sprintf('%d %% completed.',ceil(param_counter/num_kernel_param*100));
        disp(buf);
        
        time_total
        param_total
        
        x_SVDD = x_org;
        y_SVDD = y_org;
        
        %%
        train_start = tic;
        SET = incSVDD(x_SVDD,y_SVDD,C,kernel_type,kernel_param,0);
        train_lapsed = toc(train_start);
        train_time_buf = [train_time_buf, train_lapsed];
        
        selection_start = tic;
        sample_margin = CalSampleMargin(SET.O.x,SET.O.y,SET,kernel_param,kernel_type);  % calculate the sample margin
        sm_entropy_buf = skewness(sample_margin);
        selection_lapsed = toc(selection_start);
        selection_time_buf = [selection_time_buf, selection_lapsed];
        
        sample_margin_buf{idx} = sample_margin;
        idx = idx + 1;
        sm_max = [sm_max,max(sample_margin)];
        sm_min = [sm_min,min(sample_margin)];
        sm_skewness_total = [sm_skewness_total,sm_entropy_buf];
        
        Kim_overall_time_buf = [Kim_overall_time_buf,(train_lapsed + selection_lapsed)];
    end
    % training_time = toc
    % incSVDD_drawing(x,y,SET,kernel_param,kernel_type,time_delay);
    
    Kim_overall_time = sum(Kim_overall_time_buf);
    time_total = [time_total, Kim_overall_time]
    
    disp('Entropy');
    [value, select_idx] = min(abs(sm_skewness_total));
    kernel_param = floor(kernel_param_buf(select_idx));
    % SET = incSVDD(x_SVDD,y_SVDD,C,kernel_type,kernel_param,0);
    % incSVDD_drawing(x_SVDD,y_SVDD,SET,kernel_param,kernel_type,time_delay);
    selected_param = kernel_param_buf(select_idx);
    param_total = [param_total, selected_param]
    
    %%
    % result drawing
    Figure1 = figure(1);
    set(Figure1,'defaulttextinterpreter','latex');
    plot(kernel_param_buf,sm_skewness_total)
    xlabel('kernel parameter ($\sigma$)');
    ylabel('entropy');
    axis([kernel_param_buf(1),kernel_param_buf(end),0,max(sm_skewness_total)+10]);
    
    % %%
    % Figure2 = figure(2);
    % set(Figure2,'defaulttextinterpreter','latex');
    % plot(kernel_param_buf,sm_max,'r-');
    % xlabel('kernel parameter ($\sigma$)');
    % hold on
    % plot(kernel_param_buf,sm_min,'b-');
    % xlabel('kernel parameter ($\sigma$)');
    % ylabel('sample margin value');
    % axis([kernel_param_buf(1),kernel_param_buf(end),0,1]);
    
    %%
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
    
end

[mean(time_total), std(time_total)]
[mean(param_total), std(param_total)]

incSVDD_drawing(x_SVDD,y_SVDD,SET,kernel_param,kernel_type,time_delay);
