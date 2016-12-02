%%

time_total = [];
param_total = [];

for niter = 1:1
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
    % Landgrebe
    TPr_buf = [];
    FPr_buf = [];
    
    train_time_buf = [];
    Landgrebe_gen_outlier_time_buf = [];
    selection_time_buf = [];
    test_time_buf = [];
    Landgrebe_overall_time_buf = [];
    proposed_overall_time_buf = [];
    
    for kernel_param = kernel_param_buf
        clc;
        disp(['outside: ',num2str(kernel_param)]);
        
        param_counter = param_counter+1;
        disp(['Iteration: ', num2str(niter)]);
        disp('Searching for an optimal kernal parameter using Landgrebe method!');
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
        
        %%
        SVDD_param_selection_Landgrebe;
        TPr_buf = [TPr_buf, TPr];
        FPr_buf = [FPr_buf, FPr];
        
        Landgrebe_overall_time_buf = [Landgrebe_overall_time_buf,(train_lapsed + Landgrebe_gen_outlier_lapsed + test_lapsed)];
    end
    % training_time = toc
    % incSVDD_drawing(x,y,SET,kernel_param,kernel_type,time_delay);
    
    Landgrebe_overall_time = sum(Landgrebe_overall_time_buf);
    time_total = [time_total, Landgrebe_overall_time]
    
    %%
    % result drawing
    % Figure1 = figure(1);
    % set(Figure1,'defaulttextinterpreter','latex');
    % plot(kernel_param_buf,sm_entropy_total)
    % xlabel('kernel parameter ($\sigma$)');
    % ylabel('entropy');
    % axis([kernel_param_buf(1),kernel_param_buf(end),0,max(sm_entropy_total)+10]);
    
    
    %%
    % Figure4 = figure(4);
    % set(Figure4,'defaulttextinterpreter','latex', 'Position', [200,200,500,500]);
    % clf
    % % FPr_buf = sort(FPr_buf);
    % % TPr_buf = sort(TPr_buf);
    % % plot(sort(FPr_buf), sort(TPr_buf), 'r-');
    % hold on
    % grid on
    % plot(FPr_buf(1:10), TPr_buf(1:10), 'r-');
    % % plot(FPr_buf, TPr_buf, 'ro');
    %
    % % for i=1:num_kernel_param
    % %     text(FPr_buf(i)-0.01, TPr_buf(i)+0.01, num2str(kernel_param_buf(i)), 'Color', 'k');
    % % end
    %
    % % h_ann = annotation('textarrow',[FPr_buf(2)+0.25, FPr_buf(2)+0.125],[TPr_buf(2)-0.25,TPr_buf(2)-0.055],...
    % %            'String',['$\sigma$ = ', num2str(kernel_param_buf(2))])
    % %
    % % set(h_ann, 'fontsize', 30);
    % % set(gca,'fontsize', 13);
    % axis square
    % axis([0,1,0,1]);
    % xlabel('false positive rate on artificial outliers ($FPr^{a.o.}$)');
    % ylabel('true positive rate ($TPr$)');
    %
    % % save_path = '/home/hjchang/Dropbox/paper_writing/[2015][Journal][TPAMI] Kinematic Structure Learning/paper/figs/fig_kernel_param_select';
    % % saveas(gcf, [save_path, '/ROC.pdf'], 'pdf')
    
    %%
    L = 0.5 * (1-TPr_buf) + 0.5 * FPr_buf;
    
%     figure(10)
%     plot((1-TPr_buf));
%     figure(11)
%     plot(FPr_buf);

    diff = L(1:end-1) - L(2:end);
    
    flag = 1;
    select_idx = -1;
%     for i = 1:length(kernel_param_buf)-1
%         if diff(i) < 0 && flag == 1
%             select_idx = i+1;
%             flag = 0;
%         end
%     end
    
    if select_idx == -1
        [min_Y, select_idx] = min(L(1:end-1));
    end
    
    selected_param = kernel_param_buf(select_idx);
    
    param_total = [param_total, selected_param]
    
    %%
    Figure5 = figure(5);
    clf
    hold on
    grid on
    set(Figure5,'defaulttextinterpreter','latex', 'Position', [100,100,500,500]);
    plot(kernel_param_buf,L,'r-');
    plot(kernel_param_buf,L,'ro');
    for i=1:num_kernel_param
        text(kernel_param_buf(i)-0.01, L(i)+0.01, num2str(kernel_param_buf(i)), 'Color', 'k');
    end
    % h_ann = annotation('textarrow',[0.4, 0.32],[0.4,0.25],...
    %            'String',['$\sigma$ = ', num2str(kernel_param_buf(2))])
    xlabel('kernel parameter ($\sigma$)');
    ylabel('loss function value');
    % axis([kernel_param_buf(1),1000,0,1]);
    % axis square
    
    % set(h_ann, 'fontsize', 30);
    
    % save_path = '/home/hjchang/Dropbox/paper_writing/[2015][Journal][TPAMI] Kinematic Structure Learning/paper/figs/fig_kernel_param_select';
    % saveas(Figure5, [save_path, '/loss.pdf'], 'pdf')
    
    pause(0.5);
    
    %%
    Figure4 = figure(4);
    set(Figure4,'defaulttextinterpreter','latex', 'Position', [200,200,500,500]);
    clf
    % FPr_buf = sort(FPr_buf);
    % TPr_buf = sort(TPr_buf);
    % plot(sort(FPr_buf), sort(TPr_buf), 'r-');
    hold on
    grid on
    plot(FPr_buf, TPr_buf, 'r-');
    % plot(FPr_buf, TPr_buf, 'ro');
    
    % for i=1:num_kernel_param
    %     text(FPr_buf(i)-0.01, TPr_buf(i)+0.01, num2str(kernel_param_buf(i)), 'Color', 'k');
    % end
    
%     h_ann = annotation('textarrow',[FPr_buf(2)+0.25, FPr_buf(2)+0.125],[TPr_buf(2)-0.25,TPr_buf(2)-0.055],...
%         'String',['$\sigma$ = ', num2str(kernel_param_buf(2))])
%     
%     set(h_ann, 'fontsize', 30);
    % set(gca,'fontsize', 13);
%     axis square
    axis([0,1,0,1]);
    xlabel('false positive rate on artificial outliers ($FPr^{a.o.}$)');
    ylabel('true positive rate ($TPr$)');
    
end

[mean(time_total), std(time_total)]
[mean(param_total), std(param_total)]

incSVDD_drawing(x_SVDD,y_SVDD,SET,kernel_param,kernel_type,time_delay);