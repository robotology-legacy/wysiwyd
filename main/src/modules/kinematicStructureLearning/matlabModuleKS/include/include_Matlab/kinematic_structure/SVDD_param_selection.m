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

% tic
kernel_max = width / 2;
kernel_min = width / 50;
kernel_step = width / 100;

kernel_param_buf = kernel_min:kernel_step:kernel_max;

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
            x_SVDD = x_org;
            y_SVDD = y_org;
            SET = incSVDD(x_SVDD,y_SVDD,C,kernel_type,kernel_param,0);
            
            sample_margin = CalSampleMargin(SET.O.x,SET.O.y,SET,kernel_param,kernel_type);  % calculate the sample margin
            sm_max = max(sample_margin);
            sm_min = min(sample_margin);
            sm_entropy = [sm_entropy,entropy(sample_margin)];
            
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
% training_time = toc
% incSVDD_drawing(x,y,SET,kernel_param,kernel_type,time_delay);

% figure(99)
% plot(sm_entropy)

disp('Entropy');
[value, optimal_kern_param_entropy] = max(sm_entropy);
kernel_param = floor(kernel_param_buf(optimal_kern_param_entropy))
SET = incSVDD(x_SVDD,y_SVDD,C,kernel_type,kernel_param,0);
% incSVDD_drawing(x_SVDD,y_SVDD,SET,kernel_param,kernel_type,time_delay);
