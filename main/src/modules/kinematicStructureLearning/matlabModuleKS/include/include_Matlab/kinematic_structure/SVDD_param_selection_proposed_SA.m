% time_total = [];
% param_total = [];
% 
% for niter = 1:10
% 
%     time_total
%     param_total
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

%%
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
%%
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


% kernel_param_buf = kernel_min:kernel_step:kernel_max;
%
% num_kernel_param = size(kernel_param_buf,2);
%
% param_counter = 0;

%%
% Sample margin
% sample_margin_buf = cell(num_kernel_param,1);
idx = 1;
sm_entropy = [];
sm_min = [];
sm_max = [];

% time buf
train_time_buf = [];
Landgrebe_gen_outlier_time_buf = [];
selection_time_buf = [];
test_time_buf = [];
Landgrebe_overall_time_buf = [];
proposed_overall_time_buf = [];

%%
% Simulated Annealing
disp(sprintf('\n'));
disp('================================================================');
disp('Searching for an optimal kernal parameter by Simulated Annealing');
disp('================================================================');

% Start location
% kernel_param_start = width / 10;

kernel_param_step = width / 10;
kernel_param_max = width / 4;
kernel_param_min = width / 100;

kernel_param_start = (kernel_param_max - kernel_param_min) * rand() + kernel_param_min;

x_SVDD = x_org;
y_SVDD = y_org;

time_SA_select = tic;

%% Simulated Annealing
% Number of cycles
n = 10;
% Number of trials per cycle
m = 10;
% Number of accepted solutions
na = 0.0;
% Probability of accepting worse solution at the start
p_start = 0.7;
% Probability of accepting worse solution at the end
p_end = 0.001;
% Initial temperature
t_start = -1.0/log(p_start);
% Final temperature
t_end = -1.0/log(p_end);
% Fractional reduction every cycle
frac = (t_end/t_start)^(1.0/(n-1.0));
% Initialize x
kernel_param_values = zeros(n,1);
kernel_param_values(1,:) = kernel_param_start;
kernel_param_i = kernel_param_start;
na = na + 1.0;
% Current best results so far
kernel_param_c = kernel_param_values(1,:);
%--------------------------------------------------------
kernel_param = kernel_param_i;
SET = incSVDD(x_SVDD,y_SVDD,C,kernel_type,kernel_param,0);
sample_margin = CalSampleMargin(SET.O.x,SET.O.y,SET,kernel_param,kernel_type);  % calculate the sample margin
sm_entropy = entropy(sample_margin);
fc = sm_entropy;
%--------------------------------------------------------
fs = zeros(n,1);
fs(1,:) = fc;
% Current temperature
t = t_start;
% DeltaE Average
DeltaE_avg = 1;

%%
for i=1:n
    disp(['Cycle: ',num2str(i),' with Temperature: ',num2str(t)])
    for j=1:m
        % Generate new trial points
%         kernel_param_i = kernel_param_c + round(kernel_param_step * 2 * (rand() - 0.5));
        kernel_param_i = kernel_param_c + kernel_param_step * 2 * (rand()-0.5);
        % Clip to upper and lower bounds
        kernel_param_i = max(min(kernel_param_i,kernel_param_max),kernel_param_min);
        %--------------------------------------------------------
        kernel_param = kernel_param_i;
        SET = incSVDD(x_SVDD,y_SVDD,C,kernel_type,kernel_param,0);
        sample_margin = CalSampleMargin(SET.O.x,SET.O.y,SET,kernel_param,kernel_type);  % calculate the sample margin
        sm_entropy = entropy(sample_margin);
        %--------------------------------------------------------
        
        DeltaE = abs(sm_entropy-fc);
        
        if (sm_entropy<fc)
            %             % Initialize DeltaE_avg if a worse solution was found
            %             %   on the first iteration
            if (i==1 && j==1)
                DeltaE_avg = DeltaE;
            end
            % objective function is worse
            % generate probability of acceptance
            p = exp(-DeltaE/(DeltaE_avg * t));
            %             % determine whether to accept worse point
            if (rand()<p)
                % accept the worse solution
                accept = true;
            else
                % don't accept the worse solution
                accept = false;
            end
        else
            % objective function is lower, automatically accept
            accept = true;
        end
        if (accept==true)
            % update currently accepted solution
            kernel_param_c = kernel_param_i;
            
            fc = sm_entropy;
            % increment number of accepted solutions
            na = na + 1.0;
            % update DeltaE_avg
            DeltaE_avg = (DeltaE_avg * (na-1.0) +  DeltaE) / na;
        end
    end
    % Record the best x values at the end of every cycle
    kernel_param_values(i+1,1) = kernel_param_c;
    fs(i+1) = fc;
    % Lower the temperature for next cycle
    t = frac * t;
end

kernel_param = kernel_param_c;

% time_SAtime = toc(time_SA_select);
% print solution
disp(['Best solution: ',num2str(kernel_param_c)])
% disp(['Best objective: ',num2str(fc)])
% plot(kernel_param_values(:,1),kernel_param_values(:,2),'r-o')
% 
% time_total = [time_total,time_SAtime];
% param_total = [param_total, kernel_param_c];

%%
% fig = figure(2);
% clf(fig)
% subplot(2,1,1)
% plot(fs,'r.-')
% legend('entropy')
% subplot(2,1,2)
% hold on
% plot(kernel_param_values(:,1),'b.-')
% legend('kernel parameter')

% end

% [mean(time_total), std(time_total)]
% [mean(param_total), std(param_total)]

% figure
% incSVDD_drawing(x_SVDD,y_SVDD,SET,kernel_param,kernel_type,time_delay);
