%**************************** SVDD Main *****************************
%   Incremental Support Vector Data Description(SVDD) Main
%
%   Hyung jin Chang
%   hjchang@neuro.snu.ac.kr
%********************************************************************

clc;
clear all;
close all;

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

%------------------ Data loading --------------------
% input_data = load('./data/doughnut_set.mat');
% x_org = input_data.banana_set;

% x_org = load('./data/x_t.dat');

% x_org = randn(1000,2)/4*[2 0.5; 0.5 1];
x_org = randn(200,2)/4*[2 0.7; 0.7 1]*3;
x_org = [x_org;randn(100,2)/4*[1 -0.7; 0.7 2]+2]*3;
x_org = [x_org;randn(100,2)/4*[1 -0.7; 0.7 1]+repmat([-1 3], 100, 1)]*3;

x_org = unique(x_org,'rows');
ndata = size(x_org,1);
y_org = ones(ndata,1);
%----------------------------------------------------
%---------------- Variable Setting ------------------
C = 1;                             % C value of SVDD
kernel_param = 0.5;                  % kernel parameter
kernel_type = 'gaussian';          % kernel types
%  - 'gaussian'
%  - 'linear'
%  - 'polynomian'
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
%%%%%%%%%%%%%%%%%%%%%%%% Learning Ω√¿€ %%%%%%%%%%%%%%%%%%%%%%%%%%%%
num_class = size(unique(y_org),1);
x = [];
y = [];

sm_mean = [];
sm_var = [];
sm_skewness = [];
sm_entropy = [];

tic
for kernel_param = 0.1:0.5:30
    kernel_param
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
%     incSVDD_drawing(x,y,SET,kernel_param,kernel_type,time_delay);
end
time = toc
% incSVDD_drawing(x,y,SET,kernel_param,kernel_type,time_delay);
figure(1)
subplot(3,1,1);
plot(sm_var)
subplot(3,1,2);
plot(sm_skewness)
subplot(3,1,3);
plot(sm_entropy)


[value,optimal_kern_param_var] = max(sm_var);
kernel_param = optimal_kern_param_var * 0.5
SET = incSVDD(x,y,C,kernel_type,kernel_param,0);
incSVDD_drawing(x,y,SET,kernel_param,kernel_type,time_delay);

pause(1)

[value, optimal_kern_param_skewness] = min(abs(sm_skewness));
kernel_param = optimal_kern_param_skewness * 0.5
SET = incSVDD(x,y,C,kernel_type,kernel_param,0);
incSVDD_drawing(x,y,SET,kernel_param,kernel_type,time_delay);

pause(1)

[value, optimal_kern_param_entropy] = max(sm_entropy);
kernel_param = optimal_kern_param_entropy * 0.5
SET = incSVDD(x,y,C,kernel_type,kernel_param,0);
incSVDD_drawing(x,y,SET,kernel_param,kernel_type,time_delay);
