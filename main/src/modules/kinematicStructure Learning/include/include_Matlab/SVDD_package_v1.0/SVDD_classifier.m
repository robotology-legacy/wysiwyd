%**************************** SVDD classifier *****************************
%   Support Vector Data Description(SVDD) classifier for Blind Spot Detection
%
%   Hyung jin Chang 06/10/2008
%   hjchang@neuro.snu.ac.kr
%**************************************************************************

clc
% close 100

%---------------- Variable Setting ------------------
C = 0.9;                           % C value of SVDD
kern_param = 1;                    % kernel parameter
kernel = 'gaussian';               % kernel types
                                   %  - 'gaussian'
                                   %  - 'linear'
                                   %  - 'polynomian'
                                   %  - 'sigmoid'
                                   %  - 'erbf'
%----------------------------------------------------
% input_data = load('./data/doughnut_set.mat');
% x = input_data.banana_set;
% idx = ones(size(x,1),1);
% data = [x,idx];

data = [mda_data(:,:),OF_idx(:)];
num_class = size(unique(data(:,end)),1);
dim = size(mda_data,2);

class = cell(num_class,1);

for i=1:num_class
    class{i} = struct('data',[],...
                      'class_idx',[],...
                      'alpha',[],...
                      'SV',[],...
                      'nSV',[],...
                      'a',[],...
                      'R2',[],...
                      'kernel',[],...
                      'kern_param',[]);
    class{i}.data = data(find(data(:,end) == i),[1:end-1]);
    class{i}.idx = i;

    %------------------ Training SVDD -------------------
    [class{i}.alpha, class{i}.SV, class{i}.nSV] = SVDDtrain(class{i}.data, C, kernel, kern_param);
    %   alpha: Lagrange multiplier
    %   SV: Support Vectors
    %   nSV: number of Support Vector
    %----------------------------------------------------
    %------------ Calculating the boundary --------------
    class{i}.R2 = boundary(class{i}.data,class{i}.SV,class{i}.alpha,kernel,kern_param);
    %----------------------------------------------------
    class{i}.a = class{i}.alpha'*class{i}.data;
    %------------- SVDD Boundary Drawing ----------------
    SVDDdrawing(class{i}.data,class{i}.SV,class{i}.alpha,class{i}.R2,kernel,kern_param,class{i}.idx);
    %----------------------------------------------------
end

test_data = [-2,1];
prob_ts_data = zeros(num_class,1);

for j=1:num_class
    Rm_z(class{j},test_data,kernel,kern_param)
    prob_ts_data(j) = (1/(2*pi*class{j}.R2/dim)^(dim/2))*exp(-1*Rm_z(class{j},test_data,kernel,kern_param)/(2*class{j}.R2/dim));
end
[Prob Dclass] = max(prob_ts_data);
Dclass
