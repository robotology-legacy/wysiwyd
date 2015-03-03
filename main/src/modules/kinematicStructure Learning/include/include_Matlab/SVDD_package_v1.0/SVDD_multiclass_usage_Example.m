%**************************** Multiclass SVDD Main ************************
%   Multi class Support Vector Data Description(SVDD) Main for Blind Spot Detection
%       
%   Hyung jin Chang 06/30/2008
%   hjchang@neuro.snu.ac.kr
%**************************************************************************

clc;
% clear all;
close all;
%------------------ Data loading --------------------
x1 = randn(50,3);
idx_x1 = ones(50,1);
x2 = randn(50,3)+4;
idx_x2 = ones(50,1)*2;
x3 = randn(50,3)-4;
idx_x3 = ones(50,1)*3;

tr_data = [x1;x2;x3];
tr_data_idx = [idx_x1;idx_x2;idx_x3];

%---------------- Variable Setting ------------------
class_idx = unique(tr_data_idx);
if class_idx(1) == 0
    tr_data_idx = tr_data_idx+1;
end
nclass = size(class_idx,1);
C = cell(nclass,1);
kern_param = cell(nclass,1);
kernel = cell(nclass,1);

% C{1} = 1;
% C{2} = 1;
% C{3} = 1;
% 
% kern_param{1} = 0.07;
% kern_param{2} = 0.07;
% kern_param{3} = 0.07;
% 
% kernel{1} = 'gaussian';
% kernel{2} = 'gaussian';
% kernel{3} = 'gaussian';

for i=1:nclass
    C{i} = 1;
    kern_param{i} = 0.7;
    kernel{i} = 'gaussian';
end
%----------------------------------------------------
data = [tr_data(:,:),tr_data_idx(:)];
num_class = size(unique(data(:,end)),1);
% dim = size(mda_data,2);

class = cell(num_class,1);

for i=1:num_class
    class{i} = struct('data',[],...
                      'class_idx',[],...
                      'alpha',[],...
                      'SV',[],...
                      'nSV',[],...
                      'a',[],...
                      'R2',[],...
                      'C',[],...
                      'kernel',[],...
                      'kern_param',[]);
    class{i}.data = data(find(data(:,end) == i),[1:end-1]);
    class{i}.idx = i;
    class{i}.C = C{i};
    class{i}.kernel = kernel{i};
    class{i}.kern_param = kern_param{i};    

%     tic
%     %===================== BATCH TYPE SVDD TRAINING ==================
%     %------------------ Training SVDD -------------------
%     [class{i}.alpha, class{i}.SV, class{i}.nSV] = SVDDtrain(class{i}.data, class{i}.C, class{i}.kernel, class{i}.kern_param);
%     %   alpha: Lagrange multiplier
%     %   SV: Support Vectors
%     %   nSV: number of Support Vector
%     %----------------------------------------------------
%     %------------ Calculating the boundary --------------
%     class{i}.R2 = boundary(class{i}.data,class{i}.SV,class{i}.alpha,class{i}.kernel,class{i}.kern_param,'batch');
%     %----------------------------------------------------
%     class{i}.a = class{i}.alpha'*class{i}.data;
%     %------------- SVDD Boundary Drawing ----------------
%     SVDDdrawing(class{i}.data, class{i}.SV, class{i}.alpha, class{i}.R2, class{i}.kernel, class{i}.kern_param, class{i}.idx,'batch');
%     %----------------------------------------------------
%     
%     time_batch = toc
    tic
    %================== INCREMENTAL TYPE SVDD TRAINING ==================    
    %------------------ Training SVDD -------------------
    [class{i}.alpha, class{i}.SV, class{i}.nSV] = incSVDDtrain(class{i}.data, class{i}.C, class{i}.kernel, class{i}.kern_param);        
    %   alpha: Lagrange multiplier
    %   SV: Support Vectors
    %   nSV: number of Support Vector
    %----------------------------------------------------
    %------------ Calculating the boundary --------------
    class{i}.R2 = boundary(class{i}.data,class{i}.SV,class{i}.alpha,class{i}.kernel,class{i}.kern_param, 'incremental');    
    %----------------------------------------------------
    class{i}.a = class{i}.alpha'*class{i}.SV;
    %------------- SVDD Boundary Drawing ----------------
    SVDDdrawing(class{i}.data, class{i}.SV, class{i}.alpha, class{i}.R2, class{i}.kernel, class{i}.kern_param, class{i}.idx, 'incremental');
    %----------------------------------------------------    
    time_inc = toc
    pause(50)
end