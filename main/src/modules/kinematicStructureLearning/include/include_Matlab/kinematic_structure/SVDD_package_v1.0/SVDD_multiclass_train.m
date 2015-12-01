function [class] = SVDD_multiclass_train(tr_data,tr_data_idx,C,kern_param,kernel,train_type)

%**************************** SVDD classifier *****************************
%   Support Vector Data Description(SVDD) classifier for Blind Spot Detection
%
%   Hyung jin Chang 06/10/2008
%   hjchang@neuro.snu.ac.kr
%**************************************************************************

%---------------- Variable Setting ------------------
% C = 0.9;                           % C value of SVDD
% kern_param = 1;                    % kernel parameter
% kernel = 'gaussian';               % kernel types
%  - 'gaussian'
%  - 'linear'
%  - 'polynomian'
%  - 'sigmoid'
%  - 'erbf'
%----------------------------------------------------

data = [tr_data(:,:),tr_data_idx(:)];
num_class = size(unique(data(:,end)),1);
% dim = size(mda_data,2);

class = cell(num_class,1);

for i=1:num_class
    class{i} = struct('data',[],...
        'idx',[],...
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

    switch train_type
        case {'batch'}
            %------------------ Training SVDD -------------------
            [class{i}.alpha, class{i}.SV, class{i}.nSV] = SVDDtrain(class{i}.data, class{i}.C, class{i}.kernel, class{i}.kern_param);
            %   alpha: Lagrange multiplier
            %   SV: Support Vectors
            %   nSV: number of Support Vector
            %----------------------------------------------------
            %------------ Calculating the boundary --------------
            class{i}.R2 = boundary(class{i}.data,class{i}.SV,class{i}.alpha,class{i}.kernel,class{i}.kern_param,'batch');
            %----------------------------------------------------
            class{i}.a = class{i}.alpha'*class{i}.data;
            %------------- SVDD Boundary Drawing ----------------
            SVDDdrawing(class{i}.data, class{i}.SV, class{i}.alpha, class{i}.R2, class{i}.kernel, class{i}.kern_param, class{i}.idx, 'batch');
        case {'incremental'}
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
        otherwise
            error(sprintf('Unknown proximity type: %s',kernel))
    end

end
    % test_data = [-2,1];
    % prob_ts_data = zeros(num_class,1);
    %
    % for j=1:num_class
    %     Rm_z(class{j},test_data,kernel,kern_param)
    %     prob_ts_data(j) = (1/(2*pi*class{j}.R2/dim)^(dim/2))*exp(-1*Rm_z(class{j},test_data,kernel,kern_param)/(2*class{j}.R2/dim));
    % end
    % [Prob Dclass] = max(prob_ts_data);
    % Dclass
