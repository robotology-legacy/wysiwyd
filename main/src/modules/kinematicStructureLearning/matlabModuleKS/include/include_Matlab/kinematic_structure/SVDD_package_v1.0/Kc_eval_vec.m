function K = Kc_eval_vec(x,xc,kernel,kern_param)

%=============== SVDD Kernel Function =================
% [Input]
%   xr,xc: training data
%   kernel: kernel type
%   kern_param: kernel parameter
% [Output]
%   K: calculated result
%
% Hyung jin Chang 06/13/2007
% hjchang@neuro.snu.ac.kr
%======================================================

% [ndata, ndim] = size(x);

% x_norm = zeros(ndata,1);

switch kernel
    case 'gaussian'
        x_norm_buf = (x-xc).^2;
        x_norm = x_norm_buf(:,1:2:end) + x_norm_buf(:,2:2:end);        
        K = exp(-x_norm/(kern_param^2));

%     case 'linear'
%         K = sum(x.*xc_vec,2);
% 
    case 'polynomial'
        x_norm_buf = x.*xc;
        x_norm = x_norm_buf(:,1:2:end) + x_norm_buf(:,2:2:end);          
        K = (x_norm+1).^(kern_param);
% 
%     case 'sigmoid'
%         K = tanh(sum(x.*xc_vec,2)-kern_param);
% 
    case 'erbf'
%         x_norm = sum((x-xc).^2,2);
        x_norm_buf = (x-xc).^2;
        x_norm = x_norm_buf(:,1:2:end) + x_norm_buf(:,2:2:end);          
        K = exp(-sqrt(x_norm/(kern_param^2)));

    otherwise
end