function K = Kc_eval(x,xc,kernel,kern_param)

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

[ndata, ndim] = size(x);

x_norm = zeros(ndata,1);

switch kernel
    case 'gaussian'
        x_norm = sum((x-xc).^2,2);
        K = exp(-x_norm/(kern_param^2));

    case 'linear'
        K = sum(x.*xc,2);

    case 'polynomial'
        K = (sum(x.*xc,2)+1).^(kern_param);

    case 'sigmoid'
        K = tanh(sum(x.*xc,2)-kern_param);

    case 'erbf'
        x_norm = sum(abs(x-xc).^2,2);
        K = exp(-sqrt(x_norm/(kern_param^2)));
%         disp(['    inside: ',num2str(kern_param)]);

    otherwise
end