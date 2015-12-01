function K = Kernel_Function(xi,xj,kernel,kern_param)

%=============== SVDD Kernel Function =================
% [Input]
%   xi,xj: training data
%   kernel: kernel type
%   kern_param: kernel parameter
% [Output]
%   K: calculated result
%
% Hyung jin Chang 06/13/2007
% hjchang@neuro.snu.ac.kr
%======================================================

switch kernel
    case 'gaussian'
        K = exp(-(xi-xj)*(xi-xj)'/(kern_param^2));
    case 'linear'
        K = xi*xj';
    case 'polynomial'
        K = (xi*xj'+1)^(kern_param);
    case 'sigmoid'
        K = tanh(xi*xj'-kern_param);
    case 'erbf'
        K = exp(-sqrt((xi-xj)*(xi-xj)'/(kern_param^2)));
    otherwise
end