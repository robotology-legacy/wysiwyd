function K = Kmtx_eval_calD(x,z,kernel,kern_param)

%=============== SVDD Kernel Function =================
% [Input]
%   xr,z: training data
%   kernel: kernel type
%   kern_param: kernel parameter
% [Output]
%   K: calculated result
%
% Hyung jin Chang 06/13/2007
% hjchang@neuro.snu.ac.kr
%======================================================

[z_ndata, ndata, ndim] = size(x);

x_buf = zeros(z_ndata,ndata,ndim);
x_norm = zeros(z_ndata,ndata);

switch kernel
    case 'gaussian'
        for i=1:ndim
            x_buf(:,:,i) = (x(:,:,i)-z(:,:,i)).^2;
            x_norm = x_norm + x_buf(:,:,i);
        end
        K = exp(-x_norm/(kern_param^2));
        
    case 'linear'
        x_buf1 = x.^2;
        x_buf2 = zeros(ndata,ndata);
        for i=1:ndim
            x_buf2 = x_buf2 + x_buf(:,:,i);
        end
        K = sum(x_buf2.^2,ndim);
        
    case 'polynomial'
%         x_buf1 = x.^2;
%         x_buf2 = zeros(ndata,ndata);
%         for i=1:ndim
%             x_buf2 = x_buf2 + x_buf(:,:,i);
%         end
%         K = (sum(x_buf2.^2,ndim)+1)^(kern_param);
        K = (sum(x.*z,3)+1).^(kern_param);    
        
    case 'sigmoid'
%         x_buf1 = x.^2;
%         x_buf2 = zeros(ndata,ndata);
%         for i=1:ndim
%             x_buf2 = x_buf2 + x_buf(:,:,i);
%         end
%         K = tanh(sum(x_buf2.^2,ndim)-kern_param);
        K = tanh(sum(x.*z,3)-kern_param);
        
    case 'erbf'
%         for i=1:ndim
%             x_buf(:,:,i) = (x(:,:,i)-z(:,:,i)).^2;
%             x_norm = x_norm + x_buf(:,:,i);
%         end
%         K = exp(-sqrt(x_norm/(kern_param^2)));
        for i=1:ndim
            x_buf(:,:,i) = abs(x(:,:,i)-z(:,:,i)).^2;
            x_norm = x_norm + x_buf(:,:,i);
        end
        K = exp(-sqrt(x_norm/(kern_param^2)));
        
    otherwise
end