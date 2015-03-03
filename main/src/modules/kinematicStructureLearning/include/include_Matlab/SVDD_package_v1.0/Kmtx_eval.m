function K = Kmtx_eval(xr,xc,kernel,kern_param)

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

[ndata, ndata, ndim] = size(xr);

x_buf = zeros(ndata,ndata,ndim);
x_norm = zeros(ndata,ndata);

switch kernel
    case 'gaussian'
        for i=1:ndim
            x_buf(:,:,i) = (xr(:,:,i)-xc(:,:,i)).^2;
            x_norm = x_norm + x_buf(:,:,i);
        end
        K = exp(-x_norm/(kern_param^2));
        
    case 'linear'
        x_buf1 = xr.^2;
        x_buf2 = zeros(ndata,ndata);
        for i=1:ndim
            x_buf2 = x_buf2 + x_buf(:,:,i);
        end
        K = sum(x_buf2.^2,ndim);
        
    case 'polynomial'
%         x_buf = xr;
%         x_buf2 = zeros(ndata,ndata);
%         for i=1:ndim
%             x_buf2 = x_buf2 + x_buf(:,:,i);
%         end
%         K = (sum(x_buf2.^2,ndim)+1)^(kern_param);
        K = (sum(xr.*xc,3)+1).^(kern_param);
        
    case 'sigmoid'
%         x_buf1 = xr.^2;
%         x_buf2 = zeros(ndata,ndata);
%         for i=1:ndim
%             x_buf2 = x_buf2 + x_buf(:,:,i);
%         end
%         K = tanh(sum(x_buf2.^2,ndim)-kern_param);
        K = tanh(sum(xr.*xc,3)-kern_param);
        
    case 'erbf'
        for i=1:ndim
            x_buf(:,:,i) = abs(xr(:,:,i)-xc(:,:,i)).^2;
            x_norm = x_norm + x_buf(:,:,i);
        end
        K = exp(-sqrt(x_norm/(kern_param^2)));
        
    otherwise
end