function K = genKmtx(x,kernel_type,kernel_param)

[ndata, ndim] = size(x);

xr = zeros(ndata,ndata,ndim);
xc = zeros(ndata,ndata,ndim);

for i=1:ndim
    xr(:,:,i) = repmat(x(:,i),[1,ndata]);
    xc(:,:,i) = repmat(x(:,i)',[ndata,1]);
end

K = Kmtx_eval(xr,xc,kernel_type,kernel_param);