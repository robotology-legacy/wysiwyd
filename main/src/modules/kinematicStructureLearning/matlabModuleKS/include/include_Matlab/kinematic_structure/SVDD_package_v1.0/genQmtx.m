function Q = genQmtx(x,y,kernel_type,kernel_param)

[ndata, ndim] = size(x);

xr = zeros(ndata,ndata,ndim);
xc = zeros(ndata,ndata,ndim);
yr = repmat(y,[1,ndata]);
yc = repmat(y',[ndata,1]);

for i=1:ndim
    xr(:,:,i) = repmat(x(:,i),[1,ndata]);
    xc(:,:,i) = repmat(x(:,i)',[ndata,1]);
end

K = Kmtx_eval(xr,xc,kernel_type,kernel_param);
Q = yr.* yc.* K;