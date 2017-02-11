function Q = genQc(x,y,xc,yc,kernel_type,kernel_param)

[ndata, ndim] = size(x);

xc_vec = repmat(xc,[ndata,1]);
yc_vec = repmat(yc,[ndata,1]);

K = Kc_eval(x,xc_vec,kernel_type,kernel_param);
Q = y.* yc.* K;