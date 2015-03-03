function dist = calD(z,zy,SET,kernel_param,kernel_type)

[z_ndata,z_ndim] = size(z);
dist = zeros(z_ndata,1);

x = [SET.S.x ; SET.E.x ; SET.O.x];
y = [SET.S.y ; SET.E.y ; SET.O.y];
alpha = [SET.S.alpha ; SET.E.alpha ; SET.O.alpha];
alpha_buf = repmat(alpha,[1,z_ndata]);

Sidx = [1:SET.S.ndata]';
Eidx = [SET.S.ndata+1:SET.S.ndata+SET.E.ndata]';
Oidx = [SET.S.ndata+SET.E.ndata+1:SET.S.ndata+SET.E.ndata+SET.O.ndata]';

Lij = alpha_buf' * genQmtx(x,y,kernel_type,kernel_param) * alpha_buf;

Mij = alpha_buf' * genQc_calD(x,y,z,zy,kernel_type,kernel_param)';
dist = ones(z_ndata,1) - 2*diag(Mij) + diag(Lij);
