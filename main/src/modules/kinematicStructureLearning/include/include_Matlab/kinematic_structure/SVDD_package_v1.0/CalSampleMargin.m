function sample_margin = CalSampleMargin(z,zy,SET,kernel_param,kernel_type)

[z_ndata,z_ndim] = size(z);
sample_margin = zeros(z_ndata,1);

x = [SET.S.x ; SET.E.x];
y = [SET.S.y ; SET.E.y];
alpha = [SET.S.alpha ; SET.E.alpha];
alpha_buf = repmat(alpha,[1,z_ndata]);

a_norm = sqrt(alpha' * genQmtx(x,y,kernel_type,kernel_param) * alpha);
a_phi_x = alpha' * genQc_calD(x,y,z,zy,kernel_type,kernel_param)';

sample_margin = a_phi_x/a_norm;

