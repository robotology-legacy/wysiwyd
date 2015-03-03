function Q = genQc_vec(x,y,xc,yc,kernel_type,kernel_param)

[n_sv_data, n_sv_dim] = size(x);
[n_test_data, n_test_dim] = size(xc);

x = repmat(x,[1,n_test_data]);

test_data_buf = reshape(xc',[1,n_test_data*n_test_dim]);
xc_vec = repmat(test_data_buf,[n_sv_data,1]);
yc_vec = repmat(yc,[n_sv_data,n_test_data]);

K = Kc_eval_vec(x,xc_vec,kernel_type,kernel_param);
% Q = y.* yc_vec.* K;
Q = K;