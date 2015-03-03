function K = cal_kernel(xi,xj,parameter)

K = exp(-norm(xi-xj)^2/(parameter^2));