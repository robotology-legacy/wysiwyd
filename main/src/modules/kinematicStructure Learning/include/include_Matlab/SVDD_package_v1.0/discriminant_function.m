function resultC = discriminant_function(test_data,x,y,alpha,b,kernel,kern_param)

[ndata,ndim] = size(x);
fbuf=0;

for i=1:ndata
    fbuf = fbuf + alpha(i)*y(i)*Kernel_Function(x(i,:),test_data,kernel,kern_param);
end

f = fbuf+b;

if f>0
    resultC = 1;
else
    resultC = -1;
end
