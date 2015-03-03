function b = cal_b(x,y,alpha,SV,C,kernel,kern_param)

[ndata,ndim] = size(x);

fbuf = 0;
ind_SV = find(alpha > eps & alpha <=C);

for n=1:size(ind_SV,1)    
    for i=1:ndata
        fbuf = fbuf + alpha(i)*y(i)*Kernel_Function(x(i,:),x(ind_SV(n),:),kernel,kern_param);
    end
    
    b(n) = y(ind_SV(n))-fbuf;
    fbuf=0;
end
b = round(b*10000)/10000;
unique_b = unique(b);
number_of_unique_b = size(unique_b,2);

b = mean(b,2);

if number_of_unique_b ~= 1
    fprintf('WARNING!! all of the b is not same!!\n');
    fprintf('  Im suggesting you to change the kernel parameter!!\n');
    fprintf('  returning average of b values: %d',b);
else number_of_unique_b == 1
    fprintf('You got only one b value: %d',b);
end