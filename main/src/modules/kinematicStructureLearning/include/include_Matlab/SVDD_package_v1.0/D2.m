function result = D2(class,test_data,kernel,kern_param)

num_data = size(class.data,1);

term1 = Kernel_Function(test_data,test_data,kernel,kern_param);
term2 = 0;
term3 = 0;

for i=1:num_data
    term2 = term2 + class.alpha(i)*Kernel_Function(test_data,class.data(i,:),kernel,kern_param);
    
    for j=1:num_data
        term3 = term3 + class.alpha(i)*class.alpha(j)*Kernel_Function(class.data(i,:),class.data(j,:),kernel,kern_param);        
    end
end

result = term1-2*term2+term3;
