function result = Rm_z(class,test_data,kernel,kern_param)

% num_data = size(class.data,1);

% term1 = Kernel_Function(test_data,test_data,kernel,kern_param);
term1 = 1;
term2 = 0;
term3 = 0;

non_zero_idx = find(class.alpha ~= 0);

for i=1:class.nSV
    term2 = term2 + ...
        class.alpha(non_zero_idx(i)) *...
        Kernel_Function(test_data,class.data(non_zero_idx(i),:),kernel,kern_param);
    
    for j=1:class.nSV
        term3 = term3 + ...
            class.alpha(non_zero_idx(i))*class.alpha(non_zero_idx(j))*...
            Kernel_Function(class.data(non_zero_idx(i),:),class.data(non_zero_idx(j),:),kernel,kern_param);        
    end
end

result = term1-2*term2+term3;
