function SVDD_ts_result = SVDD_multiclass_classifier(ts_data,tr_data,tr_data_idx,SVDD_tr_result)

num_class = size(unique(tr_data_idx),1);
dim = size(SVDD_tr_result{1}.data,2);
prob_ts_data = zeros(num_class,1);

for j=1:num_class
%     Rm_z(SVDD_tr_result{j},ts_data,SVDD_tr_result{j}.kernel,SVDD_tr_result{j}.kern_param);
    prob_ts_data(j) = (1/(2*pi*SVDD_tr_result{j}.R2/dim)^(dim/2))*...
        exp(-1*Rm_z(SVDD_tr_result{j},ts_data,SVDD_tr_result{j}.kernel,SVDD_tr_result{j}.kern_param)/(2*SVDD_tr_result{j}.R2/dim));
end
prob_ts_data;
[Prob SVDD_ts_result] = max(prob_ts_data);
