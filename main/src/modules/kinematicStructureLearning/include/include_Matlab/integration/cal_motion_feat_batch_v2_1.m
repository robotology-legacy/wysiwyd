function feat_out = cal_motion_feat_batch_v2_1(KineStruct, i, j, k, preCompMtx, method)

% % feat_out = zeros(6,1);
% 
% dist_buf = [preCompMtx{i,j,1}.cal_m_F, preCompMtx{j,k,1}.cal_m_F, preCompMtx{k,i,1}.cal_m_F];
% 
% %%
% min_value = min(dist_buf,[],1);
% max_value = max(dist_buf,[],1);
% 
% % feat_out(1) = min_value(1);
% % feat_out(2) = max_value(1);
% % feat_out(3) = min_value(2);
% % feat_out(4) = max_value(2);
% % feat_out(5) = min_value(3);
% % feat_out(6) = max_value(3);
% feat_out = [min_value,max_value];

% feat_out = zeros(6,1);

%----------------------------------------------------------
feat_out = [preCompMtx{i,j,1}.min,preCompMtx{i,j,1}.max,preCompMtx{j,k,1}.min,preCompMtx{j,k,1}.max,preCompMtx{k,i,1}.min,preCompMtx{k,i,1}.max];
end