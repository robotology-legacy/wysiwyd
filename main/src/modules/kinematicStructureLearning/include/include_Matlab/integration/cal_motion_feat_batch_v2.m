function feat_out = cal_motion_feat_batch_v2(KineStruct, i, j, k, preCompMtx, method)

nFrm = KineStruct.num_frames;

% ang_buf = zeros(nFrm,3);
% dist_buf = zeros(nFrm,3);

% norm_value = cal_logm(rotz(0), rotz(pi)) * 2;
norm_value = 9.1514;

feat_out = zeros(6,1);

switch(method)
    case 'angle'
%         parfor frm_idx = 1:nFrm
%             ang_buf(frm_idx,:) = [preCompMtx{i,j,k,frm_idx}.ang_ij, preCompMtx{i,j,k,frm_idx}.ang_jk, preCompMtx{i,j,k,frm_idx}.ang_ki];
%         end
        ang_buf = [preCompMtx{i,j,1}.ang_buf, preCompMtx{j,k,1}.ang_buf, preCompMtx{k,i,1}.ang_buf] / (2*pi);
        %%
        min_value = min(ang_buf,[],1);
        max_value = max(ang_buf,[],1);
        feat_out = reshape([min_value;max_value], [1,6])';
        
    case 'log'

        dist_buf = [preCompMtx{i,j,1}.cal_m_F, preCompMtx{j,k,1}.cal_m_F, preCompMtx{k,i,1}.cal_m_F] ./ norm_value;    
        
        %%
        min_value = min(dist_buf,[],1);
        max_value = max(dist_buf,[],1);
        
%         feat_out = reshape([min_value;max_value], [1,6])';
        feat_out(1) = min_value(1);
        feat_out(2) = max_value(1);
        feat_out(3) = min_value(2);
        feat_out(4) = max_value(2);
        feat_out(5) = min_value(3);
        feat_out(6) = max_value(3);
end
end