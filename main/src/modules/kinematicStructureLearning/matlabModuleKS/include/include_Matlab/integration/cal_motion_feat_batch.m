function feat_out = cal_motion_feat_batch(KineStruct, i, j, k, preCompMtx, method)

nFrm = KineStruct.num_frames;

ang_buf = zeros(nFrm,3);
dist_buf = zeros(nFrm,3);

switch(method)
    case 'angle'
        parfor frm_idx = 1:nFrm
            ang_buf(frm_idx,:) = [preCompMtx{i,j,k,frm_idx}.ang_ij, preCompMtx{i,j,k,frm_idx}.ang_jk, preCompMtx{i,j,k,frm_idx}.ang_ki];
        end
        %%
        min_value = min(ang_buf,[],1);
        max_value = max(ang_buf,[],1);
        feat_out = reshape([min_value;max_value], [1,6])';
        
    case 'log'
        parfor frm_idx = 1:nFrm
            pair_buf = [cal_logm(preCompMtx{i,j,k,frm_idx}.rotz_i_ij, preCompMtx{i,j,k,frm_idx}.rotz_j_ij),...
                        cal_logm(preCompMtx{i,j,k,frm_idx}.rotz_j_jk, preCompMtx{i,j,k,frm_idx}.rotz_k_jk),...
                        cal_logm(preCompMtx{i,j,k,frm_idx}.rotz_k_ki, preCompMtx{i,j,k,frm_idx}.rotz_i_ki)] ./ cal_logm(rotz(0), rotz(pi));
            dist_buf(frm_idx,:) = pair_buf;
        end
        
        %%
        min_value = min(dist_buf,[],1);
        max_value = max(dist_buf,[],1);
        feat_out = reshape([min_value;max_value], [1,6])';
end
end