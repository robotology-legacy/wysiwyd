function rotMtx = calRotMatx(KineStruct, fstep)

frm_idx_list = [1 : fstep : KineStruct.num_frames-fstep];

rotMtx = cell(length(frm_idx_list), KineStruct.num_seg); 

for seg_idx = 1:KineStruct.num_seg
    for frm_idx = 1:length(frm_idx_list)
        x_k = KineStruct.y(:,KineStruct.seg_idx{seg_idx},frm_idx_list(frm_idx));
        x_l = KineStruct.y(:,KineStruct.seg_idx{seg_idx},frm_idx_list(frm_idx)+fstep);
%% by fundamental matrix
        [F_buf,e1,e2] = fundmatrix(x_k,x_l);
        [rot, t] = EssentialMatrixToCameraMatrix(F_buf);
        R = rot(:,:,1);

%% by eigen vector
%%
        rotMtx{frm_idx, seg_idx} = R;
    end
end