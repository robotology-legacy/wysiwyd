function out = preComputedMtx_v2(KineStruct, method)
nNode = KineStruct.num_seg;
nFrm = KineStruct.num_frames;
jnt = KineStruct.joint_center;

ang_buf = zeros(nFrm,2);
dist_buf = zeros(nFrm,2);

out = cell(nNode, nNode, nFrm);

switch(method)
    case 'angle'
        %         out_struct = struct('vi_jnt_ij',[],'vj_jnt_ij',[],'vj_jnt_jk',[],'vk_jnt_jk',[],'vk_jnt_ki',[],'vi_jnt_ki',[],'ang_ij',[],'ang_jk',[],'ang_ki',[]);
        for i=1:nNode
            for j=1:nNode
                if i~=j
                    ang_buf = zeros(nFrm,1);
                    parfor frm_idx = 1:nFrm
                        out_struct = struct('vi_jnt_ij',[],'vj_jnt_ij',[],'ang_ij',[]);
                        % angle between i-j
                        out_struct.vi_jnt_ij = KineStruct.seg_center(:,i,frm_idx) - jnt{i,j}(:,frm_idx);
                        out_struct.vj_jnt_ij = KineStruct.seg_center(:,j,frm_idx) - jnt{i,j}(:,frm_idx);
                        
                        out_struct.ang_ij = mod( atan2( det([out_struct.vi_jnt_ij,out_struct.vj_jnt_ij]) , dot(out_struct.vi_jnt_ij,out_struct.vj_jnt_ij) ) , 2*pi );
                        ang_buf(frm_idx) = out_struct.ang_ij;
                        
                        out{i,j,frm_idx} = out_struct;
                    end
                    out{i,j,1}.ang_buf = ang_buf;
                end
            end
        end
        
        
    case 'log'
        %         out_struct = struct('vi_jnt_ij',[],'vj_jnt_ij',[],'vj_jnt_jk',[],'vk_jnt_jk',[],'vk_jnt_ki',[],'vi_jnt_ki',[],'ang_i_ij',[],'ang_j_ij',[],'ang_j_jk',[],'ang_k_jk',[],'ang_k_ki',[],'ang_i_ki',[],'rotz_i_ij',[],'rotz_j_ij',[],'rotz_j_jk',[],'rotz_k_jk',[],'rotz_k_ki',[],'rotz_i_ki',[]);
        for i=1:nNode
            for j=1:nNode
                if i~=j
                    cal_m_buf = zeros(nFrm,1);
                    %                     parfor frm_idx = 1:nFrm
                    for frm_idx = 1:nFrm
                        out_struct = struct('vi_jnt_ij',[],'vj_jnt_ij',[],'ang_i_ij',[],'ang_j_ij',[],'rotz_i_ij',[],'rotz_j_ij',[],'cal_m',[]);
                        % angle between i-j
                        out_struct.vi_jnt_ij = KineStruct.seg_center(:,i,frm_idx) - jnt{i,j}(:,frm_idx);
                        out_struct.vj_jnt_ij = KineStruct.seg_center(:,j,frm_idx) - jnt{i,j}(:,frm_idx);
                        %%
                        out_struct.ang_i_ij = mod( atan2( det([out_struct.vi_jnt_ij,[1;0]]) , dot(out_struct.vi_jnt_ij,[1;0]) ) , 2*pi );
                        out_struct.ang_j_ij = mod( atan2( det([out_struct.vj_jnt_ij,[1;0]]) , dot(out_struct.vj_jnt_ij,[1;0]) ) , 2*pi );
                        
                        out_struct.rotz_i_ij = rotz(out_struct.ang_i_ij);
                        out_struct.rotz_j_ij = rotz(out_struct.ang_j_ij);
                        
                        out_struct.cal_m = cal_logm(out_struct.rotz_i_ij, out_struct.rotz_j_ij);
                        cal_m_buf(frm_idx) = out_struct.cal_m;
                        
                        out{i,j,frm_idx} = out_struct;
                    end
                    out{i,j,1}.cal_m_F = cal_m_buf;
                    out{i,j,1}.min = min(cal_m_buf);
                    out{i,j,1}.max = max(cal_m_buf);
                end
            end
        end
end
end