function out = preComputedMtx_v4(KineStruct, method)
nNode = KineStruct.num_seg;
nFrm = KineStruct.num_frames;
jnt = KineStruct.joint_center;

% intermediate_result = cell(nNode, nNode, nFrm);
out = cell(nNode, nNode, nFrm);

switch(method)
    case 'angle'
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
                        
                        intermediate_result{i,j,frm_idx} = out_struct;
                    end
                    intermediate_result{i,j,1}.ang_buf = ang_buf;
                end
            end
        end       
        
    case 'log'
    %% Relative angle based method
        for i=1:nNode
            for j=1:nNode
                if i~=j
                    cal_m_buf = zeros(nFrm,1);
                    parfor frm_idx = 1:nFrm
                        out_struct = struct('vi_jnt_ij',[],'vj_jnt_ij',[],'ang_ij',[],'rotm_z',[]);
                        % angle between i-j
                        out_struct.vi_jnt_ij = KineStruct.seg_center(:,i,frm_idx) - jnt{i,j}(:,frm_idx);
                        out_struct.vj_jnt_ij = KineStruct.seg_center(:,j,frm_idx) - jnt{i,j}(:,frm_idx);
                        %%
                        out_struct.ang_ij = atan2( det([out_struct.vi_jnt_ij,out_struct.vj_jnt_ij]) , dot(out_struct.vi_jnt_ij,out_struct.vj_jnt_ij) );

                        out_struct.rotm_z = rotz(out_struct.ang_ij);
                        
%                         out_struct.cal_m = cal_logm(out_struct.rotz_i_ij, out_struct.rotz_j_ij);
%                         cal_m_buf(frm_idx) = out_struct.cal_m;
                        
                        intermediate_result{i,j,frm_idx} = out_struct;
                    end
                end
            end            
        end
        
        for i=1:nNode
            for j=1:nNode
                for k=1:nNode
                    if (i ~= j) && (j ~= k) && (k ~= i)
                        cal_m_buf = zeros(nFrm,1);
                        parfor frm_idx = 1:nFrm                        
                            cal_m_buf(frm_idx) = cal_logm(intermediate_result{i,j,frm_idx}.rotm_z, intermediate_result{i,k,frm_idx}.rotm_z);                           
                        end                        
                        out{i,j,k}.cal_m_F = cal_m_buf;
                        
                    end
                end
            end
        end
        
end
end