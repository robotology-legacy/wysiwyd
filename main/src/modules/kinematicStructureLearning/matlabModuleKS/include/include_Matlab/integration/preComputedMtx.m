function out = preComputedMtx(KineStruct, method)
nNode = KineStruct.num_seg;
nFrm = KineStruct.num_frames;
jnt = KineStruct.joint_center;

ang_buf = zeros(nFrm,3);
dist_buf = zeros(nFrm,3);

out = cell(nNode, nNode, nNode, nFrm);

switch(method)
    case 'angle'
        %         out_struct = struct('vi_jnt_ij',[],'vj_jnt_ij',[],'vj_jnt_jk',[],'vk_jnt_jk',[],'vk_jnt_ki',[],'vi_jnt_ki',[],'ang_ij',[],'ang_jk',[],'ang_ki',[]);
        for i=1:nNode
            for j=1:nNode
                for k=1:nNode
                    if i~=j && j~=k && k~=i
                        parfor frm_idx = 1:nFrm
                            out_struct = struct('vi_jnt_ij',[],'vj_jnt_ij',[],'vj_jnt_jk',[],'vk_jnt_jk',[],'vk_jnt_ki',[],'vi_jnt_ki',[],'ang_ij',[],'ang_jk',[],'ang_ki',[]);
                            % angle between i-j
                            out_struct.vi_jnt_ij = KineStruct.seg_center(:,i,frm_idx) - jnt{i,j}(:,frm_idx);
                            out_struct.vj_jnt_ij = KineStruct.seg_center(:,j,frm_idx) - jnt{i,j}(:,frm_idx);
                            
                            % angle between j-k
                            out_struct.vj_jnt_jk = KineStruct.seg_center(:,j,frm_idx) - jnt{j,k}(:,frm_idx);
                            out_struct.vk_jnt_jk = KineStruct.seg_center(:,k,frm_idx) - jnt{j,k}(:,frm_idx);
                            
                            % angle between k-i
                            out_struct.vk_jnt_ki = KineStruct.seg_center(:,k,frm_idx) - jnt{k,i}(:,frm_idx);
                            out_struct.vi_jnt_ki = KineStruct.seg_center(:,i,frm_idx) - jnt{k,i}(:,frm_idx);
                            
                            out_struct.ang_ij = mod( atan2( det([out_struct.vi_jnt_ij,out_struct.vj_jnt_ij]) , dot(out_struct.vi_jnt_ij,out_struct.vj_jnt_ij) ) , 2*pi );
                            out_struct.ang_jk = mod( atan2( det([out_struct.vj_jnt_jk,out_struct.vk_jnt_jk]) , dot(out_struct.vj_jnt_jk,out_struct.vk_jnt_jk) ) , 2*pi );
                            out_struct.ang_ki = mod( atan2( det([out_struct.vk_jnt_ki,out_struct.vi_jnt_ki]) , dot(out_struct.vk_jnt_ki,out_struct.vi_jnt_ki) ) , 2*pi );
                            
                            out{i,j,k,frm_idx} = out_struct;
                        end
                        %%
                    end
                end
            end
        end
        
    case 'log'
        %         out_struct = struct('vi_jnt_ij',[],'vj_jnt_ij',[],'vj_jnt_jk',[],'vk_jnt_jk',[],'vk_jnt_ki',[],'vi_jnt_ki',[],'ang_i_ij',[],'ang_j_ij',[],'ang_j_jk',[],'ang_k_jk',[],'ang_k_ki',[],'ang_i_ki',[],'rotz_i_ij',[],'rotz_j_ij',[],'rotz_j_jk',[],'rotz_k_jk',[],'rotz_k_ki',[],'rotz_i_ki',[]);
        for i=1:nNode
            for j=1:nNode
                for k=1:nNode
                    if i~=j && j~=k && k~=i
                        parfor frm_idx = 1:nFrm
                            out_struct = struct('vi_jnt_ij',[],'vj_jnt_ij',[],'vj_jnt_jk',[],'vk_jnt_jk',[],'vk_jnt_ki',[],'vi_jnt_ki',[],'ang_i_ij',[],'ang_j_ij',[],'ang_j_jk',[],'ang_k_jk',[],'ang_k_ki',[],'ang_i_ki',[],'rotz_i_ij',[],'rotz_j_ij',[],'rotz_j_jk',[],'rotz_k_jk',[],'rotz_k_ki',[],'rotz_i_ki',[]);
                            % angle between i-j
                            out_struct.vi_jnt_ij = KineStruct.seg_center(:,i,frm_idx) - jnt{i,j}(:,frm_idx);
                            out_struct.vj_jnt_ij = KineStruct.seg_center(:,j,frm_idx) - jnt{i,j}(:,frm_idx);
                            
                            % angle between j-k
                            out_struct.vj_jnt_jk = KineStruct.seg_center(:,j,frm_idx) - jnt{j,k}(:,frm_idx);
                            out_struct.vk_jnt_jk = KineStruct.seg_center(:,k,frm_idx) - jnt{j,k}(:,frm_idx);
                            
                            % angle between k-i
                            out_struct.vk_jnt_ki = KineStruct.seg_center(:,k,frm_idx) - jnt{k,i}(:,frm_idx);
                            out_struct.vi_jnt_ki = KineStruct.seg_center(:,i,frm_idx) - jnt{k,i}(:,frm_idx);
                            
                            %%
                            out_struct.ang_i_ij = mod( atan2( det([out_struct.vi_jnt_ij,[1;0]]) , dot(out_struct.vi_jnt_ij,[1;0]) ) , 2*pi );
                            out_struct.ang_j_ij = mod( atan2( det([out_struct.vj_jnt_ij,[1;0]]) , dot(out_struct.vj_jnt_ij,[1;0]) ) , 2*pi );
                            out_struct.ang_j_jk = mod( atan2( det([out_struct.vj_jnt_jk,[1;0]]) , dot(out_struct.vj_jnt_jk,[1;0]) ) , 2*pi );
                            out_struct.ang_k_jk = mod( atan2( det([out_struct.vk_jnt_jk,[1;0]]) , dot(out_struct.vk_jnt_jk,[1;0]) ) , 2*pi );
                            out_struct.ang_k_ki = mod( atan2( det([out_struct.vk_jnt_ki,[1;0]]) , dot(out_struct.vk_jnt_ki,[1;0]) ) , 2*pi );
                            out_struct.ang_i_ki = mod( atan2( det([out_struct.vi_jnt_ki,[1;0]]) , dot(out_struct.vi_jnt_ki,[1;0]) ) , 2*pi );
                            
                            out_struct.rotz_i_ij = rotz(out_struct.ang_i_ij);
                            out_struct.rotz_j_ij = rotz(out_struct.ang_j_ij);
                            out_struct.rotz_j_jk = rotz(out_struct.ang_j_jk);
                            out_struct.rotz_k_jk = rotz(out_struct.ang_k_jk);
                            out_struct.rotz_k_ki = rotz(out_struct.ang_k_ki);
                            out_struct.rotz_i_ki = rotz(out_struct.ang_i_ki);
                            
                            out{i,j,k,frm_idx} = out_struct;
                        end
                    end
                end
            end
            
        end
end
end