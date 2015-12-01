function [feat1, feat2] = computMotionFeature(KineStruct_P, KineStruct_Q, t1)

nNode1 = KineStruct_P.num_seg;
nNode2 = KineStruct_Q.num_seg;

nFrm1 = KineStruct_P.num_frames;
nFrm2 = KineStruct_Q.num_frames;

feat1 = zeros(6,length(t1));
feat2 = zeros(6,nNode2^3);

t1 = t1 + 1;

method = 'log';
% method = 'angle';

%% Precompute Matrix
preCompMtxP = preComputedMtx(KineStruct_P, method);
preCompMtxQ = preComputedMtx(KineStruct_Q, method);

%% Cal feat1
for t=1:length(t1)
    i= t1(1,t);
    j= t1(2,t);
    k= t1(3,t);
    
    if i==j || j==k || k==i
        feat1(:,t) = -10;
    else
        %         feat1(:,t) = cal_motion_feat(KineStruct_P, i, j, k, method);
        feat1(:,t) = cal_motion_feat_batch(KineStruct_P, i, j, k, preCompMtxP, method);
    end
end

%% Cal feat2
t=0;
for i=1:nNode2
    for j=1:nNode2
        for k=1:nNode2
            t = t+1;
            
            if i==j || j==k || k==i
                feat2(:,t) = -10;
            else
                %                 feat2(:,t) = cal_motion_feat(KineStruct_Q, i, j, k, method);
                feat2(:,t) = cal_motion_feat_batch(KineStruct_Q, i, j, k, preCompMtxQ, method);
            end
        end
    end
end

end

%%
function feat_out = cal_motion_feat(KineStruct, i, j, k, method)

nFrm = KineStruct.num_frames;
jnt = KineStruct.joint_center;

ang_buf = zeros(nFrm,3);
dist_buf = zeros(nFrm,3);

switch(method)
    case 'angle'
        parfor frm_idx = 1:nFrm
            % angle between i-j
            vi_jnt_ij = KineStruct.seg_center(:,i,frm_idx) - jnt{i,j}(:,frm_idx);
            vj_jnt_ij = KineStruct.seg_center(:,j,frm_idx) - jnt{i,j}(:,frm_idx);
            
            % angle between j-k
            vj_jnt_jk = KineStruct.seg_center(:,j,frm_idx) - jnt{j,k}(:,frm_idx);
            vk_jnt_jk = KineStruct.seg_center(:,k,frm_idx) - jnt{j,k}(:,frm_idx);
            
            % angle between k-i
            vk_jnt_ki = KineStruct.seg_center(:,k,frm_idx) - jnt{k,i}(:,frm_idx);
            vi_jnt_ki = KineStruct.seg_center(:,i,frm_idx) - jnt{k,i}(:,frm_idx);
            
            ang_ij = mod( atan2( det([vi_jnt_ij,vj_jnt_ij]) , dot(vi_jnt_ij,vj_jnt_ij) ) , 2*pi );
            ang_jk = mod( atan2( det([vj_jnt_jk,vk_jnt_jk]) , dot(vj_jnt_jk,vk_jnt_jk) ) , 2*pi );
            ang_ki = mod( atan2( det([vk_jnt_ki,vi_jnt_ki]) , dot(vk_jnt_ki,vi_jnt_ki) ) , 2*pi );
            
            ang_buf(frm_idx,:) = [ang_ij, ang_jk, ang_ki];
        end
        %%
        min_value = min(ang_buf,[],1);
        max_value = max(ang_buf,[],1);
        feat_out = reshape([min_value;max_value], [1,6])';
        
    case 'log'
        parfor frm_idx = 1:nFrm
            % angle between i-j
            vi_jnt_ij = KineStruct.seg_center(:,i,frm_idx) - jnt{i,j}(:,frm_idx);
            vj_jnt_ij = KineStruct.seg_center(:,j,frm_idx) - jnt{i,j}(:,frm_idx);
            
            % angle between j-k
            vj_jnt_jk = KineStruct.seg_center(:,j,frm_idx) - jnt{j,k}(:,frm_idx);
            vk_jnt_jk = KineStruct.seg_center(:,k,frm_idx) - jnt{j,k}(:,frm_idx);
            
            % angle between k-i
            vk_jnt_ki = KineStruct.seg_center(:,k,frm_idx) - jnt{k,i}(:,frm_idx);
            vi_jnt_ki = KineStruct.seg_center(:,i,frm_idx) - jnt{k,i}(:,frm_idx);
            
            %%
            ang_i_ij = mod( atan2( det([vi_jnt_ij,[1;0]]) , dot(vi_jnt_ij,[1;0]) ) , 2*pi );
            ang_j_ij = mod( atan2( det([vj_jnt_ij,[1;0]]) , dot(vj_jnt_ij,[1;0]) ) , 2*pi );
            ang_j_jk = mod( atan2( det([vj_jnt_jk,[1;0]]) , dot(vj_jnt_jk,[1;0]) ) , 2*pi );
            ang_k_jk = mod( atan2( det([vk_jnt_jk,[1;0]]) , dot(vk_jnt_jk,[1;0]) ) , 2*pi );
            ang_k_ki = mod( atan2( det([vk_jnt_ki,[1;0]]) , dot(vk_jnt_ki,[1;0]) ) , 2*pi );
            ang_i_ki = mod( atan2( det([vi_jnt_ki,[1;0]]) , dot(vi_jnt_ki,[1;0]) ) , 2*pi );
            
            rotz_i_ij = rotz(ang_i_ij);
            rotz_j_ij = rotz(ang_j_ij);
            rotz_j_jk = rotz(ang_j_jk);
            rotz_k_jk = rotz(ang_k_jk);
            rotz_k_ki = rotz(ang_k_ki);
            rotz_i_ki = rotz(ang_i_ki);
            
            pair_buf = [cal_logm(rotz_i_ij, rotz_j_ij), cal_logm(rotz_j_jk, rotz_k_jk), cal_logm(rotz_k_ki, rotz_i_ki)] ./ cal_logm(rotz(0), rotz(pi));
            dist_buf(frm_idx,:) = pair_buf;
        end
        
        %%
        min_value = min(dist_buf,[],1);
        max_value = max(dist_buf,[],1);
        feat_out = reshape([min_value;max_value], [1,6])';
end
end

%%
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
            pair_buf = [cal_logm(preCompMtx{i,j,k,frm_idx}.rotz_i_ij, preCompMtx{i,j,k,frm_idx}.rotz_j_ij), cal_logm(preCompMtx{i,j,k,frm_idx}.rotz_j_jk, preCompMtx{i,j,k,frm_idx}.rotz_k_jk), cal_logm(preCompMtx{i,j,k,frm_idx}.rotz_k_ki, preCompMtx{i,j,k,frm_idx}.rotz_i_ki)] ./ cal_logm(rotz(0), rotz(pi));
            dist_buf(frm_idx,:) = pair_buf;
        end
        
        %%
        min_value = min(dist_buf,[],1);
        max_value = max(dist_buf,[],1);
        feat_out = reshape([min_value;max_value], [1,6])';
end
end

%%
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