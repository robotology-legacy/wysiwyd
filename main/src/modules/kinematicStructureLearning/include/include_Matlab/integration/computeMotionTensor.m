function [feat] = computeMotionTensor(KineStruct_P, KineStruct_Q)

nNode1 = KineStruct_P.num_seg;
nNode2 = KineStruct_Q.num_seg;

nFrm1 = KineStruct_P.num_frames;
nFrm2 = KineStruct_Q.num_frames;

jnt1 = KineStruct_P.joint_center;
jnt2 = KineStruct_Q.joint_center;

node1 = KineStruct_P.seg_center;
node2 = KineStruct_Q.seg_center;

idxSet1 = zeros(nNode1^3,3);
idxSet2 = zeros(nNode2^3,3);

angSet1 = zeros(nNode1^3,6);
angSet2 = zeros(nNode2^3,6);

distSet1 = zeros(nNode1^3,6);
distSet2 = zeros(nNode2^3,6);

% [idxSet1(:,3), idxSet1(:,2), idxSet1(:,1)] = find(ones(nNode1, nNode1, nNode1));
% [idxSet2(:,3), idxSet2(:,2), idxSet2(:,1)] = find(ones(nNode2, nNode2, nNode2));
n=1;
for i=1:nNode1
    for j=1:nNode1
        for k=1:nNode1
            if i~=j && j~=k && k~=i
                idxSet1(n,:) = [i,j,k];
                
                ang_buf = zeros(nFrm1,3);
                dist_buf = zeros(nFrm1,3);
                
                for frm_idx = 1:nFrm1
                    % angle between i-j
                    vi_jnt_ij = KineStruct_P.seg_center(:,i,frm_idx) - jnt1{i,j}(:,frm_idx);
                    vj_jnt_ij = KineStruct_P.seg_center(:,j,frm_idx) - jnt1{i,j}(:,frm_idx);
                    ang_ij = mod( atan2( det([vi_jnt_ij,vj_jnt_ij]) , dot(vi_jnt_ij,vj_jnt_ij) ) , 2*pi );
                    ang_i_ij = mod( atan2( det([vi_jnt_ij,[1;0]]) , dot(vi_jnt_ij,[1;0]) ) , 2*pi );
                    ang_j_ij = mod( atan2( det([vj_jnt_ij,[1;0]]) , dot(vj_jnt_ij,[1;0]) ) , 2*pi );                    
                    
                    % angle between j-k
                    vj_jnt_jk = KineStruct_P.seg_center(:,j,frm_idx) - jnt1{j,k}(:,frm_idx);
                    vk_jnt_jk = KineStruct_P.seg_center(:,k,frm_idx) - jnt1{j,k}(:,frm_idx);
                    ang_jk = mod( atan2( det([vj_jnt_jk,vk_jnt_jk]) , dot(vj_jnt_jk,vk_jnt_jk) ) , 2*pi );
                    ang_j_jk = mod( atan2( det([vj_jnt_jk,[1;0]]) , dot(vj_jnt_jk,[1;0]) ) , 2*pi );
                    ang_k_jk = mod( atan2( det([vk_jnt_jk,[1;0]]) , dot(vk_jnt_jk,[1;0]) ) , 2*pi );                    
                    
                    % angle between k-i
                    vk_jnt_ki = KineStruct_P.seg_center(:,k,frm_idx) - jnt1{k,i}(:,frm_idx);
                    vi_jnt_ki = KineStruct_P.seg_center(:,i,frm_idx) - jnt1{k,i}(:,frm_idx);
                    ang_ki = mod( atan2( det([vk_jnt_ki,vi_jnt_ki]) , dot(vk_jnt_ki,vi_jnt_ki) ) , 2*pi );
                    ang_k_ki = mod( atan2( det([vk_jnt_ki,[1;0]]) , dot(vk_jnt_ki,[1;0]) ) , 2*pi );
                    ang_i_ki = mod( atan2( det([vi_jnt_ki,[1;0]]) , dot(vi_jnt_ki,[1;0]) ) , 2*pi );                    
                    
                    ang_buf(frm_idx,:) = [ang_ij, ang_jk, ang_ki];
                    %%
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
                min_1 = min(ang_buf,[],1);
                max_1 = max(ang_buf,[],1);
                angSet1(n,:) = reshape([min_1;max_1], [1,6]);
                %%
                min_1 = min(dist_buf,[],1);
                max_1 = max(dist_buf,[],1);
                distSet1(n,:) = reshape([min_1;max_1], [1,6]);
                
                n = n+1;
            end
        end
    end
end
idxSet1(n:end,:) = [];
angSet1(n:end,:) = [];
distSet1(n:end,:) = [];

n=1;
for i=1:nNode2
    for j=1:nNode2
        for k=1:nNode2
            if i~=j && j~=k && k~=i
                idxSet2(n,:) = [i,j,k];
                
                ang_buf = zeros(nFrm2,3);
                dist_buf = zeros(nFrm2,3);
                for frm_idx = 1:nFrm2
                    % angle between i-j
                    vi_jnt_ij = KineStruct_Q.seg_center(:,i,frm_idx) - jnt2{i,j}(:,frm_idx);
                    vj_jnt_ij = KineStruct_Q.seg_center(:,j,frm_idx) - jnt2{i,j}(:,frm_idx);
                    ang_ij = mod( atan2( det([vi_jnt_ij,vj_jnt_ij]) , dot(vi_jnt_ij,vj_jnt_ij) ) , 2*pi );
                    ang_i_ij = mod( atan2( det([vi_jnt_ij,[1;0]]) , dot(vi_jnt_ij,[1;0]) ) , 2*pi );
                    ang_j_ij = mod( atan2( det([vj_jnt_ij,[1;0]]) , dot(vj_jnt_ij,[1;0]) ) , 2*pi );

                    % angle between j-k
                    vj_jnt_jk = KineStruct_Q.seg_center(:,j,frm_idx) - jnt2{j,k}(:,frm_idx);
                    vk_jnt_jk = KineStruct_Q.seg_center(:,k,frm_idx) - jnt2{j,k}(:,frm_idx);
                    ang_jk = mod( atan2( det([vj_jnt_jk,vk_jnt_jk]) , dot(vj_jnt_jk,vk_jnt_jk) ) , 2*pi );
                    ang_j_jk = mod( atan2( det([vj_jnt_jk,[1;0]]) , dot(vj_jnt_jk,[1;0]) ) , 2*pi );
                    ang_k_jk = mod( atan2( det([vk_jnt_jk,[1;0]]) , dot(vk_jnt_jk,[1;0]) ) , 2*pi );
                    
                    % angle between k-i
                    vk_jnt_ki = KineStruct_Q.seg_center(:,k,frm_idx) - jnt2{k,i}(:,frm_idx);
                    vi_jnt_ki = KineStruct_Q.seg_center(:,i,frm_idx) - jnt2{k,i}(:,frm_idx);
                    ang_ki = mod( atan2( det([vk_jnt_ki,vi_jnt_ki]) , dot(vk_jnt_ki,vi_jnt_ki) ) , 2*pi );
                    ang_k_ki = mod( atan2( det([vk_jnt_ki,[1;0]]) , dot(vk_jnt_ki,[1;0]) ) , 2*pi );
                    ang_i_ki = mod( atan2( det([vi_jnt_ki,[1;0]]) , dot(vi_jnt_ki,[1;0]) ) , 2*pi );                    
                    
                    ang_buf(frm_idx,:) = [ang_ij, ang_jk, ang_ki];
                    
                    %%
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
                min_2 = min(ang_buf,[],1);
                max_2 = max(ang_buf,[],1);
                angSet2(n,:) = reshape([min_2;max_2], [1,6]);
                
                %%
                min_2 = min(dist_buf,[],1);
                max_2 = max(dist_buf,[],1);
                distSet2(n,:) = reshape([min_2;max_2], [1,6]);
                
                n = n+1;
            end
        end
    end
end
idxSet2(n:end,:) = [];
angSet2(n:end,:) = [];
distSet2(n:end,:) = [];


nIdxSet1 = length(idxSet1);
nIdxSet2 = length(idxSet2);

H_ang = zeros(nNode1^3, nNode2^3);
H_dist = zeros(nNode1^3, nNode2^3);

for idx1 = 1:nIdxSet1
    for idx2 = 1:nIdxSet2
        i1 = (idxSet1(idx1,1) - 1) * nNode1^2 + (idxSet1(idx1,2) - 1) * nNode1 + idxSet1(idx1,3);
        i2 = (idxSet2(idx2,1) - 1) * nNode2^2 + (idxSet2(idx2,2) - 1) * nNode2 + idxSet2(idx2,3);
        
        % by 2-norm
                norm_compare_ang = [norm(angSet1(idx1,:) - angSet2(idx2,:))] ./ norm([1 1 1 1 1 1]);
                norm_compare_dist = [norm(distSet1(idx1,:) - distSet2(idx2,:))] ./ norm([1 1 1 1 1 1]);
        
        % by Frobenius norm
%         norm_compare_ang = norm(angSet1(idx1,:) - angSet2(idx2,:), 'fro');
%         norm_compare_dist = norm(distSet1(idx1,:) - distSet2(idx2,:), 'fro');
        
        %         H(i1,i2) = sum(norm_compare);
        H_ang(i1,i2) = norm_compare_ang;
        H_dist(i1,i2) = norm_compare_dist;
    end
end

feat = H;

end