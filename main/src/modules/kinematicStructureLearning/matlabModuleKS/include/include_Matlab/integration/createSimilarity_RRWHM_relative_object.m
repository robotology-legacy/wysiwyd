function problem = createSimilarity_RRWHM_relative_object(KineStruct_P, KineStruct_Q)

nP1 = KineStruct_P.num_seg;
nP2 = KineStruct_Q.num_seg;

%% 1st Order Similarity - Object
%=========================================================
% Distance to an object
%=========================================================
disp('..... calculating 1st order similarity to object!');
indH1_object = zeros(nP1*nP2,1);
valH1_object = zeros(nP1*nP2,1);

% Calculate distance to each joint
nF1 = KineStruct_P.num_frames;
nF2 = KineStruct_Q.num_frames;

dist_obj_P = zeros(nP1, nF1);
dist_obj_Q = zeros(nP2, nF2);

for id_P1 = 1:nP1
    for fidx_P1 = 1:nF1
%         dist_obj_P(id_P1, fidx_P1) = sqrt(sum((KineStruct_P.object(:,id_P1,fidx_P1)-KineStruct_P.seg_center(:,id_P1,fidx_P1)).^2));
        dist_obj_P(id_P1, fidx_P1) = sqrt(sum((KineStruct_P.object'-KineStruct_P.seg_center(:,id_P1,fidx_P1)).^2));
    end
end
dist_obj_P = mean(dist_obj_P,2);

for id_P2 = 1:nP2
    for fidx_P2 = 1:nF2
%         dist_obj_Q(id_P2, fidx_P2) = sqrt(sum((KineStruct_Q.object(:,id_P2,fidx_P2)-KineStruct_Q.seg_center(:,id_P2,fidx_P2)).^2));
        dist_obj_Q(id_P2, fidx_P2) = sqrt(sum((KineStruct_Q.object'-KineStruct_Q.seg_center(:,id_P2,fidx_P2)).^2));
    end
end
dist_obj_Q = mean(dist_obj_Q,2);
nodeSimilarity_object = dist_obj_P * dist_obj_Q';

% nodeSimilarity_object = reshape(nodeSimilarity_object, [nP1,nP2])';
% nodeSimilarity_object = nodeSimilarity_object';
nodeSimilarity_object = nodeSimilarity_object / max(max(nodeSimilarity_object));
nodeSimilarity_object = 1 - nodeSimilarity_object;

% for id_i1 = 1:nP1
%     for id_i2 = 1:nP2
%         nodeSimilarity_object(id_i1,id_i2) = nodeSimilarity_object(id_i1,id_i2);
%     end
% end

% nodeSimilarity = nodeSimilarity + nodeSimilarity_buf;

temp_idx = 1;
for id_i1 = 1:nP1
    for id_i2 = 1:nP2
        id_1 = (id_i1-1)*nP2 + (id_i2-1);
        
        indH1_object(temp_idx,1) = id_1;
        valH1_object(temp_idx,1) = exp(-nodeSimilarity_object(id_i1,id_i2));
        temp_idx = temp_idx + 1;
    end
end

indH1_object(temp_idx:end,:) = [];
valH1_object(temp_idx:end,:) = [];

%% 1st Order Similarity
%=========================================================
% Topology Similarity
%=========================================================
disp('..... calculating 1st order similarity!');
indH1_topology = zeros(nP1*nP2,1);
valH1_topology = zeros(nP1*nP2,1);

% calculate topological similarity
writeKineStruct(KineStruct_P, KineStruct_Q);

[status,result] = system('R -f ../../include/include_Matlab/topology_similarity/Incremmental_Topologically_constrained_subisomorphism_old2.r');
nodeSimilarity_topology = readNodeSimilarity('../../include/include_Matlab/topology_similarity/matrix-correspondance-with-removal.output');
% nodeSimilarity = readNodeSimilarity('../../include/include_Matlab/topology_similarity/matrix-correspondance-no-removal.output');
nodeSimilarity_topology = reshape(nodeSimilarity_topology, [KineStruct_P.num_seg,KineStruct_Q.num_seg])';
nodeSimilarity_topology = nodeSimilarity_topology';
nodeSimilarity_topology = nodeSimilarity_topology / max(max(nodeSimilarity_topology));

% End Joint & Intermediate Joint
structure_buf_P = [KineStruct_P.structure_i;KineStruct_P.structure_j];
structure_buf_Q = [KineStruct_Q.structure_i;KineStruct_Q.structure_j];

num_connection_P = histcounts(structure_buf_P, KineStruct_P.num_seg);
if strcmp(KineStruct_P.videoFileName,'video.avi')
    num_estimated_connection_P = checkConnection(KineStruct_P, 'P');
    num_connection_P = num_connection_P + num_estimated_connection_P;
end

num_connection_Q = histcounts(structure_buf_Q, KineStruct_Q.num_seg);
% if strcmp(KineStruct_Q.videoFileName,'video.avi')
%     num_estimated_connection_Q = checkConnection(KineStruct_Q, 'Q')
%     num_connection_Q = num_connection_Q + num_estimated_connection_Q;
% end

end_joint_P = (num_connection_P == 1);
end_joint_Q = (num_connection_Q == 1);

% nodeSimilarity_buf = nodeSimilarity;
for id_i1 = 1:nP1
    for id_i2 = 1:nP2
%         nodeSimilarity_buf(id_i1,id_i2) = end_joint_P(id_i1)*end_joint_Q(id_i2);
%         nodeSimilarity(id_i1,id_i2) = end_joint_P(id_i1)*end_joint_Q(id_i2);
%         if num_connection_P(id_i1) == num_connection_Q(id_i2)
%             nodeSimilarity(id_i1,id_i2) = 1;
%         end
        nodeSimilarity_topology(id_i1,id_i2) = exp(-abs(num_connection_P(id_i1)-num_connection_Q(id_i2))) * nodeSimilarity_topology(id_i1,id_i2);
    end
end

% nodeSimilarity = nodeSimilarity + nodeSimilarity_buf;


temp_idx = 1;
for id_i1 = 1:nP1
    for id_i2 = 1:nP2
        id_1 = (id_i1-1)*nP2 + (id_i2-1);
        
        indH1_topology(temp_idx,1) = id_1;
        valH1_topology(temp_idx,1) = nodeSimilarity_topology(id_i1,id_i2);
        temp_idx = temp_idx + 1;
    end
end
% [temp_idx-1, numElements(nP1, nP2, 1)]

indH1_topology(temp_idx:end,:) = [];
valH1_topology(temp_idx:end,:) = [];

% w1 = numElements(nP1, nP2, 1);
% valH1 = valH1 / max(valH1) / w1;


indH1 = (indH1_object + indH1_topology) / 2;
valH1 = (valH1_object / max(valH1_object) + valH1_topology / max(valH1_topology)) / 2;

%% 2nd Order Similarity
%=========================================================
% Kinematic Similarity
%=========================================================
disp('..... calculating 2nd order similarity!')
indH2 = zeros(nP1^2*nP2^2,2);
valH2 = zeros(nP1^2*nP2^2,1);

aff1 = KineStruct_P.affinity;
aff2 = KineStruct_Q.affinity;

Eij1 = zeros(nP1,nP1);
for idx = 1:length(KineStruct_P.structure_i)
    P_i = KineStruct_P.structure_i(idx);
    P_j = KineStruct_P.structure_j(idx);
    Eij1(P_i, P_j) = 1;
    Eij1(P_j, P_i) = 1;
end

Eij2 = zeros(nP2,nP2);
for idx = 1:length(KineStruct_Q.structure_i)
    P_i = KineStruct_Q.structure_i(idx);
    P_j = KineStruct_Q.structure_j(idx);
    Eij2(P_i, P_j) = 1;
    Eij2(P_j, P_i) = 1;
end

temp_idx = 1;
for id_j1 = 1:nP1
    for id_i1 = 1:nP1
        for id_j2 = 1:nP2
            for id_i2 = 1:nP2
                if (id_i1 ~= id_j1) && (id_i2 ~= id_j2)
                    
                    %                     if Eij1(id_i1, id_j1) == 1 && Eij2(id_i2, id_j2) == 1
                    
                    id_2 = (id_i1-1) * nP2 + (id_i2-1);
                    id_1 = (id_j1-1) * nP2 + (id_j2-1);
                    
                    indH2(temp_idx,:) = [id_2, id_1];
                    %                     valH2(temp_idx,:) = aff1(id_i1,id_j1)*aff2(id_i2,id_j2);
                    valH2(temp_idx,:) = exp(-norm(aff1(id_i1,id_j1)-aff2(id_i2,id_j2)));
                    
                    temp_idx = temp_idx + 1;
                    %                     end
                end
            end
        end
    end
end
% [temp_idx-1, numElements(nP1, nP2, 2)]

indH2(temp_idx:end,:) = [];
valH2(temp_idx:end,:) = [];

% w2 = numElements(nP1, nP2, 2);
% valH2 = valH2 / max(valH2) / w2;

%% 3rd Order Similarity
%=========================================================
% Motion Similarity
%=========================================================
disp('..... calculating 3rd order similarity!')
indH3 = zeros(nP1^3*nP2^3,3);
valH3 = zeros(nP1^3*nP2^3,1);

method = 'log';
% method = 'angle';

% Precompute Matrix
disp('.......... generating pre-computed matrices!')
preCompMtxP = preComputedMtx_v3(KineStruct_P, method);
preCompMtxQ = preComputedMtx_v3(KineStruct_Q, method);

h = waitbar(0,'Generating similarity mtx for RRWHM...');

temp_idx = 1;
step = 1;
steps = nP1^3;

for id_k1 = 1:nP1
    for id_j1 = 1:nP1
        for id_i1 = 1:nP1
            
            waitbar(step / steps);
            step = step + 1;
            
            if (id_i1 ~= id_j1) && (id_j1 ~= id_k1) && (id_k1 ~= id_i1)
%                 feat1 = cal_motion_feat_batch_v3(KineStruct_P, id_i1, id_j1, id_k1, preCompMtxP, method);
                feat1 = [preCompMtxP{id_i1,id_j1,id_k1}.min,preCompMtxP{id_i1,id_j1,id_k1}.max,preCompMtxP{id_j1,id_k1,id_i1}.min,preCompMtxP{id_j1,id_k1,id_i1}.max,preCompMtxP{id_k1,id_i1,id_j1}.min,preCompMtxP{id_k1,id_i1,id_j1}.max] / 9.1514;
                
                buf_id3 = (id_k1-1) * nP2;
                buf_id2 = (id_j1-1) * nP2;
                buf_id1 = (id_i1-1) * nP2;
                
                for id_k2 = 1:nP2
                    for id_j2 = 1:nP2
                        if (id_j2 ~= id_k2)
                            for id_i2 = 1:nP2
                                
                                if (id_i2 ~= id_j2) && (id_k2 ~= id_i2)
                                    
                                    %                                 if Eij1(id_i1, id_j1) && Eij1(id_j1, id_k1) && Eij1(id_k1, id_i1) && Eij2(id_i2, id_j2) && Eij2(id_j2, id_k2) && Eij2(id_k2, id_i2)
                                    %                                 id_3 = buf_id3 + (id_k2-1);
                                    %                                 id_2 = buf_id2 + (id_j2-1);
                                    %                                 id_1 = buf_id1 + (id_i2-1);
                                    %
                                    %                                 indH3(temp_idx,:) = [id_3, id_2, id_1];
                                    indH3(temp_idx,:) = [buf_id3 + (id_k2-1), buf_id2 + (id_j2-1), buf_id1 + (id_i2-1)];
                                    
%                                     feat2 = cal_motion_feat_batch_v3(KineStruct_Q, id_i2, id_j2, id_k2, preCompMtxQ, method);
                                    feat2 = [preCompMtxQ{id_i2,id_j2,id_k2}.min,preCompMtxQ{id_i2,id_j2,id_k2}.max,preCompMtxQ{id_j2,id_k2,id_i2}.min,preCompMtxQ{id_j2,id_k2,id_i2}.max,preCompMtxQ{id_k2,id_i2,id_j2}.min,preCompMtxQ{id_k2,id_i2,id_j2}.max] / 9.1514;
                                    valH3(temp_idx,1) = exp(-norm(feat1-feat2));
                                    
                                    temp_idx = temp_idx + 1;
                                    %                                 end
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end
% [temp_idx-1, numElements(nP1, nP2, 3)]

indH3(temp_idx:end,:) = [];
valH3(temp_idx:end,:) = [];
close(h)

% w3 = numElements(nP1, nP2, 3);
% valH3 = valH3 / max(valH3) / w3;

%% save the matching problem
problem.indH1 = int32(indH1);
problem.valH1 = double(valH1);
problem.indH2 = int32(indH2);
problem.valH2 = double(valH2);
problem.indH3 = int32(indH3);
problem.valH3 = double(valH3);
problem.nP1 = nP1;
problem.nP2 = nP2;

problem.X0 = ones(nP2, nP1);

% problem.trueMatch = seq;
% problem.gtruth = zeros(nP2, nP1);
% for i = 1:nP1
%     problem.gtruth(seq(i), i) = 1;
% end
