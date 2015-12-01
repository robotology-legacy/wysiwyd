function problem = createSimilarity_RRWHM_relative_nonPreprocessing(KineStruct_P, KineStruct_Q)

nP1 = KineStruct_P.num_seg;
nP2 = KineStruct_Q.num_seg;

%% 1st Order Similarity
%=========================================================
% Topology Similarity
%=========================================================
disp('..... calculating 1st order similarity!');
indH1 = zeros(nP1*nP2,1);
valH1 = zeros(nP1*nP2,1);

% calculate topological similarity
writeKineStruct(KineStruct_P, KineStruct_Q);

system('R -f ../../include/include_Matlab/topology_similarity/Incremmental_Topologically_constrained_subisomorphism_old2.r');
nodeSimilarity = readNodeSimilarity('../../include/include_Matlab/topology_similarity/matrix-correspondance-with-removal.output');
% nodeSimilarity = readNodeSimilarity('./topology/matrix-correspondance-no-removal.output');
nodeSimilarity = reshape(nodeSimilarity, [KineStruct_P.num_seg,KineStruct_Q.num_seg])';
nodeSimilarity = nodeSimilarity';
nodeSimilarity = nodeSimilarity / max(max(nodeSimilarity));

temp_idx = 1;
for id_i1 = 1:nP1
    for id_i2 = 1:nP2
        id_1 = (id_i1-1)*nP2 + (id_i2-1);
        
        indH1(temp_idx,1) = id_1;
        valH1(temp_idx,1) = nodeSimilarity(id_i1,id_i2);
        temp_idx = temp_idx + 1;
    end
end
% [temp_idx-1, numElements(nP1, nP2, 1)]

indH1(temp_idx:end,:) = [];
valH1(temp_idx:end,:) = [];

% w1 = numElements(nP1, nP2, 1)
% max(valH1)
% valH1 = valH1 / max(valH1) / w1;

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

% w2 = numElements(nP1, nP2, 2)
% max(valH2)
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

% w3 = numElements(nP1, nP2, 3)
% [max(valH1),max(valH2),max(valH3)]
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
