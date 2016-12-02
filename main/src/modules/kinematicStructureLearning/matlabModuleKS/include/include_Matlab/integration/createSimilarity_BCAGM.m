function problem = createSimilarity_BCAGM(KineStruct_P, KineStruct_Q)

nP1 = KineStruct_P.num_seg;
nP2 = KineStruct_Q.num_seg;

% calculate topological similarity
writeKineStruct(KineStruct_P, KineStruct_Q);

system('R -f topology/Incremmental_Topologically_constrained_subisomorphism.r');
nodeSimilarity = readNodeSimilarity('./topology/matrix-correspondance-with-removal.output');
% nodeSimilarity = readNodeSimilarity('./topology/matrix-correspondance-no-removal.output');
nodeSimilarity = reshape(nodeSimilarity, [KineStruct_P.num_seg,KineStruct_Q.num_seg])';
nodeSimilarity = nodeSimilarity';

%% 3rd Order Similarity
disp('..... calculating 3rd order similarity!')
indH3 = zeros(nP1^3*nP2^3,3);
valH3 = zeros(nP1^3*nP2^3,1);

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

method = 'log';
% method = 'angle';

% Precompute Matrix
disp('.......... generating pre-computed matrices!')
preCompMtxP = preComputedMtx_v2(KineStruct_P, method);
preCompMtxQ = preComputedMtx_v2(KineStruct_Q, method);

h = waitbar(0,'Generating similarity mtx for BCAGM...');

temp_idx = 1;

step = 1;
steps = nP1^3 * nP2^3;

normalisation1 = numElements(nP1, nP2, 1);
normalisation2 = numElements(nP1, nP2, 2);
normalisation3 = numElements(nP1, nP2, 3);

for id_k1 = 1:nP1
    for id_j1 = 1:nP1
        for id_i1 = 1:nP1
            
            if (id_i1 ~= id_j1) && (id_j1 ~= id_k1) && (id_k1 ~= id_i1)
                feat1 = cal_motion_feat_batch_v2(KineStruct_P, id_i1, id_j1, id_k1, preCompMtxP, method);
            end
            
            for id_k2 = 1:nP2
                for id_j2 = 1:nP2
                    for id_i2 = 1:nP2
                        
                        waitbar(step / steps);
                        step = step + 1;
                        
                        id_3 = (id_k1-1) * nP2 + (id_k2-1);
                        id_2 = (id_j1-1) * nP2 + (id_j2-1);
                        id_1 = (id_i1-1) * nP2 + (id_i2-1);
                        
                        indH3(temp_idx,:) = [id_3, id_2, id_1];
                        
                        %------------------------------------
                        % 1st order
                        if (id_i1 == id_j1) && (id_j1 == id_k1) && (id_k1 == id_i1) && (id_i2 == id_j2) && (id_j2 == id_k2) && (id_k2 == id_i2)
                            
                            valH3(temp_idx,1) = nodeSimilarity(id_i1,id_i2) / normalisation1;
                            
                            %------------------------------------
                            % 3rd order
                        elseif (id_i1 ~= id_j1) && (id_j1 ~= id_k1) && (id_k1 ~= id_i1) && (id_i2 ~= id_j2) && (id_j2 ~= id_k2) && (id_k2 ~= id_i2)
                            
%                             if Eij1(id_i1, id_j1) && Eij1(id_j1, id_k1) && Eij1(id_k1, id_i1) && Eij2(id_i2, id_j2) && Eij2(id_j2, id_k2) && Eij2(id_k2, id_i2)
                                feat2 = cal_motion_feat_batch_v2(KineStruct_Q, id_i2, id_j2, id_k2, preCompMtxQ, method);
                                valH3(temp_idx,1) = exp(-norm(feat1-feat2)) / normalisation3;
%                             end
                            %------------------------------------
                            % 2nd order
                        else
%                             if Eij1(id_i1, id_j1) == 1 && Eij2(id_i2, id_j2) == 1
                                valH3(temp_idx,1) = exp(-norm(aff1(id_i1,id_j1)-aff2(id_i2,id_j2))) / normalisation2;
%                             end
                        end
                        
                        temp_idx = temp_idx + 1;
                    end
                end
            end
        end
    end
end
indH3(temp_idx:end,:) = [];
valH3(temp_idx:end,:) = [];

close(h)

%% save the matching problem
problem.indH1 = [];
problem.valH1 = [];
problem.indH2 = [];
problem.valH2 = [];
problem.indH3 = int32(indH3);
problem.valH3 = double(valH3);

problem.X0 = ones(nP2, nP1);

% problem.trueMatch = seq;
% problem.gtruth = zeros(nP2, nP1);
% for i = 1:nP1
%     problem.gtruth(seq(i), i) = 1;
% end
