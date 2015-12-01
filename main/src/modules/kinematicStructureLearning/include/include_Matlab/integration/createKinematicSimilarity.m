function problem = createKinematicSimilarity(KineStruct_P, KineStruct_Q, problem)

nP1 = KineStruct_P.num_seg;
nP2 = KineStruct_Q.num_seg;

%% 2nd Order Tensor Construction
indH2 = [];
valH2 = [];

aff1 = KineStruct_P.affinity;
aff2 = KineStruct_Q.affinity;

for id_j1 = 1:nP1
    for id_i1 = 1:nP1
        for id_j2 = 1:nP2
            for id_i2 = 1:nP2
                if (id_i1 ~= id_j1) && (id_i2 ~= id_j2)
                    id_k1 = ((id_i1-1) + (id_j1-1)*nP1);
                    id_k2 = ((id_i2-1) + (id_j2-1)*nP2);
                    
                    indH2 = [indH2; [id_k1, id_k2]];                   
                    valH2 = [valH2; 1-aff1(id_j1,id_i1)*aff2(id_j2,id_i2)];                   
                end
            end
        end
    end
end

%% save the matching problem

problem.indH2 = int32(indH2);
problem.valH2 = double(valH2);

% problem.trueMatch = seq;
% problem.gtruth = zeros(nP2, nP1);
% for i = 1:nP1
%     problem.gtruth(seq(i), i) = 1;
% end
