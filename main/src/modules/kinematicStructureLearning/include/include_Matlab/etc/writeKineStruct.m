function writeKineStruct(KineStruct_P, KineStruct_Q)

fileID = fopen('../../include/include_Matlab/topology_similarity/myGraph.txt','w');
fprintf(fileID,'%s\t%s\t%s\n','n1','n2','graphType');
for idx = 1:length(KineStruct_P.structure_j)
    fprintf(fileID,'%d\t%d\t%s\n',KineStruct_P.structure_j(idx), KineStruct_P.structure_i(idx), 'g1');
end
% for idx = length(KineStruct_Q.structure_j):-1:1
for idx = randperm(length(KineStruct_Q.structure_j))    
    fprintf(fileID,'%d\t%d\t%s\n',KineStruct_Q.structure_j(idx), KineStruct_Q.structure_i(idx), 'g2');
end
fclose(fileID);

end