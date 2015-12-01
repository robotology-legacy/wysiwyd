function writeKineStruct_for_NETAL(KineStruct_P, KineStruct_Q, method_path)

fileID_P = fopen([method_path,'NETAL/graph_P.tab'],'w');
for idx = 1:length(KineStruct_P.structure_j)
    char_j = char(KineStruct_P.structure_j(idx)-1+'a');
    char_i = char(KineStruct_P.structure_i(idx)-1+'a');
    fprintf(fileID_P,'%s%c\t%s%c\n', 'g2-', char_j, 'g2-', char_i);
end
fclose(fileID_P);

fileID_Q = fopen([method_path,'NETAL/graph_Q.tab'],'w');
for idx = randperm(length(KineStruct_Q.structure_j))
    char_j = char(KineStruct_Q.structure_j(idx)-1+'a');
    char_i = char(KineStruct_Q.structure_i(idx)-1+'a');
    fprintf(fileID_Q,'%s%c\t%s%c\n', 'g1-', char_j, 'g1-', char_i);
end
fclose(fileID_Q);

end