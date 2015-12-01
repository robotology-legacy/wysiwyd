function X = readMAGNAResult(resultName,KineStruct_P, KineStruct_Q)
A = importdata(resultName);

X = zeros(KineStruct_P.num_seg, KineStruct_Q.num_seg);

for idx = 1:length(A)    
    X(A(idx,1), A(idx,2)) = 1;    
end

end