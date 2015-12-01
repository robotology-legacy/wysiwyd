function X = readNETALResult(resultName,KineStruct_P, KineStruct_Q)
A = importdata(resultName);

X = zeros(KineStruct_P.num_seg, KineStruct_Q.num_seg);

for idx = 1:length(A)
    buf = A{idx};
    idx_P = buf(4);
    idx_Q = buf(12);
    
    X(double(idx_P)-96, double(idx_Q)-96) = 1;    
end

end