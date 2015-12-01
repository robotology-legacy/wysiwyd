function dist = cal_logm(R_A,R_B)
    dist = norm(logm(R_A'*R_B),'fro');
end