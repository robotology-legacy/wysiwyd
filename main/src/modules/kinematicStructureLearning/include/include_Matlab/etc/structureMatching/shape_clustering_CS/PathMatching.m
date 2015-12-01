function [save_C, Cost, Matching] = PathMatching(PF1,PF2,alpha)


n1 = length(PF1);
n2 = length(PF2);
n = max(n1,n2);
% C = ones(n,n) * Inf;
for i = 1:n1
    for j = 1:n2
        Cost = vertice_dis(PF1{i},PF2{j},alpha);
        C(i,j) = Cost;
    end
end
save_C = C;

[Cost, Matching] = OSB(C, 2.2);
% [Cost, Matching] = OSB(C, 0);