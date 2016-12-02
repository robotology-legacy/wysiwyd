function [Cost, Matching] = OSB(C, const)

[n1,n2] = size(C);
flag = 0;
if n1>n2;
    C=C';
    flag = 1;
end
n = min(n1,n2);
id = [1:n];
ds = [];
for i = 1:n
    C(n+1,:) = C(1,:);
    C(1,:) = [];
    id(n+1) = id(1);
    id(1) = [];
    costmat=C;
    statmat=min(costmat');
    jumpcost=mean(statmat)+std(statmat);
    [pathcost,indxrow,indxcol] = findpathDAG2(costmat,2, 2, jumpcost*const);
    Matching = zeros(n, max(n1,n2));
    for k = 1:length(indxrow)
        Matching (id(indxrow(k)), indxcol(k)) = 1;
    end
    ds = [ds;pathcost];
    Matchings{i} = Matching;
end
[Cost,l] = min(ds);
Matching = Matchings{l};
if(flag)
    Matching = Matching';
end