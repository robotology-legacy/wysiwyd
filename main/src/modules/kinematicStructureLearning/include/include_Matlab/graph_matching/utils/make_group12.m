function [group1 group2] = make_group12(matchList)
% make group1 and group2 based on the match list
% group1 = # of matches by ( # of groups in feat1 )
% group2 = # of matches by ( # of groups in feat2 )

nMatch = size(matchList,1);

featList = matchList(:,1);
featList_unique = unique(featList);
nGroup = length(featList_unique);
group = logical(sparse(nMatch,nGroup));
for i=1:nGroup
    group( find(featList == featList_unique(i)), i) = true;
end
group1 = group;

featList = matchList(:,2);
featList_unique = unique(featList);
nGroup = length(featList_unique);
group = logical(sparse(nMatch,nGroup));
for i=1:nGroup
    group( find(featList == featList_unique(i)), i) = true;
end
group2 = group;