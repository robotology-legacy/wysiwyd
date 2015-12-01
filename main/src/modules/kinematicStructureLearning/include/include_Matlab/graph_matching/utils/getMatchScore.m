function MatchScore = getMatchScore(indH, valH, X)
% H: 3rd order potential
% X: Assignment matrix
MatchScore = mexGetMatchScore(indH, valH, X(:));