%% Make test problem for 2nd order methods
function [ problem_2nd ] = similarityScore_2nd( KineStruct_P, KineStruct_Q)

nP1 = KineStruct_P.num_seg;
nP2 = KineStruct_Q.num_seg;
% if size(problem.P1, 1) == 2 
%     P1 = problem.P1';
%     P2 = problem.P2';
% else 
%     P1 = problem.P1;
%     P2 = problem.P2;
% end

aff_1 = KineStruct_P.affinity;
aff_2 = KineStruct_Q.affinity;

%% 2nd Order Matrix
E12 = ones(nP1,nP2);
n12 = nnz(E12);
[L12(:,1), L12(:,2)] = find(E12);
[group1, group2] = make_group12(L12);

E1 = ones(nP1); E2 = ones(nP2);
[L1(:,1), L1(:,2)] = find(E1);
[L2(:,1), L2(:,2)] = find(E2);

% G1 = P1(L1(:,1),:)-P1(L1(:,2),:);
% G2 = P2(L2(:,1),:)-P2(L2(:,2),:);

% if 0
%     G1_x = reshape(G1(:,1), [nP1 nP1]);
%     G1_y = reshape(G1(:,2), [nP1 nP1]);
%     G2_x = reshape(G2(:,1), [nP2 nP2]);
%     G2_y = reshape(G2(:,2), [nP2 nP2]);
%     M = (repmat(G1_x, nP2, nP2)-kron(G2_x,ones(nP1))).^2 + (repmat(G1_y, nP2, nP2)-kron(G2_y,ones(nP1))).^2;
% else
%     G1 = sqrt(G1(:,1).^2+G1(:,2).^2);
%     G2 = sqrt(G2(:,1).^2+G2(:,2).^2);
%     G1 = reshape(G1, [nP1 nP1]);
%     G2 = reshape(G2, [nP2 nP2]);
%     M = (repmat(G1, nP2, nP2)-kron(G2,ones(nP1))).^2;
% end

%     G1 = sqrt(G1(:,1).^2+G1(:,2).^2);
%     G2 = sqrt(G2(:,1).^2+G2(:,2).^2);
%     G1 = reshape(G1, [nP1 nP1]);
%     G2 = reshape(G2, [nP2 nP2]);
%     M = (repmat(G1, nP2, nP2)-kron(G2,ones(nP1))).^2;

%%
% M = zeros(nP1^2, nP2^2);
% 
% for idx1 = 1:nP1^2
%     for idx2 = 1:nP2^2
%         M(idx1, idx2) = aff_1(L1(idx1,1), L1(idx1,2)) * aff_2(L2(idx2,1), L2(idx2,2));
%     end
% end

M = kron(aff_1,aff_2);
M(1:(n12+1):end)=0;

%% Return the value
problem_2nd.nP1 = nP1;
problem_2nd.nP2 = nP2;
% problem_2nd.P1 = P1;
% problem_2nd.P2 = P2;
problem_2nd.L12 = L12;
problem_2nd.E12 = E12;

problem_2nd.affinityMatrix = M;
problem_2nd.group1 = group1;
problem_2nd.group2 = group2;

end
