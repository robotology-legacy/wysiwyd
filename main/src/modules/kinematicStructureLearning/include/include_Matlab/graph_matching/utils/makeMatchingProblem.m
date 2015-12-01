%% Make test problem
function [ problem ] = makeMatchingProblem( Set )
%% Get values from structure
strNames = fieldnames(Set);
for i = 1:length(strNames), eval([strNames{i} '= Set.' strNames{i} ';']); end
%% Set number of nodes
if bOutBoth, nP1 = nInlier + nOutlier; else nP1 = nInlier; end
nP2 = nInlier + nOutlier;

%% Generate Nodes
switch typeDistribution
    case 'normal', P1 = randn(2, nP1); Pout = randn(2,nOutlier); % Points in Domain 1
    case 'uniform', P1 = rand(2, nP1); Pout = rand(2,nOutlier); % Points in Domain 1
    otherwise, disp(''); error('Insert Point Distribution');
end

% point transformation matrix
Mrot = [cos(transRotate) -sin(transRotate) ; sin(transRotate) cos(transRotate) ];
P2 = Mrot*P1*transScale+deformation*randn(2,nP1); % Points in Domain 2
if bOutBoth, P2(:,(nInlier+1):end) = Pout; else P2 = [P2 Pout]; end


% % show two point sets 
% figure; title('after transformation');
% plot(P1(1, :), P1(2, :), 'ro', 'markerFaceColor', 'r'); hold on;
% plot(P2(1, 1:nP1), P2(2, 1:nP1), 'bo', 'markerFaceColor', 'b'); hold on;
% plot(P2(1, nP1+1:end), P2(2, nP1+1:end), 'ko'); hold on;
% pause;

problem = createTensor(bPermute, P1, P2);

end
