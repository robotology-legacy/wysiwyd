function [pairs, maxBaseline] = loadCars(nTest)

maxBaseline = 1000;

for i = 1:nTest
    load(['data/car/pair_', num2str(i)]);
    pairs{i}.P1 = features1(:, 1:2)';
    pairs{i}.P2 = features2(:, 1:2)';
    pairs{i}.gTruth = gTruth;
    pairs{i}.N = length(gTruth);
    
    if gTruth ~= 1:length(gTruth) 
        disp('ERROR!!!!!!!!!!!!!!');
    end
    maxBaseline = min(maxBaseline, size(features1, 1)-length(gTruth));
%     maxBaseline = min(maxBaseline, size(features2, 1)-length(gTruth));
end
