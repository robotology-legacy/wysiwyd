function newClusterPlot(x)

global newCluster

cVec = 'rbkgcmyrbkgcmybrkgcmybrkgcmybrkgcmybrkgcmy';

nc = 0;

for i = 1:size(newCluster,1)
    if ~isempty(newCluster{i})
        nc = nc+1;        
        plot(x(newCluster{i}',1),x(newCluster{i}',2),[cVec(nc) '.']);
    end
end