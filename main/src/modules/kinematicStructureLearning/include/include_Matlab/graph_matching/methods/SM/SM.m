function [ X ] = SM( affinityMatrix )

options.disp = 0;
[eigenVectorMat eigenValue] = eigs(affinityMatrix, 1, 'lm', options); % EigenSolver
eigenVector = eigenVectorMat(:,1); % Principal eigenvector
X = abs(eigenVector);

end

