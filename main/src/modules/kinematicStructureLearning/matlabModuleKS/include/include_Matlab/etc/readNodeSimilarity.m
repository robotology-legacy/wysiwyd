function out = readNodeSimilarity(fileName)

fileID = fopen(fileName, 'r');
formatSpec = '%f';
out = fscanf(fileID,formatSpec);
fclose(fileID);

end