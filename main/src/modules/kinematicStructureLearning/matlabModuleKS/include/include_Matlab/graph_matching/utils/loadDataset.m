function pts = loadDataset(folder)
% pts:      Nx2

pts = {};
files = dir([folder, '*.mat']);
for i = 1:length(files)
    load([folder, files(i).name]);
    pts{i} = pts_coord;
end
