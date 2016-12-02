% Created by Quynh Nguyen
function showMatching(algName, P1, P2, X, seq, GT, title_string, saveToFile)

if saveToFile
    addpath('export_fig-master');
end

fig = figure;
title(title_string);
hold on;
nP1 = size(P1, 2);
nP2 = size(P2, 2);

% plot outliers from 2nd point cloud
plot(P2(1, :), P2(2, :), 'k*', 'markerSize', 5);

% plot inliers from 1st point cloud
plot(P1(1, :), P1(2, :), 'ro', 'markerFaceColor', 'r', 'markerSize', 6);

% plot inliers from 2nd point cloud
for i = 1:nP1
    plot(P2(1, GT(i)), P2(2, GT(i)), 'bo', 'markerFaceColor', 'b', 'markerSize', 6);
end

% plot grouth truth
% for i = 1:nP1
%     plot([P1(1, i), P2(1, GT(i))], [P1(2, i), P2(2, GT(i))], 'b-', 'lineWidth', 2);
% end

% draw false matches
for i = 1:nP1
    if seq(i) ~= GT(i)
        plot([P1(1, i), P2(1, seq(i))], [P1(2, i), P2(2, seq(i))], 'r-', 'lineWidth', 2);
    end
end

% draw true matches
for i = 1:nP1
    if seq(i) == GT(i)
        plot([P1(1, i), P2(1, seq(i))], [P1(2, i), P2(2, seq(i))], 'b-', 'lineWidth', 2);
    end
end

if saveToFile
%     axis square;
%     axis off;
    box on;
    set(gcf, 'color', 'w');
    export_fig(algName, '-eps', '-painters', '-native');
end

hold off