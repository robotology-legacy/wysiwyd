% function plotResults_House(xData, yData, Alg, plotSet, filename)

addpath('export_fig-master');

hFig = figure;
hold on;

set(gca, 'fontSize', plotSet.fontSize*0.8);

for k = 1:length(Alg)
    plot(xData, yData(k, :), ...
        'LineWidth', plotSet.lineWidth, ...
        'Color', Alg(k).color, ...
        'LineStyle', Alg(k).lineStyle, ...
        'Marker', Alg(k).marker, ...
        'MarkerSize', plotSet.markerSize);
end
if ~exist('Xmin', 'var') || ~exist('Xmax', 'var')
    Xmin = min(xData(:));
    Xmax = max(xData(:));
end
if ~exist('Ymin', 'var') || ~exist('Ymax', 'var')
    Ymin = min(yData(:));
    Ymax = max(yData(:));
end
axis tight;
axis([Xmin Xmax Ymin-0.02*(Ymax-Ymin) Ymax+0.02*(Ymax-Ymin)]);
xlabel([plotSet.font plotSet.xLabelText], 'FontSize', plotSet.fontSize);
ylabel([plotSet.font plotSet.yLabelText], 'FontSize', plotSet.fontSize);

T = get(gca,'tightinset');
set(gca,'position',[T(1) T(2) 1-T(1)-T(3) 1-T(2)-T(4)]);

if show_legend
    hLegend = legend(Alg(:).strName);
    set(hLegend, 'Location', 'southwest', 'fontSize', 16)
end

hold off;

% legend('boxoff');
set(gcf, 'color', 'w');
export_fig(filename, '-eps', '-painters', '-native');

clear Xmin Xmax Ymin Ymax k yData yLabelText hLegend

