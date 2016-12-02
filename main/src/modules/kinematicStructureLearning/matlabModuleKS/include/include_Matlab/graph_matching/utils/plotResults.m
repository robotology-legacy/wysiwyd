addpath('export_fig-master');

handleCount = handleCount+1; h(handleCount) = figure;
hold on;

set(gca, 'fontSize', plotSet.fontSize*0.8);

for k = 1:length(Alg)
    if 0
        errorbar(settings{Con}{4}, yData(:,k), stdData(:, k), ...
            'LineWidth', plotSet.lineWidth, ...
            'Color', Alg(k).color, ...
            'LineStyle', Alg(k).lineStyle, ...
            'Marker', Alg(k).marker, ...
            'MarkerSize', plotSet.markerSize);
    else
        plot(settings{Con}{4}, yData(:,k), ...
            'LineWidth', plotSet.lineWidth, ...
            'Color', Alg(k).color, ...
            'LineStyle', Alg(k).lineStyle, ...
            'Marker', Alg(k).marker, ...
            'MarkerSize', plotSet.markerSize);
    end
end
if ~exist('Xmin', 'var') || ~exist('Xmax', 'var')
    Xmin = min(settings{Con}{4});
    Xmax = max(settings{Con}{4});
end
if ~exist('Ymin', 'var') || ~exist('Ymax', 'var')
    Ymin = min(yData(:));
    Ymax = max(yData(:));
end
axis([Xmin Xmax Ymin-0.02*(Ymax-Ymin) Ymax+0.02*(Ymax-Ymin)]);
xlabel([plotSet.font settings{Con}{2}], 'FontSize', plotSet.fontSize);
ylabel([plotSet.font yLabelText], 'FontSize', plotSet.fontSize);
% for k = 1:length(Fix)
%     text(Xmin+0.1*(Xmax-Xmin), Ymin+0.1*(length(Fix)-k+1)*(Ymax-Ymin), ...
%         [plotSet.font settings{Fix(k)}{2} ' = ' num2str(settings{Fix(k)}{4})], ...
%         'FontSize', plotSet.fontSize);
% end
if show_legend
    hLegend = legend(Alg(:).strName);
    % set(hLegend, 'Location', 'best');
    set(hLegend, 'Location', 'best', 'FontSize', 16);
end

T = get(gca,'tightinset');
set(gca,'position',[T(1) T(2) 1-T(1)-T(3) 1-T(2)-T(4)]);

hold off;

% legend('boxoff');
set(gcf, 'color', 'w');
export_fig(filename, '-eps', '-painters', '-native');

clear Xmin Xmax Ymin Ymax k yData yLabelText hLegend