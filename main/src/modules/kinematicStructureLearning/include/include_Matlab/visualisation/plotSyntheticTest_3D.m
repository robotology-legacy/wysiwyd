setPlotOption;

set(gca, 'fontSize', plotSet.fontSize*0.8);


xlabel([plotSet.font 'Motion variation'], 'FontSize', plotSet.fontSize);
ylabel([plotSet.font 'Accuracy'], 'FontSize', plotSet.fontSize);

acc_total_plot = acc_total;
acc_total_plot(:,:,1) = acc_total(:,:,5);
acc_total_plot(:,:,2) = acc_total(:,:,6);
acc_total_plot(:,:,3) = acc_total(:,:,2);
acc_total_plot(:,:,4) = acc_total(:,:,3);
acc_total_plot(:,:,5) = acc_total(:,:,4);
acc_total_plot(:,:,6) = acc_total(:,:,1);

% figure(654)
hfig = figure
clf
hold on
for plot_idx = 1:size(acc_total_plot,3);
    acc_value = mean(acc_total_plot(:,:,plot_idx),1);
    plot(acc_value, ...
        'LineWidth', plotSet.lineWidth, ...
        'Color', plotOption(plot_idx).color, ...
        'LineStyle', plotOption(plot_idx).lineStyle, ...
        'Marker', plotOption(plot_idx).marker, ...
        'MarkerSize', plotSet.markerSize);
    axis([1,length(acc_value),0,1])
end
% axis([1,length(acc_value),0,1])

% test = 1;

switch(testName)
    case 'nodeAddTest'  % for node adding
        show_legend = 0;
        if show_legend
            hLegend = legend(plotOption(:).strName);
            set(hLegend, 'Location', 'best');
%                 set(hLegend, 'Location', 'best', 'FontSize', 9);
        end
        
        ylabel('Accuracy');
        xlabel('# of outliers');
        set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9'});
        
        fig_save_name = ['/home/hjchang/Research/code/Matlab/cvpr2016/code/result/synthetic/graph/nodeAddTest_3D'];
        
    case 'motionVarTest'  % for motion perturb
        show_legend = 0;
        if show_legend
            hLegend = legend(plotOption(:).strName);
            set(hLegend, 'Location', 'best');
            %     set(hLegend, 'Location', 'best', 'FontSize', 14);
        end
        
        ylabel('Accuracy');
        xlabel('Motion perturbation ratio');
        set(gca,'XTickLabel',{'0.0','0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9'});
        
        fig_save_name = ['/home/hjchang/Research/code/Matlab/cvpr2016/code/result/synthetic/graph/motionPerturbationTest_3D'];
        
    case 'symmetryTest'  % for symmetry
        show_legend = 1;
        if show_legend
            hLegend = legend(plotOption(:).strName);
            set(hLegend, 'Location', 'best');
            %     set(hLegend, 'Location', 'best', 'FontSize', 14);
        end
        
        ylabel('Accuracy');
        xlabel('Symmetry order');
        set(gca, 'XTick', [1,2,3,4,5])

        fig_save_name = ['/home/hjchang/Research/code/Matlab/cvpr2016/code/result/synthetic/graph/symmetryTest_3D'];
        
end

T = get(gca,'tightinset');
% set(gca,'position',[T(1) T(2) 1-T(1)-T(3) 1-T(2)-T(4)]);

hold off;

legend('boxoff');
set(gcf, 'color', 'w');
set(hfig, 'Position', [0 0 480 240])

export_fig(fig_save_name,'-pdf');










