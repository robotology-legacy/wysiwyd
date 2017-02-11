nPlot = 0; 
%% 1
if 1
    nPlot = nPlot + 1;
    plotOption(nPlot).strName = 'NETAL';
    % Display option
    plotOption(nPlot).color = 'm'; % color
    plotOption(nPlot).lineStyle = '--'; % linestyle
    plotOption(nPlot).marker = 'd'; % linestyle
end
%% 2
if 1
    nPlot = nPlot + 1;    
    plotOption(nPlot).strName = 'MAGNA++';
    % Display option
%     plotOption(nPlot).color = [0.5 0 1]; % color
%     plotOption(nPlot).lineStyle = '-'; % linestyle
%     plotOption(nPlot).marker = 'o'; % linestyle
    plotOption(nPlot).color = 'g'; % color
    plotOption(nPlot).lineStyle = '-.'; % linestyle
    plotOption(nPlot).marker = '^'; % linestyle
end
%% 3
if 1
    nPlot = nPlot + 1;    
    plotOption(nPlot).strName = 'Proposed 1st term only';
    % Display option
    plotOption(nPlot).color = [1 0.5 0.5]; % color
    plotOption(nPlot).lineStyle = ':'; % linestyle
    plotOption(nPlot).marker = 'x'; % linestyle    
%     plotOption(nPlot).color = 'g'; % color
%     plotOption(nPlot).lineStyle = ':'; % linestyle
%     plotOption(nPlot).marker = 'x'; % linestyle    
end
%% 4
if 1
    nPlot = nPlot + 1;
    plotOption(nPlot).strName = 'Proposed 2nd term only';
    % Display option
%     plotOption(nPlot).color = 'b'; % color
%     plotOption(nPlot).lineStyle = '-'; % linestyle
%     plotOption(nPlot).marker = 's'; % linestyle
    plotOption(nPlot).color = [0.5 0.6 0.7]; % color
    plotOption(nPlot).lineStyle = ':'; % linestyle
    plotOption(nPlot).marker = '+'; % linestyle    
end
%% 5
if 1   
    nPlot = nPlot + 1;    
    plotOption(nPlot).strName = 'Proposed 3rd term only';
    % Display option
%     plotOption(nPlot).color = 'c'; % color
%     plotOption(nPlot).lineStyle = '-.'; % linestyle
%     plotOption(nPlot).marker = 'x'; % linestyle
    plotOption(nPlot).color = [0 0.5 0.9]; % color
    plotOption(nPlot).lineStyle = ':'; % linestyle
    plotOption(nPlot).marker = '*'; % linestyle    

end
%% 6
if 1
    nPlot = nPlot + 1;
    plotOption(nPlot).strName = 'Proposed all';
    % Display option
    plotOption(nPlot).color = 'r'; % color
    plotOption(nPlot).lineStyle = '-.'; % linestyle
    plotOption(nPlot).marker = 's'; % linestyle        
end
%% 7
if 1
    nPlot = nPlot + 1;
    plotOption(nPlot).strName = '7';
    % Display option
    plotOption(nPlot).color = 'y'; % color
    plotOption(nPlot).lineStyle = '-'; % linestyle
    plotOption(nPlot).marker = '*'; % linestyle
end
%% 8
if 1
    nPlot = nPlot + 1;
    plotOption(nPlot).strName = '8';
    % Display option
    plotOption(nPlot).color = [1 0.5 0.5]; % color
    plotOption(nPlot).lineStyle = ':'; % linestyle
    plotOption(nPlot).marker = 's'; % linestyle
end
%% 9
if 1
    nPlot = nPlot + 1;
    plotOption(nPlot).strName = '9';
    % Display option
    plotOption(nPlot).color = [0.5 0.6 0.7]; % color
    plotOption(nPlot).lineStyle = ':'; % linestyle
    plotOption(nPlot).marker = 'x'; % linestyle
end
%% 10
if 1
    nPlot = nPlot + 1;
    plotOption(nPlot).strName = '10';
    % Display option
    plotOption(nPlot).color = [0 0.5 0.9]; % color
    plotOption(nPlot).lineStyle = ':'; % linestyle
    plotOption(nPlot).marker = 'x'; % linestyle
end
%%
clear nPlot

%%
plotSet.lineWidth = 3;
plotSet.markerSize = 10;
plotSet.fontSize = 20;
% plotSet.font = '\fontname{times new roman}';
plotSet.font = '\fontname{Arial}';