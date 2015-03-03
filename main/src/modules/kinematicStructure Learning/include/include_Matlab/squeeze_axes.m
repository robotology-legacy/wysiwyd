function squeeze_axes(handles)
%
% squeeze_axes(handles) Squeeze axes to remove dead space.
%
%   Inputs:
%
% -> handles: the subplot axes handles organized as a grid. I.e.
% handles(1,1) is the axes in the first line and first column, whereas
% handles(4,4) is the axes in the forth line and forth column.
%

% - Creation Date: Mon, 16 Sep 2013 
% - Last Modified: Tue, 17 Sep 2013 
% - Author(s): 
%   - W.S.Freund <wsfreund_at_gmail_dot_com> 

% TODO: Make squeeze axes compatible with axes that occupy multiple
% subplot places.

nHorSubPlot =  size(handles,2);
nVertSubPlot = size(handles,1);

subplotWidth = 1/nHorSubPlot;
subplotHeight = 1/nVertSubPlot;

botPos = linspace(1-subplotHeight,0,nVertSubPlot);
leftPos = linspace(0,1-subplotWidth,nHorSubPlot);

for curLine=1:nVertSubPlot
  for curColumn=1:nHorSubPlot
    curAxes = handles(curLine,curColumn);
    if curAxes 
      % Set OuterPosition to occupy as most space as possible
      curAxesOuterPos = [leftPos(curColumn) botPos(curLine) subplotWidth ...
        subplotHeight];
      set(curAxes,'OuterPosition',curAxesOuterPos);
      % Remove dead space inside subplot border:
      curAxesTightPos=get(curAxes,'TightInset');
      noDeadSpacePos = curAxesOuterPos + [curAxesTightPos(1:2) ...
        -(curAxesTightPos(1:2) + curAxesTightPos(3:4))];
      set(curAxes,'Position',noDeadSpacePos)
    end
  end                                                         
end                                                           

end