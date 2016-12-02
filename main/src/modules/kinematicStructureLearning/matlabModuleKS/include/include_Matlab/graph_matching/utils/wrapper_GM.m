%% 2nd Order Methods
%% Makes current problem into Graph Matching form
function [X] = wrapper_GM(method, cdata)

% Make function evaluation script
str = ['feval(@' func2str(method.fhandle)];
for j = 1:length(method.variable), str = [str ',cdata.' method.variable{j} ]; end
if ~isempty(method.param), for i = 1:length(method.param), str = [str, ',method.param{' num2str(i) '}']; end; end
str = [str, ')'];

% Function evaluation & Excution time Check
X = eval(str);
