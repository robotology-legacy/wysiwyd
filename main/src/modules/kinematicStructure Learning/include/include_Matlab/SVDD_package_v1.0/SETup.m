function SET = SETup(x,y,alpha,g,Sidx,Eidx,Oidx)

global SET
global C

index_S_alpha = find(alpha > 0 & alpha < C);
index_E_alpha = find(alpha == C);
index_O_alpha = find(alpha == 0);

index_S_g = find(g == 0);
index_E_g = find(g < 0);
index_O_g = find(g > 0);

% if index_S_alpha ~= index_S_g
%     disp('_____________________________S');
% end
% if index_E_alpha ~= index_E_g
%     disp('_____________________________E');
% end
% if index_O_alpha ~= index_O_g
%     disp('_____________________________O');    
% end

SET.S.x = x(index_S_alpha,:);
SET.S.y = y(index_S_alpha);
SET.S.alpha = alpha(index_S_alpha,:);
SET.S.g = g(index_S_alpha,:);
SET.S.ndata = size(index_S_alpha,1);

SET.E.x = x(index_E_alpha,:);
SET.E.y = y(index_E_alpha);
SET.E.alpha = alpha(index_E_alpha,:);
SET.E.g = g(index_E_alpha,:);
SET.E.ndata = size(index_E_alpha,1);

SET.O.x = x(index_O_alpha,:);
SET.O.y = y(index_O_alpha);
SET.O.alpha = alpha(index_O_alpha,:);
SET.O.g = g(index_O_alpha,:);
SET.O.ndata = size(index_O_alpha,1);

% SET.S.x = x(index_S_g,:);
% SET.S.y = y(index_S_g);
% SET.S.alpha = alpha(index_S_g,:);
% SET.S.g = g(index_S_g,:);
% SET.S.ndata = size(index_S_g,1);
% 
% SET.E.x = x(index_E_g,:);
% SET.E.y = y(index_E_g);
% SET.E.alpha = alpha(index_E_g,:);
% SET.E.g = g(index_E_g,:);
% SET.E.ndata = size(index_E_g,1);
% 
% SET.O.x = x(index_O_g,:);
% SET.O.y = y(index_O_g);
% SET.O.alpha = alpha(index_O_g,:);
% SET.O.g = g(index_O_g,:);
% SET.O.ndata = size(index_O_g,1);


