function incSVDD_drawing(x,y,SET,kernel_param,kernel_type,time_delay)

ndata = SET.S.ndata + SET.E.ndata + SET.O.ndata;
x = [SET.S.x ; SET.E.x ; SET.O.x];
y = [SET.S.y ; SET.E.y ; SET.O.y];
alpha = [SET.S.alpha ; SET.E.alpha ; SET.O.alpha];
g = [SET.S.g ; SET.E.g ; SET.O.g];

if nargin < 4
    time_delay = 0.1;
end

SV = SET.S.x;

% clf
% [R2, mu] = boundary(x,y,alpha);


% SVDDdrawing(x,y,SET,kernel_param,kernel_type,1);
% pause(time_delay)