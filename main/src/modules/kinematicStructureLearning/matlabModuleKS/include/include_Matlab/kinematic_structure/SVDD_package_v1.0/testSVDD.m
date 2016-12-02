function result_vec = testSVDD(test_data,SET,kernel_param,kernel_type)

%=============== SVDD Boundary Drawing ===============
% [Input]
%   x: training data
%   SV: support vector
%   R2: R2 is the distance from the center of the sphere
%       a to the boundary
%   alpha: Lagrange multiplier
%   kernel: kernel type
%   kern_param: kernel parameter
%   class_idx: class label
%
% Hyung jin Chang 06/10/2008
% hjchang@neuro.snu.ac.kr
%======================================================

% ndata = SET.S.ndata + SET.E.ndata + SET.O.ndata;
% x = [SET.S.x ; SET.E.x ; SET.O.x];
% y = [SET.S.y ; SET.E.y ; SET.O.y];
% alpha = [SET.S.alpha ; SET.E.alpha ; SET.O.alpha];
ndata = SET.S.ndata;
x = [SET.S.x];
y = [SET.S.y];
alpha = [SET.S.alpha];
% g = [SET.S.g ; SET.E.g ; SET.O.g];
% SV = SET.S.x;
[R2, a] = boundary(x,y,alpha);

[n_test_data, n_test_dim] = size(test_data);

%%
Lij = alpha' * genQmtx(x,y,kernel_type,kernel_param) * alpha;
% fij = alpha' * genQc(x,y,test_data,1,kernel_type,kernel_param);
fij = alpha' * genQc_vec(x,y,test_data,1,kernel_type,kernel_param);

ones_vec = ones(n_test_data,1);
R2_vec = repmat(R2,[n_test_data,1]);
Lij_vec = repmat(Lij,[n_test_data,1]);
fij_vec = fij';

result_vec = ones_vec-2*fij_vec+Lij_vec-R2_vec;