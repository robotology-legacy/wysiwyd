function [R2,mu] = boundary(x,y,alpha,SV_idx)

%======== SVDD boundary calculating function ==========
% [Input]
%   x: training data
%   xk: support vector
%   alpha: Lagrange multiplier
%   kernel: kernel type
%   kern_param: kernel parameter
% [Output]
%   R2: R2 is the distance from the center of the sphere
%       a to the boundary
%
% Hyung jin Chang 06/13/2007
% hjchang@neuro.snu.ac.kr
%======================================================

global C;
global kernel_param;
global kernel_type;
% global alpha;
global SV
global SV_class_idx
global a
global R2
global learning_type; 
global num_class
global SET
global g


if nargin < 4
    xk = x(find(alpha > 0 & alpha < C),:);
    xk = xk(1,:);

    yk = y(find(alpha > 0 & alpha < C),:);
    yk = yk(1,:);
    
elseif nargin == 4
    xk = x(SV_idx,:);
    yk = y(SV_idx,:);
end
    
[ndata, ndim] = size(x);
% xk = SV(1,:);   % any support vector is ok



% Lij = 0;        % last term of equation
% Mij = 0;        % mid term of equation
% 
% Li = 0;         % last term of equation for a
% 
% for i=1:ndata
%     for j=1:ndata
%         Lij = Lij + alpha(i)*alpha(j)*y(i)*y(j)*Kernel_Function(x(i,:),x(j,:),kernel_type,kernel_param);
%     end
%     Mij = Mij + alpha(i) * y(i) * Kernel_Function(x(i,:),xk,kernel_type,kernel_param);
%     
%     Li = Li + alpha(i) * x(i,:) * y(i,:);
% end
% 
% R2 = Kernel_Function(xk,xk,kernel_type,kernel_param) - 2*Mij + Lij;

% a = Li;

% alphar = repmat(alpha,[1,ndata]);
% alphac = alphar';
% 
% R2 = 1 - 2 * alpha .* genQc(x,y,xk,yk,kernel_type,kernel_param)...
%     + alphar .* alphac .* genQmtx(x,y,kernel_type,kernel_param);



Mij = alpha' * genQc(x,y,xk,yk,kernel_type,kernel_param);
Lij = alpha' * genQmtx(x,y,kernel_type,kernel_param) * alpha;

R2 = 1 - 2*Mij + Lij;
mu = 1 - 2*Mij;


