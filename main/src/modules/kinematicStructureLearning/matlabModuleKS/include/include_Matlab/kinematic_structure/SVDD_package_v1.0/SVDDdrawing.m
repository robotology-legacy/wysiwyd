function SVDDdrawing(x,y,SET,kernel_param,kernel_type, class_idx)

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

ndata = SET.S.ndata + SET.E.ndata + SET.O.ndata;
x = [SET.S.x ; SET.E.x ; SET.O.x];
y = [SET.S.y ; SET.E.y ; SET.O.y];
alpha = [SET.S.alpha ; SET.E.alpha ; SET.O.alpha];
g = [SET.S.g ; SET.E.g ; SET.O.g];
SV = SET.S.x;
[R2, a] = boundary(x,y,alpha);

% global C;
% global kernel_param;
% global kernel_type;
% global alpha;
% global SV
% global SV_class_idx
% global a
% global R2
% global learning_type;
% global num_class
% global SET
% global g

% switch learning_type
%     case {'batch'}
%         [ndata, ndim] = size(x);
%     case {'incremental'}
%         [ndata, ndim] = size(SV);
% end

[ndata, ndim] = size(x);

data_min = min(x,[],1);
data_max = max(x,[],1);       

cVec = 'rbkgcmyrbkgcmybrkgcmybrkgcmybrkgcmybrkgcmy';

if ndim == 2    
    syms Z x1 y1;
    Z = [x1 y1];
    
    fij = 0;
    Lij = 0;        
    
    Lij = alpha' * genQmtx(x,y,kernel_type,kernel_param) * alpha;
    fij = alpha' * genQc(x,y,Z,1,kernel_type,kernel_param);
    
    f = 1-2*fij+Lij-R2;
    
%     figure(1)
    subplot(2,3,2);
    hold on;
    ezplot(f,[data_min(1)-10 data_max(1)+10 data_min(2)-10 data_max(2)+10]);
%     axis([0, width, 0, height]);
    title('Shape Description (SVDD)')
    
%     figure(1)
    subplot(2,3,2);
    hold on;
    plot(x(:,1),x(:,2),[cVec(class_idx) '.']);    
    plot(SV(:,1),SV(:,2),'bs');
    plot(SV(:,1),SV(:,2),'m.');
%     xlabel('\bf{X ��}','fontsize',12);
%     ylabel('\bf{Y ��}','fontsize',12);    
%     axis([0, width, 0, height]);
    axis equal;
%     title('Support Vector Domain Description 2 Dimension');     
%     hold off;
    
elseif ndim == 3
    syms Z x1 y1 z1;
    Z = [x1 y1 z1];
    
    fij = 0;
    Lij = 0;        

    Lij = alpha' * genQmtx(x,y,kernel_type,kernel_param) * alpha;
    fij = alpha' * genQc(x,y,Z,1,kernel_type,kernel_param);
    
    f = 1-2*fij+Lij-R2;

    figure(100)    
    hold on;
    plot3(x(:,1),x(:,2),x(:,3),[cVec(class_idx) '.']);    
    plot3(SV(:,1),SV(:,2),SV(:,3),'gs');
    SVDDdrawing_3D(f,class_idx,data_min,data_max,cVec);
    
    xlabel('\bf{X ��}','fontsize',12);
    ylabel('\bf{Y ��}','fontsize',12);
    zlabel('\bf{Z ��}','fontsize',12);
    title('Support Vector Domain Description 3 Dimension');     
    grid on;
    hold off;
    axis equal;
else
    printf('It cannot be drawn!! Because the dimension of data is not 2 or 3!!');
end