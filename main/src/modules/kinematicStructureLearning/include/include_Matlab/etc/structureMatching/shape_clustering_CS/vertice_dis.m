function [sc_cost, indxrow, indxcol]=vertice_dis(r1,r2, alpha);

[m1,n1]=size(r1);
[m2,n2]=size(r2);
M=zeros(n1,n2);

for i=1:n1;
    for j=1:n2;
%          M(i,j)=sum(exp(0.5*(r1(:,i)-r2(:,j)).^2./(r1(:,i)+r2(:,j))));
         M(i,j)=sum((r1(1:end-1, i)-r2(1:end-1, j)).^2./abs((r1(1:end-1, i)+r2(1:end-1, j)))) + alpha * (r1(end, i) - r2(end,j))^2 / abs(r1(end, i) + r2(end,j));
        %M(i,j)=sqrt(sum((r1(:,i)-r2(:,j)).^2)/50);
        %M(i,j) = dtw1D(r1(:,i),r2(:,j), 2);
        %M(i,j)=DtwDistance(r1(:,i),r2(:,j));
    end
end



%--------------
if n1>n2;
    M=M';
end
%-------------
costmat=M;
statmat=min(costmat');
jumpcost=mean(statmat)+std(statmat);
%[a1,b1]=min(costmat,[],1);
%[a2,b2]=min(costmat,[],2);
%sc_cost=max(mean(a1),mean(a2));
[pathcost,indxrow,indxcol] = findpathDAG1(costmat,2, 2, jumpcost*2.2);%2,2);
% [pathcost,indxrow,indxcol] = findpathDAG1(costmat,2, 2, jumpcost);
sc_cost=pathcost;
% d=0;
% for k=1:length(indxrow)
%     d=d+costmat(indxrow(k),indxcol(k));
% end
% sc_cost=d/length(indxrow);