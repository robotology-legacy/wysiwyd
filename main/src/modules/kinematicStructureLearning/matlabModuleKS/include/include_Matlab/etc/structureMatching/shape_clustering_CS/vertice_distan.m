% function [sc_cost, indxrow, indxcol]=vertice_distan(r1,r2, alpha);
function [ sumMatch ]=vertice_distan(r1,r2, alpha, c1, c2);

% punish = 50;
[m1,n1]=size(r1);
[m2,n2]=size(r2);
M=zeros(n1,n2);

for i=1:n1
    for j=1:n2
%          M(i,j)=sum(exp(0.5*(r1(:,i)-r2(:,j)).^2./(r1(:,i)+r2(:,j))));
         M(i,j)=sum((r1(1:end-1, i)-r2(1:end-1, j)).^2./abs((r1(1:end-1, i)+r2(1:end-1, j)))) + alpha * (r1(end, i) - r2(end,j))^2 / abs(r1(end, i) + r2(end,j));
        %M(i,j)=sqrt(sum((r1(:,i)-r2(:,j)).^2)/50);
        %M(i,j) = dtw1D(r1(:,i),r2(:,j), 2);
        %M(i,j)=DtwDistance(r1(:,i),r2(:,j));
    end
end


if n1>n2;
    M2=M';
else 
    M2 = M;
end
%-------------
costmat=M2;
statmat=min(costmat');
jumpcost=mean(statmat)+std(statmat);
punish = jumpcost*1.2;
% punish = jumpcost*7;

nc1Length = length( c1 );
nc2Length = length( c2 );
for i = 1: nc1Length
    Match( i ) = M( c1( i ), c2( i ) );
end

sumMatch = sum(Match);
diffn1 = 0;
diffn2 = 0;
if nc1Length < n1
    diffn1 = n1 - nc1Length;
end
if nc1Length < n2
    diffn2 = n2 - nc2Length;
end
diff = diffn1+diffn2;
sumMatch = (diff*punish + sumMatch) / length(Match);