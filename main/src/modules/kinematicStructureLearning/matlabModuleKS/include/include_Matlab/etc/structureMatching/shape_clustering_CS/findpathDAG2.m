% Longin Jan Latecki, latecki@temple.edu, June 2006
%

%--------------------------------------------------------------------------
% function [pathcost,indxrow,indxcol] = findpathDAG(matx,queryskip,corresWindow,jumpcost)
% find path with. min. energy
% input: difference matrix
% output: the cost of cheapest path, and indices of the cheapest path 
% The cheapest path is computed on DAG
%--------------------------------------------------------------------------
function [pathcost,indxrow,indxcol] = findpathDAG2(matx,queryskip,corresWindow,jumpcost)



[m,n]=size(matx);
weight=Inf(m,n); %the weight of the actually cheapest path 
camefromcol=zeros(m,n); %the index of the parent col where the cheapest path came from
camefromrow=zeros(m,n); %the index of the parent row where the cheapest path came from

weight(1,:)=matx(1,:); %initialize first row

%for each row i we compute the path cost in the next row i+1
for i=1:m-1 %index over rows
    stopj=min([n, i+corresWindow]);
    for j=i:stopj %index over columns
        stoprowjump=min([m, i+queryskip]);
        for rowjump=i+1:stoprowjump %index over rows
            stopk=min([n, j+1+corresWindow]);
            for k=j+1:stopk %index over columns
                newweight = ( (i/(i+1))*weight(i,j) + (1/(i+1))*( matx(rowjump,k) + (rowjump-i-1)*jumpcost ) ); %we favor shorter jumps by multiplying with (rowjump-i)
                if weight(rowjump,k) >= newweight
                    weight(rowjump,k) = newweight;
                    camefromcol(rowjump,k)=j;
                    camefromrow(rowjump,k)=i;
                end
            end
        end
    end
end

%camefromrow
%camefromcol
%weight

%collecting the idnices of points on the cheapest path 
%mincol is the index of the col of the last point on the cheapest path
[pathcost,i]=min(weight(m,:));           % pathcost: minimal value
%[minrow,mincol]=index2d(i,size(weight));    % minrow,mincol: index, weight(minrow,mincol)=v
minrow=m; mincol=i;
indxcol=[mincol];
indxrow=[minrow];
mincoltemp=camefromcol(minrow,mincol);
minrow=camefromrow(minrow,mincol);
mincol=mincoltemp;

while (minrow>0 && mincol>0)
    indxcol=[ mincol indxcol];
    indxrow=[ minrow indxrow];
    mincoltemp=camefromcol(minrow,mincol);
    minrow=camefromrow(minrow,mincol);
    mincol=mincoltemp;
end

return

% function [r,c]=index2d(index,sz)
% converts matlab 1d index of matrix into 2d index
% input: index,        sz: matrixsize
%usage
%[v,i]=min(M(:));                  % v: minimal value
%[r,c]=index2d(i,size(M));         % r,c: index, M(r,c)=v

function [r,c]=index2d(index,sz)
r=mod(index-1,sz(1))+1;
c=floor((index-1)/sz(1))+1;
return