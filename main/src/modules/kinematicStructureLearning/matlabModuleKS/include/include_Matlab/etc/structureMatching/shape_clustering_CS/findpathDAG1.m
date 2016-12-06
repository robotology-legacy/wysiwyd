function [pathcost,indxrow,indxcol] = findpathDAG1(matx,queryskip,corresWindow,jumpcost)

[m,n]=size(matx);
weight=Inf(m,n); %the weight of the actually cheapest path 
camefromcol=zeros(m,n); %the index of the parent col where the cheapest path came from
camefromrow=zeros(m,n); %the index of the parent row where the cheapest path came from
stepcount=ones(m,n); %counts the number of steps (corresponding pairs)

weight(1,:)=matx(1,:); %initialize first row

for i=1:m-1 %index over rows
    for j=1:n-1 %index over columns
        stoprowjump=min([m, i+queryskip]);
        for rowjump=i+1:stoprowjump %second index over rows
            stopk=min([n, j+corresWindow]);
            for k=j+1:stopk %second index over columns
               newweight = ( weight(i,j) +  matx(rowjump,k) + sqrt((rowjump-i-1)^2+(k-j-1)^2)*jumpcost) ; %we favor shorter jumps by multiplying with (rowjump-i-1)
                if (1/(stepcount(i,j)))*weight(rowjump,k) >= (1/(stepcount(i,j)+1))*newweight
                    weight(rowjump,k) = newweight;
                    camefromrow(rowjump,k)=i;
                    camefromcol(rowjump,k)=j;
                    stepcount(rowjump,k)=stepcount(i,j)+1;
                end
            end
        end
    end
end
weight=weight./stepcount;

%camefromrow
%camefromcol
%weight
%stepcount

%collecting the idnices of points on the cheapest path 
%mincol is the index of the col of the last point on the cheapest path
[pathcost,mincol]=min(weight(m,:));           % pathcost: minimal value
minrow=m; 
indxcol=[];
indxrow=[];
while (minrow>0 && mincol>0)
    indxcol=[ mincol indxcol];
    indxrow=[ minrow indxrow];
    mincoltemp=camefromcol(minrow,mincol);
    minrow=camefromrow(minrow,mincol);
    mincol=mincoltemp;
end

return
