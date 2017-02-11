% Longin Jan Latecki, latecki@temple.edu, June 2006; updated Jan 2010
% Updated June 2010 Suzan Koknar-Tezel, tezel@sju.edu
%
% This is an improved version of tsDAGjump4.m
% This fuction is symmetric and allows the skipping of the first and last
% elements of both the target and query sequences.

% The description of the tsDAGjump4.m function is presented in
% L. J. Latecki, Q. Wang, S. Koknar-Tezel, and V. Megalooikonomou. Optimal Subsequence Bijection.
% IEEE Int. Conf. on Data Mining (ICDM), Omaha, Nebraska, USA, October 2007

% The description of this function is presented in (submitted for review)
% S. Koknar-Tezel and Longin Jan Latecki. Sequence matching as subsequence bijection. 
% IEEE Trans. Pattern Analysis and Machine Intelligence (PAMI)

% Parameters:
%   t1 - the first (query) time series. A row vector.
%   t2 - the second (target) time series. A row vector.
%
% Optional Parameters:
%   warpwin - the warping window, same as that used in Dynamic Time Warping
%   queryskip - sets a restriction on how many consecutive elements of sequence t1 can be skipped in the matching
%   targetskip - sets a restriction on how many consecutive elements of sequence t2 can be skiped in the mataching
%             best set  warpwin=queryskip=targetskip
%   jumpcost - the cost of jumps in t1 or t2 used to balance the similarity of elements. The number of jumps is mulitplied by jumpcost.
%   if jumpcost==-1 or not present, then it will be estimated
%   fig ==1, a figure showing the correspndence will be shown
%
% Return values:
%    pathcost - the cost of the cheapest matching path between t1 and t2;
%               it is a combination of distances between corresponding elements and the panelties for jumping (skipping)
%    indxrow - the index of elements in a substring of t1 that best match t2
%    indxcol - the index of elements in a substring of t2 that best match t1
%    d  - squared Euclidean distance of correspndoning elements (no jump penalty)
%
% Example:
%    t1=[ 1 2 8 12 6 8]; t2=[ 1 2 9  3 3  5 9];
%    t1=[ 1 2 8 12 6 8.5]; t2=[ 1 2 9  3 3  5.5 9];
%    [pathcost,indxrow,indxcol,jumpcost, d] = OSBv5(t1,t2,5,5,5,-1,1)
%    %jumpcost is determined automatically in this example

function [pathcost,indxrow,indxcol,jumpcost,d] = OSBv5(t1,t2,warpwin,queryskip,targetskip,jumpcost,fig)
m=length(t1);
n=length(t2);
%if warpwin is not given then
if ~exist('warpwin')
    warpwin = Inf;
end
%if targetskip is not given then
if ~exist('queryskip')
    queryskip = Inf;
end
%if queryskip is not given then
if ~exist('targetskip')
    targetskip = Inf;
end

matx = zeros(m,n);
% create a distance matrix, each entry i,j contains squared Euclidean distance of i in t1 and j in t2
for r = 1:m
    matx(r,:) = (t2-t1(r)).^2;
end

%computing the jumpcost, it is symmetric
if ~exist('jumpcost') || jumpcost==-1
    statmatx=min(matx');
    statmatx2=min(matx);
    jumpcost=min( mean(statmatx)+std(statmatx), mean(statmatx2)+std(statmatx2) )+ eps;
end

matxE = Inf(m+2,n+2);      %we add one row and one column to beginning and end of matx 
                           %to ensure that we can skip the first and last elements
matxE(2:m+1,2:n+1)=matx;
matxE(1,1)=0;
matxE(m+2,n+2)=0;
% calling the main function
[pathcost,indxrow,indxcol] = findpathDAG(matxE,warpwin,queryskip,targetskip,jumpcost);
pathcost=pathcost/length(indxrow); %we normalize path cost
d = sqrt(sum((t1(indxrow)-t2(indxcol)).^2))/length(indxrow);   %squared Euclidean distance of correspndoning elements

if ~exist('fig')
        fig=0;
    end
    %plots the results
    if fig==1
        figure;
        hold on;
        %axis off;
        l1 = length(indxrow); %length of indxrow = length of indxcol
        shiftt1=5*std(t1);
        plot(t1+shiftt1,'r', 'LineWidth',2); % plots the query shape time series with red solid line
        shiftt2=0.1*std(t2);
        plot(t2+shiftt2, 'LineWidth',2); % plots the target shape time series with blue solid line
        for i=1:l1;
            x = [indxrow(i), indxcol(i)];
            y = [t1(indxrow(i))+shiftt1, t2(indxcol(i))+shiftt2];
            plot(x,y)
        end
        hold off;
    end

return

%--------------------------------------------------------------------------
% function [pathcost,indxrow,indxcol] = findpathDAG(matx,warpwin,queryskip,targetskip,jumpcost)
% finds path with min cost
% input: the extended difference matrix
% output: the cost of cheapest path, and indices of the cheapest path
% The cheapest path is computed on DAG
%--------------------------------------------------------------------------
function [pathcost,indxrow,indxcol] = findpathDAG(matx,warpwin,queryskip,targetskip,jumpcost)

[m,n]=size(matx); %this matx=matxE, thus it has one row an column more than the original matx above
weight=Inf(m,n); %the weight of the actually cheapest path
camefromcol=zeros(m,n); %the index of the parent col where the cheapest path came from
camefromrow=zeros(m,n); %the index of the parent row where the cheapest path came from

weight(1,:)=matx(1,:); %initialize first row
weight(:,1)=matx(:,1); %initialize first column

for i=1:m-1 %index over rows
    for j=1:n-1 %index over columns
        if abs(i-j)<=warpwin %difference between i and j must be smaller than warpwin
            stoprowjump=min([m, i+queryskip]);
            for rowjump=i+1:stoprowjump %second index over rows
                stopk=min([n, j+targetskip]);
                for k=j+1:stopk %second index over columns
                    %newweight = ( weight(i,j) +  matx(rowjump,k) + sqrt((rowjump-i-1)^2+(k-j-1)^2)*jumpcost) ;
                    newweight = ( weight(i,j) +  matx(rowjump,k) + ((rowjump-i-1)+(k-j-1))*jumpcost) ; %we favor shorter jumps by multiplying by jummpcost
                    if weight(rowjump,k) > newweight
                        weight(rowjump,k) = newweight;
                        camefromrow(rowjump,k)=i;
                        camefromcol(rowjump,k)=j;
                    end
                end
            end
        end
    end
end

% collecting the indices of points on the cheapest path
pathcost=weight(m,n);   % pathcost: minimal value
mincol=n;
minrow=m;
indxcol=[];
indxrow=[];
while (minrow>1 && mincol>1)
    indxcol=[ mincol indxcol];
    indxrow=[ minrow indxrow];
    mincoltemp=camefromcol(minrow,mincol);
    minrow=camefromrow(minrow,mincol);
    mincol=mincoltemp;
end
indxcol = indxcol(1:end-1);
indxrow = indxrow(1:end-1);
indxcol = indxcol-1;
indxrow = indxrow-1;
return
