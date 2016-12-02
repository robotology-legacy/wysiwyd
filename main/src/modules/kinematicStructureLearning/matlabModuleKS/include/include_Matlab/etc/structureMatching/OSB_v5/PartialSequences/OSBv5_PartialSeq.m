% Longin Jan Latecki, latecki@temple.edu, June 2006; updated Jan 2010
% Updated Feb 2010 by Suzan Koknar-Tezel, tezel@sju.edu
%      Now allows seqences of different lengths, but the first and last element of the first sequence must be matched
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
%   jumpcost - the cost of jumps in t1 or t2 used to balance the similarity of elements. The number of jumps is mulitplied by jumpcost
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
%    [pathcost,indxrow,indxcol,jumpcost, d] = OSBv2(t1,t2,1,5,5,5)
%

function [pathcost,indxrow,indxcol,jumpcost,d] = OSBv5_PartialSeq(t1,t2,warpwin,queryskip,targetskip,jumpcost)
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
if ~exist('jumpcost')
    statmatx=min(matx');
    statmatx2=min(matx);
    jumpcost=min( mean(statmatx)+std(statmatx), mean(statmatx2)+std(statmatx2) )+ eps;
end

matxE = Inf(m+1,n+1);%we add one row and one column to matx to ensure that we can skip the last elements
matxE(1:m,1:n)=matx;
matxE(m+1,n+1)=0;
% calling the main function
[pathcost,indxrow,indxcol] = findpathDAG(matxE,warpwin,queryskip,targetskip,jumpcost);
pathcost=pathcost/length(indxrow); %we normalize path cost
d = sqrt(sum((t1(indxrow)-t2(indxcol)).^2))/length(indxrow);   %squared Euclidean distance of correspndoning elements


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
    for j=i:n-1 %index over columns. NOTE: Starting value is i to force first element of query to participate in matching
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
%weight %this is the final result, the cost of the cheapest path is in weight(m,n)

% collecting the indices of points on the cheapest path
% mincol is the index of the col of the last point on the cheapest path
[pathcost,mincol]=min(weight(m-1,2:end));           % pathcost: minimal value
mincol = mincol + 1;                                % add one to compensate for the "2:end" in the above stmt
minrow=m-1;
indxcol=[];
indxrow=[];
while (minrow>0 && mincol>0)
    indxcol=[ mincol indxcol];
    indxrow=[ minrow indxrow];
    mincoltemp=camefromcol(minrow,mincol);
    minrow=camefromrow(minrow,mincol);
    mincol=mincoltemp;
end
%indxcol(end) = [];
%indxrow(end) = [];
return