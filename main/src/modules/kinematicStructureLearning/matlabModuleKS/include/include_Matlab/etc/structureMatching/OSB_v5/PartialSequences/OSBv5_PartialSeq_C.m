% Longin Jan Latecki, latecki@temple.edu, June 2006; updated Jan 2010
% Updated Feb 2010 by Suzan Koknar-Tezel
%      Now allows seqences of different lengths, but the first and last element must be matched
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

function [pathcost,indxrow,indxcol,jumpcost,d] = OSBv5_PartialSeq_C(t1,t2,warpwin,queryskip,targetskip,jumpcost)
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

