function [X,Z] = hypergraphMatching (Y, numberOfMatches)
% function [X,Z] = hypergraphMatching (Y, numberOfMatches)
%
% Optimal soft hyergraph matching.
%
% Algorithm due to R. Zass and A. Shashua.,
% 'Probabilistic Graph and Hypergraph Matching.',
% Computer Vision and Pattern Recognition (CVPR) Anchorage, Alaska, June 2008.
%
% Y - Marginalization of the hyperedge-to-hyperedge correspondences matrix.
% numberOfMatches - number of matches required.
%
% X [Output] - an n1 by n2 matrix with the hard matching results.
%             The i,j entry is one iff the i-th feature of the first object
%             match the j-th feature of the second object. Zero otherwise.
% Z [Output] - an n1 by n2 matrix with the soft matching results.
%             The i,j entry is the probablity that the i-th feature of the
%             first object match the j-th feature of the second object.
%
% See also:
% - graphMatching() as an example on how to use this for graphs with a
%  specific similarity function.
%
% Author: Ron Zass, zass@cs.huji.ac.il, www.cs.huji.ac.il/~zass

    % Verify input:
    if (nargin < 2)
        error 'hypergraphMatching() requires four parameters.';
    end

%    Y = Y - min(Y(:)) + eps;
    [n1,n2] = size(Y);

    maxRowSum = ones(n1,1);
    maxColumnSum = ones(1,n2);

    Z = nearestDSmax_RE (Y, maxRowSum, maxColumnSum, numberOfMatches);
    X = soft2hard (Z, numberOfMatches);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function hard = soft2hard (soft, matchesNo)

    hard = zeros(size(soft));
    for i = 1 : matchesNo
        maxSoft = max(soft,[],2);
        [dummy,r] = max(maxSoft);
        [val,c] = max(soft(r,:));
        if (val < 0)
            return;
        end
        hard(r,c) = 1;
        soft(r,:) = -Inf;
        soft(:,c) = -Inf;
    end

