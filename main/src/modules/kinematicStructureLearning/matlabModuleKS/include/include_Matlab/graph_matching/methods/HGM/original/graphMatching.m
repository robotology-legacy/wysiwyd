function [X,Z,Y] = graphMatching (G1, G2, sigma, numberOfMatches)
% function [X,Z,Y] = graphMatching (G1, G2, sigma, numberOfMatches)
%
% Optimal soft graph matching. Use exp((-d.*d) ./ sigma) as a similarity
% between edges, where d is the difference between the edges weight.
%
% Algorithm due to R. Zass and A. Shashua.,
% 'Probabilistic Graph and Hypergraph Matching.',
% Computer Vision and Pattern Recognition (CVPR) Anchorage, Alaska, June 2008.
%
% G1 - An n1 by n1 symmetric matrix, with the weight of the first graph edges.
% G2 - An n2 by n2 symmetric matrix, with the weight of the second graph edges.
% sigma - Kernel parameter for edge-to-edge correlations.
% numberOfMatches - number of matches required. 
%
% X [Output] - an n1 by n2 matrix with the hard matching results.
%             The i,j entry is one iff the i-th feature of the first object
%             match the j-th feature of the second object. Zero otherwise.
% Z [Output] - an n1 by n2 matrix with the soft matching results.
%             The i,j entry is the probablity that the i-th feature of the
%             first object match the j-th feature of the second object.
% Y [Output] - Debug information.
%
% See also:
% - hypergraphMatching() for matching two hypergraphs (rather than graphs).
%
% Author: Ron Zass, zass@cs.huji.ac.il, www.cs.huji.ac.il/~zass

    % Verify input:
    if (nargin < 4)
        error 'graphMatching() requires four parameters.';
    end
    n1 = size(G1,1);
    n2 = size(G2,1);
    if (n1 ~= size(G1,2) || n2 ~= size(G2,2))
        error 'G1 and G2 must be square matrices.';
    end
    d1 = abs(G1 - G1');
    d2 = abs(G2 - G2');
    if (max(d1(:)) > 1e-6 || max(d2(:)) > 1e-6)
        error 'G1 and G2 must be symmetric.';
    end
    clear d1 d2

    % Marginalize the matching scores into Y:
    G2t = G2';
    Y = zeros(n1,n2);
    for i = 1 : n1
        for j = 1 : n2
            d = repmat(G1(:,i),1,n2) - repmat(G2t(j,:),n1,1);
            Y = Y + exp((-d.*d) ./ sigma);
        end
    end

    % Do hypergraph matching over Y:
    [X,Z] = hypergraphMatching (Y, numberOfMatches);

