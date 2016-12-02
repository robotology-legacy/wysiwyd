function [rot,t] = EssentialMatrixToCameraMatrix(e)
% Extracts the cameras from the essential matrix
%  
% Input:
%       - e is the 3x4 essential matrix.
%
% Output:
%       - p1 = [I | 0] is the first canonic camera matrix. 
%
%       - rot and t are the rotation and translation of the
%       second camera matrix from 4 possible solutions. One camera matrix
%       must be selected. We test with a single point to determine if it is
%       in front of both cameras is sufficient to decide between the four
%       different solutions for the camera matrix (pag. 259). 
%
%----------------------------------------------------------
%
% From 
%    Book: "Multiple View Geometry in Computer Vision",
% Authors: Hartley and Zisserman, 2006, [HaZ2006]
% Section: "Extraction of cameras from the essential matrix", 
% Chapter: 9
%    Page: 258
%
%----------------------------------------------------------
%      Author: Diego Cheda
% Affiliation: CVC - UAB
%        Date: 03/06/2008
%----------------------------------------------------------


% Decompose the matrix E by svd
[u, s, v] = svd(e);

%
w = [0 -1 0; 1 0 0; 0 0 1];
z = [0 1 0; -1 0 0; 0 0 1];

% 
% E = SR where S = [t]_x and R is the rotation matrix.
% E can be factorized as:
%s = u * z * u';

% Two possibilities:
rot1 = u * w  * v';
rot2 = u * w' * v';

% Two possibilities:
t1 = u(:,3) ./max(abs(u(:,3)));
t2 = -u(:,3) ./max(abs(u(:,3)));


% 4 possible choices of the camera matrix P2 based on the 2 possible
% choices of R and 2 possible signs of t.
rot(:,:,1) = rot1; 
t(:,:,1) = t1;

rot(:,:,2) = rot2; 
t(:,:,2) = t2;

rot(:,:,3) = rot1; 
t(:,:,3) = t2;

rot(:,:,4) = rot2; 
t(:,:,4) = t1;