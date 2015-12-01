% ==========================================================================
% Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
% Authors: Hyung Jin Chang
% email:   (hj.chang@imperial.ac.uk)
% Permission is granted to copy, distribute, and/or modify this program
% under the terms of the GNU General Public License, version 2 or any
% later version published by the Free Software Foundation.
% 
% A copy of the license can be found at
% wysiwyd/license/gpl.txt
% 
% This program is distributed in the hope that it will be useful, but
% WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
% Public License for more details
% ==========================================================================

% Parameter setting
MAX_COUNT = 1000;
needToInit = true;
nightMode = false;
saveMode = false;
videoMode = 1;  % 0: webcam, 1: video

detect_interval = 3;
frame_idx = 0;
num_init_points = 0;

prev_points = [];
cur_points = [];

image = [];
cur_Gray = [];
prev_Gray = [];
mask = [];

% subPixWinSize = [5,5];
subPixWinSize = [10,10];
termcrit = struct('type','Count+EPS', 'maxCount', 20, 'epsilon', 0.03);
