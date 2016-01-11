%%
 % Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 % Authors: Martina Zambelli, Hyung Jin Chang
 % email:   m.zambelli13@imperial.ac.uk, hj.chang@imperial.ac.uk
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
%%


clc    
close all
disp('start')

%% Prepare ports and bottles for yarp

LoadYarp;
import yarp.BufferedPortBottle
import yarp.Port
import yarp.Bottle
import yarp.Network

portReadTouch=Port;
portReadTouch.close;
portReadTouch.open('/matlab/readTouch');
disp('opened port /matlab/readTouch');
pause(0.5);

portReadJoints=Port;
portReadJoints.close;
portReadJoints.open('/matlab/readJoints');
disp('opened port /matlab/readJoints');
pause(0.5);

disp('done.');

connection_check = 0;
while(~connection_check)
    Network.connect('/sensoryProcessor/skin:o', '/matlab/readTouch');
    connection_check = Network.isConnected('/sensoryProcessor/skin:o','/matlab/readTouch');
    pause(1)
    disp('waiting for connection...');
end
connection_check = 0;
while(~connection_check)
    Network.connect('/sensoryProcessor/armencoders:o', '/matlab/readJoints');
    connection_check = Network.isConnected('/sensoryProcessor/armencoders:o','/matlab/readJoints');
    pause(1)
    disp('waiting for connection...');
end
disp('Connected!')


bJoints = yarp.Bottle;
bTouch = yarp.Bottle;

fileIDjoints = fopen('jointsICUB2.txt','a+');
fileIDtouch = fopen('touchICUB2.txt','a+'); 

h=figure('doublebuffer','on', ...
       'CurrentCharacter','a');
axis off
t=text(.5,.5,'Press ESC to quit.','Hor','center');

pause(.1)
while double(get(gcf,'CurrentCharacter'))~=27
            
        %% read the current touch sensing
        bTouch.clear;
        portReadTouch.read(bTouch);
        %disp(bTouch.toString);
        
        for tt = 1:bTouch.size
            fprintf(fileIDtouch, '%d\t ', bTouch.get(tt-1).asDouble);
        end
        fprintf(fileIDtouch, '\n ');                      
         
        %% read proprioceptive data
        bJoints.clear;
        portReadJoints.read(bJoints);
        %disp(bJoints.toString)
        
        for tt = 1:bJoints.size
            fprintf(fileIDjoints, '%d\t ', bJoints.get(tt-1).asDouble);
        end
        fprintf(fileIDjoints, '\n ');
        
        pause(0.1)

end
fclose(fileIDtouch);
fclose(fileIDjoints);
close(h)

%%
disp('Going to close the port');
portReadTouch.close;
portReadJoints.close;


%% Read txt files

joints = importdata('jointsICUB2.txt');
touch = importdata('touchICUB2.txt');

% joints = importdata('joints.txt');
% touch = importdata('touch.txt');
