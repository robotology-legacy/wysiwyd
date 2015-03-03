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

% initialise YARP
LoadYarp;
import yarp.Port
import yarp.BufferedPortImageRgb
import yarp.BufferedPortBottle
import yarp.Bottle
import yarp.Time
import yarp.ImageRgb
import yarp.Image
import yarp.PixelRgb
done = 0;
b = Bottle;

%creating ports
portReadingData  = BufferedPortBottle;          %port for reading "quit" signal
portReadingImage = BufferedPortImageRgb;        %Buffered Port for reading image
portSendingImage = Port;                        %port for sending image
%first close the port just in case
%(this is to try to prevent matlab from beuing unresponsive)
portReadingData.close;
portReadingImage.close;
portSendingImage.close;

%open the ports
disp('_____opening ports...');
portReadingData.open('/matlab/read');
disp('_____opened port /matlab/read');
pause(0.5);
portReadingImage.open('/matlab/img:i');
disp('_____opened port /matlab/img:i');
pause(0.5);
portSendingImage.open('/matlab/img:o');
disp('_____opened port /matlab/img:o');
pause(0.5);
disp('_____done.');

figure(1)
run_processing = false;

% command = 'yarp connect /icubSim/cam/left /matlab/img:i';
% [status,cmdout] = system(command);
% disp(cmdout);
% command = 'yarp connect /matlab/img:o /view/left';
% [status,cmdout] = system(command);
% disp(cmdout);
% command = 'yarp write ... /matlab/read';
% [status,cmdout] = system(command);
% disp(cmdout);




while(~done)%run until you get the quit signal
    
    disp('Start loop!');
    b = portReadingData.read();%use false to have a non blocking port
    disp(b)
    
    if (sum(size(b))~=0)
        if (strcmp(b.toString, 'go'))
            run_processing = true;
            done = 0;
        elseif  (strcmp(b.toString, 'quit'))
            run_processing = false;
            done = 1;
        end
    end
    
%     disp(run_processing);
    
    while run_processing
        disp('getting a yarp image..');
        yarpImage = portReadingImage.read(true);%get the yarp image from port
        disp('got it..');
        h=yarpImage.height;
        w=yarpImage.width;
        pixSize=yarpImage.getPixelSize();
        tool=YarpImageHelper(h, w);
        tic %start time
        IN = tool.getRawImg(yarpImage); %use leo pape image patch
        TEST = reshape(IN, [h w pixSize]); %need to reshape the matrix from 1D to h w pixelSize
        COLOR = uint8(zeros(h, w, pixSize)); %create an empty image with the correct dimentions
        r = cast(TEST(:,:,1),'uint8');  % need to cast the image from int16 to uint8
        g = cast(TEST(:,:,2),'uint8');
        b = cast(TEST(:,:,3),'uint8');
        COLOR(:,:,1)= r; % copy the image to the previoulsy create matrix
        COLOR(:,:,2)= g;
        COLOR(:,:,3)= b;
        time = toc;
        fprintf('receiving a yarp image took %f seconds \n', time);
        %
        % Do any type of processing
        disp('Showing image');
        figure(1)
        clf
        imshow(COLOR);
        %
%         tic
%         %send it back to yarp
%         img = yarp.ImageRgb(); %create a new yarp image to send results to ports
%         img.resize(w,h);   %resize it to the desired size
%         img.zero();        %set all pixels to black
%         COLOR = reshape(COLOR, [h*w*pixSize 1]); %reshape the matlab image to 1D
%         tempImg = cast(COLOR ,'int16');   %cast it to int16
%         img = tool.setRawImg(tempImg, h, w, pixSize); % pass it to the setRawImg function (returns the full image)
%         portSendingImage.write(img); %send it off
%         time = toc;
%         fprintf('converting back to yarp took %f seconds \n', time);
%         %checking for quit signal
    end
    pause(0.01);    
end

disp('Going to close the port');
portReadingData.close;
portReadingImage.close;
portSendingImage.close;
