Unsupervised Kinematic Structure Learning
-----------------------------------------

This directory contains a set of codes for kinematic structure learning develeped at the Personal Robotics Laboratory in Imperial College London, UK. This module is mainly developed in Matlab with OpenCV. 

===================
MODULE REQUIREMENTS
===================
In order to run the module properly, it is required to install the followings:
- Matlab
- R
- OpenCV 2.3.x
- YARP

Then, the OpenCV and YARP should be connected to Matlab.

1. Matlab + OpenCV
You can use 'mexopencv' for this. 
Here is a well-described manual for the mexopencv setting:
http://vision.is.tohoku.ac.jp/~kyamagu/software/mexopencv/

2. Matlab + YARP
Here are instructions for calling YARP from Matlab:
http://wiki.icub.org/wiki/Calling_yarp_from_Matlab


=====================
HOW TO RUN THE MODULE
=====================
1. Loading input data from 'Cam/Video/Images'
	a) Run the 'main_solely.m' code
	b) Select a input source by the number. (Note: The '2:Yarp' is not working)
	c) Select a video or a folder of images.

2. Loading input data from 'YARP/ABM'
	a) Run the ABM server
	b) Check the yarp namespace for ABM
	c) Run the 'main_YARP_trigger.m' code
	d) $ yarp rpc /matlab/kinematicStructure/rpc
	e) $ startStructureLearning #num_instance [left or right camera] [first frame] [last frame]
	f) Closeing module: $ quit 

******* IMPORTANT ********
You should be very carefull with closing the module. If you forced to stop the matlab, then you should type in 'portTrigger.close' in the Command Window of Matlab for closing the module. Otherwise the Matlab will be frozen.
**************************


-----------------------------------------
When using these codes, please give credit to Personal Robotics Laboratory.
A convenient reference is the unsupervised kinematic structure learning work:

@inproceedings{ChangCVPR2015KinematicStructure,
	author = {Hyung Jin Chang and Yiannis Demiris},
	title = {Unsupervised Learning of Complex Articulated Kinematic Structures
	combining Motion and Skeleton Information},
	booktitle = {The IEEE Conference on Computer Vision and Pattern Recognition (CVPR)},
	month = {June},
	year = {2015}
}

The paper is available from http://www3.imperial.ac.uk/personalrobotics
Also the video result is availabe at YouTube - https://www.youtube.com/watch?v=l431rT0S29U

If you have any question, please email Hyung Jin Chang: hj.chang@imperial.ac.uk
