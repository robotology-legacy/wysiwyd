Unsupervised Kinematic Structure Learning & Correspondence Matching
-------------------------------------------------------------------

This directory contains a set of codes for kinematic structure learning and kinematic structure correspondence matching develeped at the Personal Robotics Laboratory in Imperial College London, UK. This module is mainly developed in Matlab and R with OpenCV. 


===================
MODULE REQUIREMENTS
===================
In order to run the module properly, it is required to install the followings:
+ Matlab R2014b or above
	>> Graph matching library installation
	>> run 'include/include_Matlab/graph_matching/compile.m' to compile all C-MEX files
+ R 3.2.x
	>> R also requires following packages to be installed:
	>> - igraph: install.packages("igraph")
	>> - reshape2: install.packages("reshape2")
	>> - ggplot2: install.packages("ggplot2")
+ OpenCV 2.3.x or 2.4.x: sudo apt-get install libopencv-dev
+ YARP

Then, the OpenCV and YARP should be connected to Matlab.

1. Matlab + OpenCV
You can use 'mexopencv' for this. 
Here is a well-described manual for the mexopencv setting:
http://vision.is.tohoku.ac.jp/~kyamagu/software/mexopencv/

2. Matlab + YARP
Here are instructions for calling YARP from Matlab:
http://www.yarp.it/yarp_swig.html
or
	a. run ccmake, and trigger the YARP_COMPILE_BINDINGS and CREATE_JAVA and PREPARE_CLASS_FILES
	b. run make
	c. cd build/yarp; cp LoadYarp.class YarpImageHelper.class ..
	d. change classpath.txt to contain $YARP_ROOT/build (with $YARP_ROOT substituted with the actual path)
	e. change librarypath.txt to contain $YARP_ROOT/build/lib (with $YARP_ROOT substituted with the actual path)
	f. run matlab, and call LoadYarp

If there is an error of 'libstdc++.so.6: version `GLIBCXX_3.4.19' not found':
Use LD_PRELOAD when starting Matlab.

    This seems to be the most widely used solution that requires minimum effort.
    Run locate libstdc++.so.6 to find the library on your system.
    Use LD_PRELOAD=/located-path/libstdc++.so.6 matlab to run Matlab.


=====================
HOW TO RUN THE MODULE
=====================
1. Loading input data from 'Cam/Video/Images'
	a) Run the 'src/KinematicStructureCorrepondence/main_solely.m' code
	b) Select a input source by the number. (Note: The '2:Yarp' is not working)
	c) Input a number of motion segmentaion (0: adaptive, 0<: manual). 
	d) Select a video or a folder of images.

2. Loading input data from 'YARP/ABM'
	a) Run the ABM server
	b) Check the yarp namespace for ABM
	c) Run the 'main_YARP_trigger.m' code
	d) $ yarp rpc /matlab/kinematicStructure/rpc
	e) For Kinematic structure extraction:
		$ startStructureLearning #num_instance [left or right camera] [first frame] [last frame]
	f) To finding kinematic structure correspondences between two data:
		$ findCorrespondence #num_instance [left or right or kinect] #num_instance [left or right or kinect]
	g) The module closes when ''quit'' is received



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

If you have any question, please email to Hyung Jin Chang: hj.chang@imperial.ac.uk

