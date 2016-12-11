Perspective Taking
======= 

Various packages which implement the results reported in Fischer and Demiris ICRA2016.

For installation, please

1. Install ROS. For this, follow the instructions at http://wiki.ros.org/ROS/Installation (make sure you install the proper version depending on your version of Ubuntu) and install the ros-*-desktop-full package.
2. Install catkin_tools: "sudo pip install -U catkin_tools"
3. Within "wysiwyd/main/src/modules/perspectiveTaking", run "catkin build".
4. Within "wysiwyd/main/src/modules/perspectiveTaking", run "source devel/setup.bash"; consider putting this (with absolute path, obviously) in your ~/.bashrc
5. "roslaunch perspective_taking perspective_taking.launch" and you are ready to go! You can enable / disable the various nodes at the top of the launch file.

If you use this module, please refer to the following paper:
```
@inproceedings{FischerICRA2016,
author = {Tobias Fischer and Yiannis Demiris},
title = {{Markerless Perspective Taking for Humanoid Robots in Unconstrained Environments}},
booktitle = {IEEE International Conference on Robotics and Automation},
pages = {3309--3316},
doi={10.1109/ICRA.2016.7487504},
year = {2016},
month={May}
}
```
