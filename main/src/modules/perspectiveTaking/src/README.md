Perspective Taking
======= 

Various packages which implement the results reported in Fischer and Demiris ICRA2016.

For installation, please
1) Install ROS. For this, follow the instructions at http://wiki.ros.org/indigo/Installation/Ubuntu and install the ros-indigo-desktop-full package.
2) Install catkin_tools: "sudo pip install -U catkin_tools"
2) Within "wysiwyd/main/src/modules/perspectiveTaking", run "catkin build".
3) Within "wysiwyd/main/src/modules/perspectiveTaking", run "source devel/setup.bash"; consider putting this (with absolute path, obviously) in your ~/.bashrc
4) "roslaunch perspective_taking perspective_taking.launch" and you are ready to go! You can enable / disable the various nodes at the top of the launch file.
