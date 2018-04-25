wysiwyd : What You Say Is What You Did
=======

The WYSIWYD project will create a new transparency in human robot interaction (HRI) by allowing robots to both understand their own actions and those of humans, and to interpret and communicate these in human compatible intentional terms expressed as a language-like communication channel we call WYSIWYD Robotese (WR).

## Citation and video demonstration
If you use parts of this work, please cite our paper in the IEEE Transactions on Cognitive and Developmental Systems:
```
@article{MoulinFrierFischerTCDS2017,
author={Clement Moulin-Frier and Tobias Fischer and Maxime Petit and Gregoire Pointeau and Jordi-Ysard Puigbo and Ugo Pattacini and Sock Ching Low and Daniel Camilleri and Phuong Nguyen and Matej Hoffmann and Hyung Jin Chang and Martina Zambelli and Anne-Laure Mealier and Andreas Damianou and Giorgio Metta and Tony J. Prescott and Yiannis Demiris and Peter Ford Dominey and Paul F. M. J. Verschure},
journal={IEEE Transactions on Cognitive and Developmental Systems},
title={{DAC-h3: A Proactive Robot Cognitive Architecture to Acquire and Express Knowledge About the World and the Self}},
year={2017},
keywords={Cognitive Robotics;Distributed Adaptive Control;Human-Robot Interaction;Symbol Grounding;Autobiographical Memory},
pages={to appear},
}
```
The video demonstration can be found [here](http://wysiwyd.upf.edu/research).

## Underlying software framework
The core library and modules within WYSIWYD were polished and have been updated in the [iCub-HRI repository](https://github.com/robotology/icub-hri) (along with an article published in the Frontiers in Robotics and AI). Please consider using the code in this repository as it is more up-to-date.

## Documentation
Visit the official Project [wiki](http://wiki.icub.org/wysiwyd/dox/html/index.html) for the software.

## License
WYSIWYD software and documentation are distributed under the GPL.
The full text of the license agreement can be found in: [./license/gpl.txt](https://github.com/robotology/wysiwyd/blob/master/license/gpl.txt).

Please read this license carefully before using the WYSIWYD code.

## CI Build
- Linux: [![Build Status](https://travis-ci.org/robotology/wysiwyd.png?branch=master)](https://travis-ci.org/robotology/wysiwyd)
- Windows: [![Build status](https://ci.appveyor.com/api/projects/status/4rckcp8suov8pcv1)](https://ci.appveyor.com/project/pattacini/wysiwyd)

## Build

**`OpenCV-3.0.0`** or higher is a required dependency to build `iol2opc` module (**`OpenCV-3.2.0`** is recommended).

### Build OpenCV-3.2.0

We need the new tracking features delivered with `OpenCV-3.2.0`:

1. Download `OpenCV-3.2.0`: `git clone https://github.com/Itseez/opencv.git`.
2. Checkout the correct branch: `git checkout 3.2.0`.
3. Download the external modules: `git clone https://github.com/Itseez/opencv_contrib.git`.
4. Checkout the correct branch: `git checkout 3.2.0`.
5. Configure `OpenCV` by filling in the cmake var **`OPENCV_EXTRA_MODULES_PATH`** with the path pointing to `opencv_contrib/modules` and then toggling on the var **`BUILD_opencv_tracking`**.
6. Compile `OpenCV`.

### Build WYSIWYD

Configure the project:

1. Fill in the cmake var **`OpenCV_DIR`** with the path to `OpenCV-3.2.0` build.
2. Compile `wysiwyd`.

### Building notes

It might be the case that you have also to build [**`kinect-wrapper`**](https://github.com/robotology/kinect-wrapper) with the new `OpenCV` library, as it was for me too. In particular, the setting I used foresees `yarp`, `icub-main` and all the other projects built with `OpenCV-2.x.y`, whereas `wysiwyd` along with `kinect-wrapper` built with `OpenCV-3.2.0`. This means that you can be perfectly working with two versions of the library available side by side. To ease compilation, I have also enabled the possibility to build only the client part of the `kinect-wrapper` (see [**updated instructions**](https://github.com/robotology/kinect-wrapper#cmaking-the-project)).
