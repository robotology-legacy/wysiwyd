wysiwyd : What You Say Is What You Did
=======

The WYSIWYD project will create a new transparency in human robot interaction (HRI) by allowing robots to both understand their own actions and those of humans, and to interpret and communicate these in human compatible intentional terms expressed as a language-like communication channel we call WYSIWYD Robotese (WR).

## Documentation
Visit the official Project [wiki](http://wiki.icub.org/wysiwyd/dox/html/index.html) for the software.

## CI Build
- Linux: [![Build Status](https://travis-ci.org/robotology/wysiwyd.png?branch=master)](https://travis-ci.org/robotology/wysiwyd)
- Windows: [![Build status](https://ci.appveyor.com/api/projects/status/4rckcp8suov8pcv1)](https://ci.appveyor.com/project/pattacini/wysiwyd)

## Build for tracking feature in iol2opc

To enable the new feature you have to go through the following steps.

### Build OpenCV-3.0.0

We need the new tracking features delivered with `OpenCV-3.0.0`:

1. Download `OpenCV-3.0.0`: `git clone https://github.com/Itseez/opencv.git`.
2. Checkout the correct branch: `git checkout 3.0.0`.
3. Download the external modules: `git clone https://github.com/Itseez/opencv_contrib.git`.
4. Checkout the correct branch: `git checkout 3.0.0`.
5. Configure `OpenCV` by filling in the cmake var **`OPENCV_EXTRA_MODULES_PATH`** with the path pointing to `opencv_contrib/modules` and then toggling on the var **`BUILD_opencv_tracking`**.
6. Compile `OpenCV`.

### Build WYSIWYD

Configure the project:

1. Fill in the cmake var **`OpenCV_DIR`** with the path to `OpenCV-3.0.0` build.
2. Toggle on the cmake option **`WYSIWYD_IOL2OPC_WITH_TRACKING`**. Note that if this options is `OFF`, then we do not need `OpenCV-3.0.0` and the project still compiles with the previous versions.
3. Compile `wysiwyd`.

### Building notes

It might be the case that you have also to build [**`kinect-wrapper`**](https://github.com/robotology/kinect-wrapper) with the new `OpenCV` library, as it was for me too. In particular, the setting I used foresees `yarp`, `icub-main` and all the other projects built with `OpenCV-2.x.y`, whereas `wysiwyd` along with `kinect-wrapper` built with `OpenCV-3.0.0`. This means that you can be perfectly working with two versions of the library available side by side. To ease compilation, I have also enabled the possibility to build only the client part of the `kinect-wrapper` (see [**updated instructions**](https://github.com/robotology/kinect-wrapper#cmaking-the-project)).

### How to use tracking

From an high-level perspective, the behavior is very simple: once the segmentation pipeline stops providing blobs due to big occlusions in the scene, then a tracker takes over it and keeps updating the object localization for a **tune-able time window**. 

To regulate the temporal width of such a window, play with the **`tracker_timeout`** parameter of the `iol2opc` module, which is by default set to `5.0 s`.
