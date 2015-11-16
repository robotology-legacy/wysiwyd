""" 
Import necessary libraries
"""
import matplotlib as mp
# Use this backend for when the server updates plots through the X 
mp.use('TkAgg')
import numpy as np
import pylab as pb
import GPy
# To display figures once they're called
pb.ion()
default_seed = 123344
import pods
#from SAM import SAM
import SAM

import yarp


# create ports
def createPorts():
    global leftInputPort
    global rightInputPort

    leftInputPort = yarp.Port()
    rightInputPort = yarp.Port()
    leftInputPort.open("/sam/cam/left")
    rightInputPort.open("/sam/cam/right")

    global outputCamerasRealTime

    outputCamerasRealTime = yarp.Port()
    outputCamerasRealTime.open("/realTime/cam")    # this port send the path and name of current image from left camera

    global camerasInfoBottle
    camerasInfoBottle = yarp.Bottle()

    return True

# connect ports to iCub cameras
def connectPorts():
    if not( yarp.Network.connect("/icub/cam/left", "/interface/cam/left") ):
        return False

    if not( yarp.Network.connect("/icub/cam/right", "/interface/cam/right") ):
        return False

    return True

# disconnect ports from iCub cameras
def disconnectPorts():
    yarp.Network.disconnect("/icub/cam/left", "/sam/cam/left")
    yarp.Network.disconnect("/icub/cam/right", "/sam/cam/right")
    return True


# initialise Yarp
yarp.Network.init()


# Instantiate object
a=SAM.LFM()

# disconnect ports
disconnectPorts()

