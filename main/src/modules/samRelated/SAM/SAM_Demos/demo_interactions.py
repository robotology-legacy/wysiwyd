#!/usr/bin/python

#
#The University of Sheffield
#WYSIWYD Project
#
#Example of implementation of SAMpy class
#
#Created on 29 May 2015
#
#@authors: Uriel Martinez, Luke Boorman, Andreas Damianou
#
#

import matplotlib.pyplot as plt
from SAM.SAM_Drivers import SAMDriver_interaction
import pylab as pb
import sys
import pickle
import os
import numpy
import time
import operator
import yarp

from ConfigParser import SafeConfigParser


# Check configuration file
parser = SafeConfigParser()

file_candidates = ["config_faces.ini"]
section_candidates = ["config_options"]

configData = False

print 'Finding config file'
print '-------------------'
for loc in os.curdir, os.path.expanduser("~"), os.environ.get("WYSIWYD_DIR")+"/share/wysiwyd/contexts/visionDriver":
    print loc
    try:
        found = parser.read(os.path.join(loc,file_candidates[0]))
        if not found:
            pass
        else:
            pathFound = found
            print os.path.join(loc,file_candidates[0])
            if( parser.has_section(section_candidates[0]) == True ):
                dataPath = parser.get(section_candidates[0], 'data_path')
                modelPath = parser.get(section_candidates[0], 'model_path')
                participantList_val = parser.get(section_candidates[0], 'participants')
                participantList = participantList_val.split(',')
                configData = True
            else:
                print 'config_options not found...'
    except IOError:
        pass

if( configData == True ):
    print "config paths ready"
else:
    print "config paths failed"
    exit()

print '-------------------'
print 'Config file found: ' + pathFound[0]
print dataPath
print modelPath
print participantList
print '-------------------'


# Creates and opens ports for interaction with speech module
yarp.Network.init()
inputInteractionPort = yarp.BufferedPortBottle()
inputInteractionPort.open("/sam/face/interaction:i");
choice = yarp.Bottle();

# Creates a SAMpy object
mySAMpy = SAMDriver_interaction(True, imgH = 400, imgW = 400, imgHNew = 200, imgWNew = 200,inputImagePort="/visionDriver/image:o")

# Specification of the experiment number
experiment_number = 1007#42

# Location of face data
root_data_dir=dataPath

# Image format
image_suffix=".ppm"
# Array of participants to be recognised
participant_index=participantList

# Poses used during the data collection
pose_index=['Seg']
# Use a subset of the data for training
Ntr=300

# Pose selected for training
pose_selection = 0

# Specification of model type and training parameters
model_type = 'mrd'
model_num_inducing = 30
model_num_iterations = 150
model_init_iterations = 400
fname = modelPath + '/models/' + 'mActions_' + model_type + '_exp' + str(experiment_number) #+ '.pickle'

# Enable to save the model and visualise GP nearest neighbour matching
save_model=True
visualise_output=True

# Reading face data, preparation of data and training of the model
mySAMpy.readData(root_data_dir, participant_index, pose_index)
mySAMpy.prepareData(model_type, Ntr, pose_selection)
mySAMpy.training(model_num_inducing, model_num_iterations, model_init_iterations, fname, save_model)

while( not(yarp.Network.isConnected("/speechInteraction/behaviour:o","/sam/face/interaction:i")) ):
    print "Waiting for connection with behaviour port..."
    pass


# This is for visualising the mapping of the test face back to the internal memory
if visualise_output: 
    ax = mySAMpy.SAMObject.visualise()
    visualiseInfo=dict()
    visualiseInfo['ax']=ax
    ytmp = mySAMpy.SAMObject.recall(0)
    ytmp = numpy.reshape(ytmp,(mySAMpy.imgHeightNew,mySAMpy.imgWidthNew))
    fig_nn = pb.figure()
    pb.title('Training NN')
    pl_nn = fig_nn.add_subplot(111)
    ax_nn = pl_nn.imshow(ytmp, cmap=plt.cm.Greys_r)
    pb.draw()
    pb.show()
    visualiseInfo['fig_nn']=fig_nn
else:
    visualiseInfo=None

# Read and test images from iCub eyes in real-time

#fig_input = pb.figure()
#subplt_input = fig_input.add_subplot(111)


while( True ):
    pass

    try:        
        choice = inputInteractionPort.read(True)
        testFace = mySAMpy.readImageFromCamera()
        pp = mySAMpy.testing(testFace, choice, visualiseInfo)
        #time.sleep(0.5)
        l = pp.pop()
        l.remove()
        pb.draw()
        pb.waitforbuttonpress(0.1)
        #del l
    except KeyboardInterrupt:
        print 'Interrupted'
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)

