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
try:
    import yarp
    yarpRunning = True
except ImportError:
    print 'WARNING! Yarp was not found! Switching to offline mode'
    yarpRunning = False

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
if yarpRunning:
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
economy_save = True # ATTENTION!! This is still BETA!!
visualise_output=True

# Reading face data, preparation of data and training of the model
mySAMpy.readData(root_data_dir, participant_index, pose_index)
(Yall, Lall, YtestAll, LtestAll) = mySAMpy.prepareData(model_type, Ntr, pose_selection, randSeed=experiment_number)

Lunique = numpy.unique(mySAMpy.L)
L_index = dict()
for i in range(len(Lunique)):
    L_index[Lunique[i]] = numpy.where(mySAMpy.L == i)[0]

mm = []

for i in range(len(Lunique)):
    print('# Considering label: ' + str(Lunique[i]))
    cur = SAMDriver_interaction(True, imgH = 400, imgW = 400, imgHNew = 200, imgWNew = 200,inputImagePort="/visionDriver/image:o")
    ##############
    Y_cur = Yall[L_index[i],:].copy()
    Ytest_cur = YtestAll[L_index[i],:].copy()
    L_cur = Lall[L_index[i],:].copy()
    Ltest_cur = LtestAll[L_index[i],:].copy()

    # Center data to zero mean and 1 std
    Ymean_cur = Y_cur.mean()
    Yn_cur = Y_cur - Ymean_cur
    Ystd_cur = Yn_cur.std()
    Yn_cur /= Ystd_cur
    # Normalise test data similarly to training data
    Ytestn_cur = Ytest_cur - Ymean_cur
    Ytestn_cur /= Ystd_cur

    # As above but for the labels
    #Lmean_cur = L_cur.mean()
    #Ln_cur = L_cur - Lmean_cur
    #Lstd_cur = Ln_cur.std()
    #Ln_cur /= Lstd_cur
    #Ltestn_cur = Ltest_cur - Lmean_cur
    #Ltestn_cur /= Lstd_cur

    cur.X=None     
    cur.Y = {'Y':Yn_cur}
    #self.data_labels = self.L.copy()
    ################################

    fname_cur = fname + '_L' + str(i)
    cur.training(model_num_inducing, model_num_iterations, model_init_iterations, fname_cur, save_model, economy_save)
    mm.append(cur)

for i in range(len(Lunique)):
    for j in range(len(Lunique)):
        ss = mm[i].SAMObject.familiarity(mm[j].Y['Y'])
        print('Familiarity for model: ' + str(i) + ' given label: ' + str(j) + ' is: ' + str(ss))



if yarpRunning:
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
    try: 
        if yarpRunning:       
            choice = inputInteractionPort.read(True)
            testFace = mySAMpy.readImageFromCamera()
        else:
            TODO
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

