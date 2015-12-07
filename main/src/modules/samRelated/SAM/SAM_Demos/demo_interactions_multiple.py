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
import numpy as np
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
    inputInteractionPort = yarp.Port()
    inputInteractionPort.open("/sam/face/rpc:i");

    inputBottle = yarp.Bottle();
    outputBottle = yarp.Bottle();

imgHNew = 200
imgWNew = 200
# Creates a SAMpy object
mySAMpy = SAMDriver_interaction(True, imgH = 400, imgW = 400, imgHNew = imgHNew, imgWNew = imgWNew,inputImagePort="/CLM/imageSeg/out")

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
model_num_iterations = 700
model_init_iterations = 2000
fname = modelPath + '/models/' + 'mActions_' + model_type + '_exp' + str(experiment_number) #+ '.pickle'

# Enable to save the model and visualise GP nearest neighbour matching
save_model=True
economy_save = True # ATTENTION!! This is still BETA!!
visualise_output=False
test_mode = False

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
    cur = SAMDriver_interaction(True, imgH = 400, imgW = 400, imgHNew = 200, imgWNew = 200,inputImagePort="/CLM/imageSeg/out", openPorts=False)
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

    cur.Ymean = Ymean_cur
    cur.Ystd = Ystd_cur
    # As above but for the labels
    #Lmean_cur = L_cur.mean()
    #Ln_cur = L_cur - Lmean_cur
    #Lstd_cur = Ln_cur.std()
    #Ln_cur /= Lstd_cur
    #Ltestn_cur = Ltest_cur - Lmean_cur
    #Ltestn_cur /= Lstd_cur

    cur.X=None
    cur.Y = {'Y':Yn_cur}
    cur.Ytestn = {'Ytest':Ytestn_cur}
    cur.Ltest = {'Ltest':Ltest_cur}

    fname_cur = fname + '_L' + str(i)
    cur.training(model_num_inducing, model_num_iterations, model_init_iterations, fname_cur, save_model, economy_save)
    mm.append(cur)
    ss = [];
    sstest = [];
for i in range(len(Lunique)):
    for j in range(len(Lunique)):
        ss = mm[i].SAMObject.familiarity(mm[j].Y['Y'])
        print('Familiarity of model ' + participantList[i] + ' given label: ' + participantList[j] + ' using training data is: ' + str(ss))
    print("")

print("")
print("")

for i in range(len(Lunique)):
    for j in range(len(Lunique)):
        sstest = mm[i].SAMObject.familiarity(mm[j].Ytestn['Ytest'])
        print('Familiarity of model ' + participantList[i] + ' given label: ' + participantList[j] + ' using testing data is: ' + str(sstest))
    print("")

#if yarpRunning:
 #   while( not(yarp.Network.isConnected("/speechInteraction/behaviour:o","/sam/face/interaction:i")) ):
 #       print "Waiting for connection with behaviour port..."
 #       pass

visualiseInfo = [];
# This is for visualising the mapping of the test face back to the internal memory
if visualise_output: 
    for i in range(len(Lunique)):
        ax = mm[i].SAMObject.visualise()
        visualiseInfo.append(dict())
        visualiseInfo[i]['ax']=ax
        ytmp = mm[i].SAMObject.recall(0)
        ytmp = numpy.reshape(ytmp,(mm[i].imgHeightNew,mm[i].imgWidthNew))
        #ytmp = numpy.reshape(ytmp,(mm[i].imgHeightNew,mm[i].imgWidthNew,3))
        fig_nn = pb.figure()
        pb.title('Training NN')
        pl_nn = fig_nn.add_subplot(111)
        ax_nn = pl_nn.imshow(ytmp, cmap=plt.cm.Greys_r)
        pb.draw()
        pb.show()
        visualiseInfo[i]['fig_nn']=fig_nn
    else:
        visualiseInfo[i]=None

# Read and test images from iCub eyes in real-time

#fig_input = pb.figure()
#subplt_input = fig_input.add_subplot(111)
numFaces = 1
minTestImages = 0
numImgs = 0
for i in range(len(participantList)):
    numImgs = mm[i].Ytestn['Ytest'].shape[0]
    if(i == 0):
        minTestImages = numImgs
    else:
        if(numImgs < minTestImages):
            minTestImages = numImgs

minTestImages = 10
result = np.zeros([len(participantList),minTestImages,len(participantList)])
responseIdx = np.zeros([result.shape[0],result.shape[1]])
responseVal = np.zeros([result.shape[0],result.shape[1]])
confusionMatrix = np.zeros([result.shape[0],result.shape[0]])

if(test_mode):
    fig_disp = pb.figure()
    pb.title('Current Testing Image')
    pl_nn = fig_disp.add_subplot(111)

    for i in range(result.shape[0]):
        print('Participant ' + str(i) + ': ' + participantList[i])
        print('')
        for j in range(result.shape[1]):
            
            for k in range(result.shape[2]):
                currTestData = mm[k].Ytestn['Ytest']
                currImage = np.reshape(currTestData[j,:],[1,currTestData[j,:].shape[0]])

                ytmp = numpy.reshape(currImage,(mm[i].imgHeightNew,mm[i].imgWidthNew))
                ax_nn = pl_nn.imshow(ytmp, cmap=plt.cm.Greys_r)
                fig_disp.canvas.draw()
                fig_disp.canvas.flush_events()

                resultTemp = mm[i].SAMObject.familiarity(currImage)
                result[i][j][k] = resultTemp

            #print(participantList[i] + ' Test Image ' + str(j) + ' familiarity =' + str(result[i][j][:]))
            maxIdx = np.argmax(result[i][j][:])
            responseIdx[i][j] = maxIdx
            responseVal[i][j] = result[i][j][maxIdx]


        for g in range(result.shape[0]):
            confusionMatrix[i][g] = 100*float(np.count_nonzero(responseIdx[i][:]==g))/float(responseIdx.shape[1])
            print('Percentage ' + participantList[i] + ' classified as ' + participantList[g] + ' = ' +  str(confusionMatrix[i][g]))
    #calculate optimal thresholds using familiarity thresholds from result matrix and confusion matrix percentages



else:
    while( True ):
        try: 
            if yarpRunning:
                print "Waiting for input"
                inputInteractionPort.read(inputBottle,True)

                print(inputBottle.get(0).asString() + 'received')

                if( inputBottle.get(0).asString() == "ask_name" ):

                    #testFace = numpy.zeros([numFaces,imgHNew*imgWNew*3])
                    testFace = numpy.zeros([numFaces,imgHNew*imgWNew])

                    for i in range(numFaces):
                        imageReceived = False
                        imageReceived = mySAMpy.readImageFromCamera()
                        if(imageReceived):
                            testFace[i,:] = mySAMpy.imageFlatten_testing
                        print "face" + str(i)
                    
                    #pp = mySAMpy.testing(testFace, choice, visualiseInfo)
                    ss=numpy.zeros(len(participantList))
                    for i in range(len(Lunique)):
                        testFacen = testFace;
                        #testFacen = testFace - testFace.mean()
                        #testFacen /= testFace.std()
                        #testFacen = testFace - mm[i].Ymean
                        #testFacen /= mm[i].Ystd
                        ss[i] = mm[i].SAMObject.familiarity(testFacen)
                        print('Familiarity with ' + participantList[i] + ' given current face is: ' + str(ss[i]))

                    outputBottle.clear()
                    #deciding response
                    maxIdx = np.argmax(ss)
                    maxVal = ss[maxIdx]
                    
                    threshold = 0.4
                    if(maxVal > threshold):
                        outputBottle.addString(participantList[maxIdx])
                        #outputBottle.addString("greg")
                    else:
                        #outputBottle.addString("Unknown")
                        outputBottle.addString("partner")
                    
                    print(outputBottle.get(0).asString())
                    #time.sleep(0.5)
                    #l = pp.pop()
                    #l.remove()
                    #pb.draw()
                    #pb.waitforbuttonpress(0.1)
                else:
                    outputBottle.clear()
                    outputBottle.addString("nack")
                    #outputBottle.addString("Greg")

                inputInteractionPort.reply(outputBottle)

            #del l
        except KeyboardInterrupt:
            print 'Interrupted'
            try:
                sys.exit(0)
            except SystemExit:
                os._exit(0)