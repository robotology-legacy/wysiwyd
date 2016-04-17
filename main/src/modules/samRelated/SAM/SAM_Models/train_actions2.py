from SAM.SAM_Drivers import SAMDriver_AR
import matplotlib
import matplotlib.pyplot as plt
import readline
import warnings
import GPy
from SAM.SAM_Core import SAMCore
from SAM.SAM_Core import SAMDriver
import pylab as pb
import sys 
from sys import executable
import subprocess
from subprocess import Popen, PIPE
import pickle
import os
from os import listdir, walk, system
from os.path import isfile, join, isdir
import time
import operator
import numpy

import numpy as np
import numpy.ma as ma
np.set_printoptions(threshold=numpy.nan)
import datetime
import yarp
import copy
from itertools import combinations 
from ConfigParser import SafeConfigParser
from scipy.spatial import distance
from numpy.linalg import inv
import math
import ipyparallel as ipp
import random
from sklearn.metrics import confusion_matrix


from IPython.display import clear_output
warnings.simplefilter("ignore")

def testFunc(data, lab):
        d = mySAMpy.testing(data, False)
        if(lab == d[0]):
            result = True
        else:
            result = False
        print 'Actual  ' + str(lab).ljust(11) + '  Model:  ' + str(d[0]).ljust(11) + '  with ' + str(d[1])[:6] + ' confidence: ' + str(result) + '\n'
        return d

yarpRunning = False
dataPath = sys.argv[1]
modelPath = sys.argv[2]
trainName = sys.argv[3]
mode = sys.argv[4]
singleModel = False

#participantList is extracted from number of subdirectories of dataPath
participantList = [f for f in listdir(dataPath) if isdir(join(dataPath, f))]
if (len(participantList) < 1):
    singleModel = True

off = 17
print '-------------------'
print 'Training Settings:'
print
print 'Data Path: '.ljust(off), dataPath
print 'Model Path: '.ljust(off),modelPath
print 'Participants: '.ljust(off),participantList
print 'Model Root Name: '.ljust(off), trainName
print 'Training Mode:'.ljust(off), mode
print '-------------------'
print 'Loading Parameters...'
print

try:
    parser = SafeConfigParser()
    found = parser.read(dataPath + "/config.ini")

    if(parser.has_option(trainName, 'update_mode')):
        modeConfig = parser.get(trainName, 'update_mode') 
    else:
        modeConfig = 'update'
except IOError:
    pass

if(mode == 'new' or modeConfig == 'new' or '.pickle' not in modelPath): #or update but no .pickle
    print 'Loading training parameters from: \n ', '\t' + dataPath + "/config.ini"
    try:
        parser = SafeConfigParser()
        found = parser.read(dataPath + "/config.ini")

        #load parameters from config file
        if(parser.has_option(trainName, 'experiment_number')):
            experiment_number = int(parser.get(trainName, 'experiment_number'))
        elif('.pickle' in modelPath):
            experiment_number = int(modelPath.split('__')[-2].replace('exp','')) + 1
        else:
            experiment_number = 0

        if(parser.has_option(trainName, 'ignoreLabels')):
            ignoreLabels = parser.get(trainName, 'ignoreLabels').split(',')
        else:
            ignoreLabels = ['agent_entry','agent_exit','no_agent']
        
        if(parser.has_option(trainName, 'ignoreParts')):
            ignoreParts = parser.get(trainName, 'ignoreParts').split(',')
        else:
            ignoreParts = ['partner']

        if(parser.has_option(trainName, 'actionsAllowedList')):
            actionsAllowedList = parser.get(trainName, 'actionsAllowedList').split(',')
        else:
            actionsAllowedList = ['lift_object','pull_object','push_object','drop_object','carry_object']
        
        if(parser.has_option(trainName, 'angleThreshold')):
            angleThreshold = float(parser.get(trainName, 'angleThreshold'))
        else:
            angleThreshold = 0.01

        if(parser.has_option(trainName, 'percentContactThreshold')):
            percentContactThreshold = float(parser.get(trainName, 'percentContactThreshold'))
        else:
            percentContactThreshold = 98.0
        
        if(parser.has_option(trainName, 'featuresToUse')):
            featuresToUse = parser.get(trainName, 'featuresToUse').split(',')
        else:
            featuresToUse = ['contact','selfMovementLabelK']

        if(parser.has_option(trainName, 'compressData')):
            compressData = parser.get(trainName, 'compressData')
        else:
            compressData = True
        
        if(parser.has_option(trainName, 'featuresToCompress')):
            featuresToCompress = parser.get(trainName, 'featuresToCompress').split(',')
        else:
            featuresToCompress = ['selfMovementLabelK']

        if(parser.has_option(trainName, 'maxNumItems')):
            maxNumItems = int(parser.get(trainName, 'maxNumItems'))
        else:
            maxNumItems = 20000

        if(parser.has_option(trainName, 'deltaDistanceThreshold')):
            deltaDistanceThreshold = float(parser.get(trainName, 'deltaDistanceThreshold'))
        else:
            deltaDistanceThreshold = 0.01

        if(parser.has_option(trainName, 'jointMu')):
            jointMu = float(parser.get(trainName, 'jointMu'))
        else:
            jointMu = 0
            
        if(parser.has_option(trainName, 'jointSig')):
            jointSig = float(parser.get(trainName, 'jointSig'))
        else:
            jointSig = 0.0000001
            
        if(parser.has_option(trainName, 'ratioData')):
            ratioData = int(parser.get(trainName, 'ratioData'))
        else:
            ratioData = 50

        if(parser.has_option(trainName, 'model_type')):
            model_type = parser.get(trainName, 'model_type')
        else:
            model_type = 'mrd'

        if(parser.has_option(trainName, 'model_num_inducing')):
            model_num_inducing = int(parser.get(trainName, 'model_num_inducing'))
        else:
            model_num_inducing = 30

        if(parser.has_option(trainName, 'model_num_iterations')):
            model_num_iterations = int(parser.get(trainName, 'model_num_iterations'))
        else:
            model_num_iterations = 700

        if(parser.has_option(trainName, 'model_init_iterations')):
            model_init_iterations = int(parser.get(trainName, 'model_init_iterations'))
        else:
            model_init_iterations = 2000

        if(parser.has_option(trainName, 'verbose')):
            verbose = parser.get(trainName, 'verbose')
        else:
            verbose = False

        if(parser.has_option(trainName, 'Quser')):
            Quser = int(parser.get(trainName, 'Quser'))
        else:
            Quser = 2

        if(parser.has_option(trainName, 'kernelString')):
            kernelString = parser.get(trainName, 'kernelString')
        else:
            kernelString = "GPy.kern.RBF(Q, ARD=False) + GPy.kern.Bias(Q) + GPy.kern.White(Q)"

    except IOError:
        pass
else:
    print 'Loading parameters from: \n ','\t' + modelPath
    try:
        parser = SafeConfigParser()
        found = parser.read(dataPath + "/config.ini")

        #load parameters from config file
        if(parser.has_option(trainName, 'experiment_number')):
            experiment_number = int(parser.get(trainName, 'experiment_number'))
        else:
            experiment_number = int(modelPath.split('__')[-2].replace('exp',''))
    except IOError:
        pass

    modelPickle = pickle.load(open(modelPath ,'rb'))
    ignoreLabels = modelPickle['ignoreLabels']
    ignoreParts = modelPickle['ignoreParts']
    actionsAllowedList = modelPickle['actionsAllowedList']
    angleThreshold = modelPickle['angleThreshold']
    percentContactThreshold = modelPickle['percentContactThreshold']
    featuresToUse = modelPickle['featuresToUse']
    compressData = modelPickle['compressData']
    featuresToCompress = modelPickle['featuresToCompress']
    maxNumItems = modelPickle['maxNumItems']
    deltaDistanceThreshold = modelPickle['deltaDistanceThreshold']
    jointMu = modelPickle['jointMu']
    jointSig = modelPickle['jointSig']
    ratioData = modelPickle['ratioData']
    model_type = modelPickle['model_type']
    model_num_inducing = modelPickle['model_num_inducing']
    model_num_iterations = modelPickle['model_num_iterations']
    model_init_iterations = modelPickle['model_init_iterations']
    verbose = modelPickle['verbose']
    Quser = modelPickle['Quser']
    kernelString = modelPickle['kernelString']
    
# # Creates a SAMpy object
mySAMpy = SAMDriver_AR.SAMDriver_AR(False)

if('.pickle' in modelPath):
    fname = '/'.join(modelPath.split('/')[:-1]) + '/' + dataPath.split('/')[-1] + '__' + trainName + '__' +  model_type + '__exp' + str(experiment_number)
else:
    fname = modelPath + dataPath.split('/')[-1] + '__' + trainName + '__' +  model_type + '__exp' + str(experiment_number) #+ '.pickle'

print 'Full model name: \n', '\t' + fname
print '-------------------'
print

save_model = False
economy_save = True
visualise_output = False
test_mode = True
mySAMpy.ignoreLabels = ignoreLabels
mySAMpy.ignoreParts = ignoreParts
mySAMpy.actionsAllowedList = actionsAllowedList
mySAMpy.angleThreshold = angleThreshold
mySAMpy.percentContactThreshold = percentContactThreshold
mySAMpy.featuresToUse = featuresToUse
mySAMpy.compressData = compressData
mySAMpy.featuresToCompress = featuresToCompress
mySAMpy.maxNumItems = maxNumItems
mySAMpy.deltaDistanceThreshold = deltaDistanceThreshold
mySAMpy.jointMu = jointMu
mySAMpy.jointSig = jointSig
mySAMpy.Quser = Quser
mySAMpy.verbose = verbose

# # Reading action data
mySAMpy.readData(dataPath, participantList)

minData = mySAMpy.Y.shape[0]

print mySAMpy.Y.shape
print 'minData = ' + str(minData)
print 'ratioData = ' + str(ratioData)
Ntr = ratioData

[Yall,Lall,YtestAll,LtestAll] = mySAMpy.prepareData(model_type, Ntr, randSeed=experiment_number)
print "Training with ", model_num_inducing, 'inducing points for ', model_init_iterations, '|', model_num_iterations
mySAMpy.training(model_num_inducing, model_num_iterations, model_init_iterations, fname, save_model, economy_save, keepIfPresent = False)

if visualise_output: 
    ax = mySAMpy.SAMObject.visualise()
    visualiseInfo=dict()
    visualiseInfo['ax']=ax
else:
    visualiseInfo=None

from SAM.SAM_Drivers import testingSegments

yTrainingData = mySAMpy.formatDataFunc(Yall)
testingSegments.testSegments(mySAMpy, yTrainingData, Lall)

yTrainingData = mySAMpy.formatDataFunc(YtestAll)
testingSegments.testSegments(mySAMpy, yTrainingData, LtestAll)
print
#save model with custom .pickle dictionary by iterating through all nested models
fname_cur = fname
print '-------------------'
print 'Saving: ' + fname_cur
extraParams = dict()
extraParams['YALL'] = Yall
extraParams['LALL'] = Lall
extraParams['YTEST'] = YtestAll
extraParams['LTEST'] = LtestAll
extraParams['ignoreLabels'] = mySAMpy.ignoreLabels
extraParams['ignoreParts'] = mySAMpy.ignoreParts
extraParams['actionsAllowedList'] = mySAMpy.actionsAllowedList
extraParams['angleThreshold'] = mySAMpy.angleThreshold
extraParams['percentContactThreshold'] = mySAMpy.percentContactThreshold
extraParams['featuresToUse'] = mySAMpy.featuresToUse
extraParams['compressData'] = mySAMpy.compressData
extraParams['featuresToCompress'] = mySAMpy.featuresToCompress
extraParams['maxNumItems'] = mySAMpy.maxNumItems
extraParams['deltaDistanceThreshold'] = mySAMpy.deltaDistanceThreshold
extraParams['jointMu'] = mySAMpy.jointMu
extraParams['jointSig'] = mySAMpy.jointSig
extraParams['numJoints'] = mySAMpy.numJoints
extraParams['ratioData'] = ratioData
extraParams['model_type'] = model_type
extraParams['model_num_inducing'] = model_num_inducing
extraParams['model_num_iterations'] = model_num_iterations
extraParams['model_init_iterations'] = model_init_iterations
extraParams['contactThreshold'] = mySAMpy.contactThreshold
extraParams['verbose'] = verbose
extraParams['Quser'] = mySAMpy.Quser
extraParams['textLabels'] = mySAMpy.textLabels
extraParams['economy_save'] = economy_save
extraParams['humanStaticLabels'] = mySAMpy.humanStaticLabels
extraParams['featureSections'] = mySAMpy.featureSections
extraParams['featureValues'] = mySAMpy.featureValues
extraParams['dataLogList'] = mySAMpy.dataLogList
extraParams['labelsLogList'] = mySAMpy.labelsLogList
SAMCore.save_pruned_model(mySAMpy.SAMObject, fname_cur, economy_save, extraDict=extraParams)