#!/usr/bin/env ipython
import matplotlib
import matplotlib.pyplot as plt
import readline
import warnings
import GPy
from SAM.SAM_Core import SAMCore
from SAM.SAM_Core import SAMDriver
import SAM
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
from SAM.SAM_Drivers import testingSegments

print sys.argv
stringCommand = 'from SAM.SAM_Drivers import ' + sys.argv[3] + ' as Driver'
print stringCommand
exec stringCommand

mySAMpy = Driver(False)

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
driverName = sys.argv[3]
mode = sys.argv[4]
trainName = sys.argv[5]
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
print 'Driver:'.ljust(off), driverName
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

print modeConfig
defaultParamsList = ['experiment_number','model_type','model_num_inducing', \
                    'model_num_iterations', 'model_init_iterations', 'verbose', \
                    'Quser', 'kernelString', 'ratioData', 'update_mode']

if(mode == 'new' or modeConfig == 'new' or '.pickle' not in modelPath): #or update but no .pickle
    print 'Loading training parameters from: \n ', '\t' + dataPath + "/config.ini"
    try:
        fail = False
        default = False
        parser = SafeConfigParser()
        parser.optionxform=str
        found = parser.read(dataPath + "/config.ini")
        
        #load parameters from config file
        if(parser.has_option(trainName, 'experiment_number')):
            experiment_number = int(parser.get(trainName, 'experiment_number'))
        elif('.pickle' in modelPath):
            experiment_number = int(modelPath.split('__')[-2].replace('exp','')) + 1
        else:
            fail = True
            print 'No experiment_number found'

        if(parser.has_option(trainName, 'model_type')):
            model_type = parser.get(trainName, 'model_type')
        else:
            default = True
            model_type = 'mrd'

        if(parser.has_option(trainName, 'model_num_inducing')):
            model_num_inducing = int(parser.get(trainName, 'model_num_inducing'))
        else:
            default = True
            model_num_inducing = 30

        if(parser.has_option(trainName, 'model_num_iterations')):
            model_num_iterations = int(parser.get(trainName, 'model_num_iterations'))
        else:
            default = True
            model_num_iterations = 700

        if(parser.has_option(trainName, 'model_init_iterations')):
            model_init_iterations = int(parser.get(trainName, 'model_init_iterations'))
        else:
            default = True
            model_init_iterations = 2000

        if(parser.has_option(trainName, 'verbose')):
            mySAMpy.verbose = parser.get(trainName, 'verbose')
        else:
            default = True
            mySAMpy.verbose = False

        if(parser.has_option(trainName, 'Quser')):
            mySAMpy.Quser = int(parser.get(trainName, 'Quser'))
        else:
            default = True
            mySAMpy.Quser = 2

        if(parser.has_option(trainName, 'kernelString')):
            kernelString = parser.get(trainName, 'kernelString')
        else:
            default = True
            kernelString = "GPy.kern.RBF(Q, ARD=False) + GPy.kern.Bias(Q) + GPy.kern.White(Q)"

        if(parser.has_option(trainName, 'ratioData')):
            ratioData = int(parser.get(trainName, 'ratioData'))
        else:
            default = True
            ratioData = 50

        if(default):
            print 'Default settings applied'

        mySAMpy.paramsDict = dict()
        mySAMpy.loadParameters(parser, trainName)

    except IOError:
        print 'IO Exception reading ', found
    	pass
else:
    print 'Loading parameters from: \n ','\t' + modelPath
    try:
        parser = SafeConfigParser()
        parser.optionxform=str
        found = parser.read(dataPath + "/config.ini")

        #load parameters from config file
        if(parser.has_option(trainName, 'experiment_number')):
            experiment_number = int(parser.get(trainName, 'experiment_number'))
        else:
            experiment_number = int(modelPath.split('__')[-2].replace('exp',''))

        modelPickle = pickle.load(open(modelPath ,'rb'))
        mySAMpy.paramsDict = dict()
        for j in parser.options(trainName):
            if(j not in defaultParamsList):
                print j
                mySAMpy.paramsDict[j] = modelPickle[j]
        
        ratioData = modelPickle['ratioData']
        model_type = modelPickle['model_type']
        model_num_inducing = modelPickle['model_num_inducing']
        model_num_iterations = modelPickle['model_num_iterations']
        model_init_iterations = modelPickle['model_init_iterations']
        mySAMpy.verbose = modelPickle['verbose']
        mySAMpy.Quser = modelPickle['Quser']
        kernelString = modelPickle['kernelString']

    except IOError:
        print 'IO Exception reading ', found
        pass

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

yTrainingData = mySAMpy.formatDataFunc(Yall)
testingSegments.testSegments(mySAMpy, yTrainingData, Lall)

yTrainingData = mySAMpy.formatDataFunc(YtestAll)
testingSegments.testSegments(mySAMpy, yTrainingData, LtestAll)
print
#save model with custom .pickle dictionary by iterating through all nested models
fname_cur = fname
print '-------------------'
print 'Saving: ' + fname_cur


mySAMpy.paramsDict['ratioData'] = ratioData
mySAMpy.paramsDict['model_type'] = model_type
mySAMpy.paramsDict['model_num_inducing'] = model_num_inducing
mySAMpy.paramsDict['model_num_iterations'] = model_num_iterations
mySAMpy.paramsDict['model_init_iterations'] = model_init_iterations
mySAMpy.paramsDict['economy_save'] = economy_save

mySAMpy.saveParameters()
print mySAMpy.paramsDict

SAMCore.save_pruned_model(mySAMpy.SAMObject, fname_cur, economy_save, extraDict=mySAMpy.paramsDict)