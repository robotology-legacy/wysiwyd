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
import readline
import warnings
import GPy
from SAM.SAM_Core import SAMCore
import threading
from SAM.SAM_Core import SAMDriver
from SAM.SAM_Drivers import SAMDriver_AR
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
import numpy as np
import numpy.ma as ma
import datetime
import yarp
import copy
from itertools import combinations 
from ConfigParser import SafeConfigParser
from scipy.spatial import distance
from numpy.linalg import inv
import math
import itertools

warnings.simplefilter("ignore")

#yarpRunning = False
dataPath = sys.argv[1]
modelPath = sys.argv[2]
interactionConfPath = sys.argv[3]

splitPath = modelPath.split('__')
modelBase = '__'.join(splitPath[:-1])

msplit = modelPath.split('/')
modelFolder = '/'.join(msplit[:-1])
modelName = modelBase.split('/')[-1]

participantList = [f for f in listdir(dataPath) if isdir(join(dataPath, f))]
modelList = [join(modelFolder,f.replace('.pickle','')) for f in listdir(modelFolder) if isfile(join(modelFolder, f)) if modelName in f if '.pickle' in f if '~' not in f]

#parameters are common across all items of modelList
modelPickle = pickle.load(open(modelList[0]+'.pickle' ,'rb'))
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
textLabels = modelPickle['textLabels']
contactThreshold = modelPickle['contactThreshold']
Ytrain = modelPickle['YALL']
Ltrain= modelPickle['LALL']
economy_save = modelPickle['economy_save']
humanStaticLabels = modelPickle['humanStaticLabels']
featureSections = modelPickle['featureSections']
featureValues = modelPickle['featureValues']
#Ytest = modelPickle['YTEST']
#Ltest = modelPickle['LTEST']

yarpRunning = False

# # Creates a SAMpy object
print 'Loading model ...'
mySAMpy = SAMDriver_AR.SAMDriver_AR(yarpRunning)
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
mySAMpy.contactThreshold = contactThreshold
mySAMpy.jointMu = jointMu
mySAMpy.jointSig = jointSig
mySAMpy.verbose = verbose
mySAMpy.humanStaticLabels = humanStaticLabels
mySAMpy.featureSections = featureSections
mySAMpy.featureValues = featureValues
mySAMpy.textLabels = textLabels

if model_type == 'mrd':    
    mySAMpy.X=None     
    mySAMpy.Y = {'Y':Ytrain,'L':Ltrain}
    mySAMpy.data_labels = Ltrain.copy()
    mySAMpy.labelName = textLabels
elif model_type == 'bgplvm':
    mySAMpy.X=None     
    mySAMpy.Y = {'Y':Ytain}
    mySAMpy.data_labels = Ltrain.copy()
    mySAMpy.labelName = textLabels
 
fname = modelList[0]

if Quser > 100:
    #one could parse and execute the string kernelStr for kernel instead of line below
    kernel = GPy.kern.RBF(Q, ARD=False) + GPy.kern.Bias(Q) + GPy.kern.White(Q)
else:
    kernel = None

# Simulate the function of storing a collection of events
mySAMpy.SAMObject.store(observed=mySAMpy.Y, inputs=mySAMpy.X, Q=Quser, kernel=kernel, num_inducing=model_num_inducing)
SAMCore.load_pruned_model(fname, economy_save, mySAMpy.SAMObject.model)
    
#open ports
yarp.Network.init()

sect = splitPath[0].split('/')[-1].lower()

parser2 = SafeConfigParser()
parser2.read(interactionConfPath)
portNameList = parser2.items(sect)
print
portsList = []
for j in range(len(portNameList)):
    if(portNameList[j][0] == 'rpcbase'):
        portsList.append(yarp.Port())
        print 'clientPortName', portNameList[j][1]+':i'
        portsList[j].open(portNameList[j][1]+':i')
        svPort = j
    elif(portNameList[j][0] == 'callsign'):
        callSignList = portNameList[j][1].split(',')
    else:
        parts = portNameList[j][1].split(' ')

        if(parts[1].lower() == 'imagergb'):
            portsList.append(yarp.BufferedPortImageRgb())
            portsList[j].open(parts[0])

        elif(parts[1].lower() == 'imagemono'):
            portsList.append(yarp.BufferedPortImageMono())
            portsList[j].open(parts[0])

        elif(parts[1].lower() == 'bottle'):
            portsList.append(yarp.BufferedPortBottle())
            portsList[j].open(parts[0])
        #mrd models with label/instance training will always have:
        #1 an input data line which is used when a label is requested
        #2 an output data line which is used when a generated instance is required
        if(parts[0][-1] == 'i'):
            labelPort = j
        elif(parts[0][-1] == 'o'):
            instancePort = j

#making sure all ports are connected
out = 0
while(out == 0):
    out = portsList[svPort].getOutputCount() + portsList[svPort].getInputCount()
    print 'Waiting for ' + portNameList[svPort][1] + ' to receive a connection'
    time.sleep(1)
print 'Connection received'
print
print '--------------------'
inputBottle = yarp.Bottle();
outputBottle = yarp.Bottle();
dataReceived = yarp.Bottle();

#wrap = yarp.BufferedPortBottle(portsList[svPort].asPort())
#prepare yarp variables
# imageArray = numpy.zeros((imgH, imgW, 3), dtype=numpy.uint8)
# yarpImage = yarp.ImageRgb()
# yarpImage.resize(imgH,imgW)
# yarpImage.setExternal(imageArray, imageArray.shape[1], imageArray.shape[0])

# numFaces = 10
# testFace = numpy.zeros([numFaces,imgHNew*imgWNew])
# images = numpy.zeros((numFaces, imgHNew*imgWNew), dtype=numpy.uint8)
replyString = ''
print 'Responding to callsigns: ' + ', '.join(callSignList)

def readCommands(supPort, inBottle, replyBool, replyStr, exception, recActions ):
    while(1):
        supPort.read(inBottle,replyBool)
        message = inBottle.get(0).asString()
        print(message + ' received')
        print 'responding to ' + message + ' request'

        if(message == 'EXIT'):
            exception[0] =  'keyInterupt'
            replyStr = 'ack'
        elif('label' in message):
            if(len(recActions) > 0):
                print 'label'
                print recActions
                replyStr = '__'.join(recActions)
                replyStr = 'ack ' + replyStr 
            else:
                replyStr = 'ack no_actions_recognised'
        elif('instance' in message):
            # parse all remaining Bottle contents
            if(portsList[instancePort].getInputCount() != 0):
                replyStr = 'ack'
            else:
                replyStr = 'nack'
                print 'No connections to ' + portsList[instancePort].getName()
        else:
            replyStr = 'nack'
            print message + ' is not a valid request'
        print
        supPort.reply(yarp.Bottle(replyStr))

exception = []
actionStore = []
exception.append('')
read_thread = threading.Thread(target=readCommands, args=(portsList[svPort], inputBottle, True, replyString, exception, actionStore  ))
read_thread.start()
#portsList[svPort].setTimeout(1)

numJoints = 9
data = dict()
jointsList = []
objectsList = []
angleThreshold = mySAMpy.angleThreshold
deltaDistanceThreshold = mySAMpy.deltaDistanceThreshold
contactThreshold = mySAMpy.contactThreshold
verbose = False
data = dict()
mySAMpy.configProcessing()
actionStore = []

while( True ):
        try:
            if(portsList[labelPort].getInputCount() == 0):
                print "Waiting for data connection"
                time.sleep(1)
            elif(portsList[labelPort].getPendingReads() > 0):
                #process incoming stream of data and keep last zero to zero
                #step 1: parse bottle
                dataReceived = portsList[labelPort].read(True)
                dataMessage = dataReceived.toString()
                t = dataMessage.replace('(','').replace(')','').split(' ')
                if(t > 40):
                    del t[0:2]
                    #extract data parts
                    for i in range(numJoints):
                        a = i*4
                        if(t[a] == 'shoulderCenter'):
                            t[a] = 'chest'

                        data[t[a]] = (np.array([float(t[a+1]), float(t[a+2]), float(t[a+3])]))
                        if(t[a] not in jointsList):
                            jointsList.append(t[a])

                    currIdx = (numJoints*4 -1)
                    numObjs = (len(t) - currIdx)/5

                    for i in range(numObjs):
                        a = currIdx + 1 + (i*5)
                        data[t[a]] = np.array([float(t[a+1]), float(t[a+2]), float(t[a+3])])
                        if(t[a] not in objectsList):
                            objectsList.append(t[a])

                    #check contact of either hand with either object
                    #generate list of combinations of hands and objects to check for contact
                    combinationList = []
                    combinationKeys = []
                    for i in objectsList[1:]:
                        combinationList.append(['handLeft',i])
                        combinationList.append(['handRight',i])
                        
                        combinationKeys.append(','.join(combinationList[-2]))
                        combinationKeys.append(','.join(combinationList[-1]))
                    
                    Pk = None
                    Pl = None
                    for i in range(len(combinationList)):
                        if(combinationKeys[i] not in data):
                            print 'add item', combinationKeys[i]
                            data[combinationKeys[i]] = {'Pk':[None],'Pl':[None],'prevContact':False,'currContact':False,'d':[None]}
                       
                        if(Pk == None):
                            Pk = data[combinationList[i][0]].T
                            Pl = data[combinationList[i][1]].T
                        else:
                            Pk = np.vstack((Pk,data[combinationList[i][0]].T))
                            Pl = np.vstack((Pl,data[combinationList[i][1]].T))

                    d = mySAMpy.distEuc(Pk,Pl)
                    
                    for i in range(len(combinationList)):
                        if(d[i] < mySAMpy.contactThreshold):
                            data[combinationKeys[i]]['currContact'] = True
                        else:
                            data[combinationKeys[i]]['currContact'] = False
                        
                        if(data[combinationKeys[i]]['currContact']):
                            if(data[combinationKeys[i]]['prevContact']):
                                data[combinationKeys[i]]['Pk'].append(Pk[i])
                                data[combinationKeys[i]]['Pl'].append(Pl[i])
                                if(verbose):
                                    print i,'Append data', combinationList[i]
                            else:
                                data[combinationKeys[i]]['actionOccuring'] = True
                                if(verbose):
                                    print i,'Contact between', combinationList[i], 'Action started'
                        else:
                            if(data[combinationKeys[i]]['prevContact']):
                                data[combinationKeys[i]]['actionOccuring']  = False
                                data[combinationKeys[i]]['actionLen'] = len(data[combinationKeys[i]]['Pk'])
                                
                                if(data[combinationKeys[i]]['actionLen'] > 10):
                                    if(verbose):
                                        print i,'Action stopped.', 'Len =', data[combinationKeys[i]]['actionLen']
                                    #processing the action
                                    tempQTC = mySAMpy.extractFeatures(Pk, Pl)
                                    tempQTC = mySAMpy.chooseFeatures(tempQTC)
                                    print
                                    [label, prob] = mySAMpy.testing(tempQTC[None,:], False)
                                    sentence = "You " + label.split('_')[0] + "ed the " + str(combinationList[i][1]) + " with your " + str(combinationList[i][0]).replace('hand','') + ' hand'
                                    print sentence
                                    actionStore.append(sentence)
                                else:
                                    if(verbose):
                                        print i,'Action stopped.', 'Len =', data[combinationKeys[i]]['actionLen'], 'Action too short'
                                    else:
                                        'Action too short'
                                    
                                data[combinationKeys[i]]['Pk'] = [None]
                                data[combinationKeys[i]]['Pl'] = [None]
                            else:
                                if(verbose):
                                    print i,'x'

                        data[combinationKeys[i]]['prevContact'] =  data[combinationKeys[i]]['currContact']
                    if(verbose):
                        print
                else:
                    print 'Incorrect message received'

            if(exception[0] == 'keyInterupt'):
                raise KeyboardInterrupt

        except KeyboardInterrupt:
            print 'Exiting ...'
            for j in portsList:
                j.close()
            try:
                sys.exit(0)
            except SystemExit:
                os._exit(0)
for j in portsList:
    j.close()