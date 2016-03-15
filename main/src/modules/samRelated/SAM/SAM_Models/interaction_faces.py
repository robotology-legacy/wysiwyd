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
from SAM.SAM_Core import SAMCore
import pylab as pb
import sys
import pickle
import os
from os import listdir
from os.path import isfile, join, isdir
import numpy
import time
import operator
import numpy as np
from ConfigParser import SafeConfigParser
import yarp
import cv2

#yarpRunning = False
dataPath = sys.argv[1]
modelPath = sys.argv[2]
interactionConfPath = sys.argv[3]

splitPath = modelPath.split('__')
modelBase = '__'.join(splitPath[:-1])

msplit = modelPath.split('/')
modelFolder = '/'.join(msplit[:-1])
modelName = modelBase.split('/')[-1]

print modelFolder
print modelName
participantList = [f for f in listdir(dataPath) if isdir(join(dataPath, f))]
modelList = [join(modelFolder,f.replace('.pickle','')) for f in listdir(modelFolder) if isfile(join(modelFolder, f)) if modelName in f if '.pickle' in f if '~' not in f]
print '\n'.join(modelList)

#parameters are common across all items of modelList
modelPickle = pickle.load(open(modelList[0]+'.pickle' ,'rb'))
imgH = modelPickle['imgH']
imgW = modelPickle['imgW']
imgHNew = modelPickle['imgHNew']
imgWNew = modelPickle['imgWNew']
ratioData = modelPickle['percentTestData']
image_suffix = modelPickle['image_suffix']
model_type = modelPickle['model_type']
model_num_inducing = modelPickle['num_inducing']
model_init_iterations = modelPickle['model_init_iterations']
model_num_iterations = modelPickle['model_num_iterations']
kernelString = modelPickle['kernelString']
Q = modelPickle['Q']
economy_save = True
pose_index=['']
pose_selection = 0

# # Creates a SAMpy object
mySAMpy = SAMDriver_interaction(False, imgH = imgH, imgW = imgW, imgHNew = imgHNew, imgWNew = imgWNew)

# # Reading face data, preparation of data and training of the model
mySAMpy.readData(dataPath, participantList, pose_index)

minImages = mySAMpy.Y.shape[1]
Ntr = int(minImages*ratioData/100)
Ntest = minImages - Ntr

allPersonsY = mySAMpy.Y;
allPersonsL = mySAMpy.L;

for i in range(len(participantList)):
	#print participantList[i]
	mySAMpy.Y = allPersonsY[:,:,i,None]
	mySAMpy.L = allPersonsL[:,:,i,None]
	(Yalli, Lalli, YtestAlli, LtestAlli) = mySAMpy.prepareData(model_type, Ntr, pose_selection, randSeed=2)

	if(i==0):
		Yall = Yalli.copy();
		Lall = Lalli.copy();
		YtestAll = YtestAlli.copy()
		LtestAll = LtestAlli.copy()
	else:
		Yall = np.vstack([Yall,Yalli])
		Lall = np.vstack([Lall,Lalli])
		YtestAll = np.vstack([YtestAll,YtestAlli])
		LtestAll = np.vstack([LtestAll, LtestAlli])

allPersonsY = None
alPersonsL = None

mm = []
print 'modelList = ' + str(modelList)
for i in range(len(modelList)):
	print('# Loading model: ' + str(participantList[i]))
	cur = SAMDriver_interaction(False, imgH = imgH, imgW = imgW, imgHNew = imgHNew, imgWNew = imgWNew)

	startIDx = Ntr*i;
	endIDx = (Ntr*(i+1))
	
	Y_cur = Yall[startIDx:endIDx,:].copy()
	L_cur = Lall[startIDx:endIDx,:].copy()
	
	startIDx = Ntest*i;
	endIDx = (Ntest*(i+1))
	
	Ytest_cur = YtestAll[startIDx:endIDx,:].copy()
	Ltest_cur = LtestAll[startIDx:endIDx,:].copy()
	
	# Center data to zero mean and 1 std
	Ymean_cur = Y_cur.mean()
	Yn_cur = Y_cur - Ymean_cur
	Ystd_cur = Yn_cur.std()
	Yn_cur /= Ystd_cur
	# Normalise test data similarly to training data
	Ytestn_cur = Ytest_cur - Ymean_cur
	Ytestn_cur /= Ystd_cur
	
	#cur.Ymean = Ymean_cur
	#cur.Ystd = Ystd_cur
	# As above but for the labels
	#Lmean_cur = L_cur.mean()
	#Ln_cur = L_cur - Lmean_cur
	#Lstd_cur = Ln_cur.std()
	#Ln_cur /= Lstd_cur
	#Ltestn_cur = Ltest_cur - Lmean_cur
	#Ltestn_cur /= Lstd_cur
	
	cur.X = None
	cur.Y = None
	cur.Y = {'Y':Yn_cur}
	cur.Ytestn = {'Ytest':Ytestn_cur}
	cur.Ltest = {'Ltest':Ltest_cur}
	print 
	fname = modelList[i]

	if Q > 100:
		#one could parse and execute the string kernelStr for kernel instead of line below
		kernel = GPy.kern.RBF(Q, ARD=False) + GPy.kern.Bias(Q) + GPy.kern.White(Q)
	else:
		kernel = None

    # Simulate the function of storing a collection of events
	cur.SAMObject.store(observed=cur.Y, inputs=cur.X, Q=Q, kernel=kernel, num_inducing=model_num_inducing)
	SAMCore.load_pruned_model(fname, economy_save, cur.SAMObject.model)
	mm.append(cur)
	
#open ports
yarp.Network.init()

sect = splitPath[0].split('/')[-1].lower()

parser2 = SafeConfigParser()
parser2.read(interactionConfPath)
portNameList = parser2.items(sect)
print portNameList
portsList = []
for j in range(len(portNameList)):
	if(portNameList[j][0] == 'rpcbase'):
		portsList.append(yarp.RpcServer())
		portsList[j].open(portNameList[j][1]+':i')
		svPort = j
	elif(portNameList[j][0] == 'callsign'):
		callSignList = portNameList[j][1].split(',')
	else:
		parts = portNameList[j][1].split(' ')
		print parts

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

#prepare yarp image variables
imageArray = numpy.zeros((imgH, imgW, 3), dtype=numpy.uint8)
yarpImage = yarp.ImageRgb()
yarpImage.resize(imgH,imgW)
yarpImage.setExternal(imageArray, imageArray.shape[1], imageArray.shape[0])
imageReceived = yarp.ImageRgb()

numFaces = 10
testFace = numpy.zeros([numFaces,imgHNew*imgWNew])
images = numpy.zeros((numFaces, imgHNew*imgWNew), dtype=numpy.uint8)

print 'Responding to callsigns: ' + ', '.join(callSignList)
while( True ):
        try: 
			print "Waiting for input"
			print
			time.sleep(1)

			portsList[svPort].read(inputBottle,True)
			message = inputBottle.get(0).asString()
			print(message + ' received')
			print 'responding to ' + message + ' request'

			if(message == 'EXIT'):
				raise KeyboardInterrupt
				portsList[svPort].reply(yarp.Bottle('ack'))
			elif('label' in message):
				if(portsList[labelPort].getInputCount() != 0):
					#--------------------------------------------------------------------------------------------
					#this whole section can be replaced with a function call from a class containing all functions
					for i in range(numFaces):
						imageReceived = portsList[labelPort].read(True)
						if(imageReceived != None):
							yarpImage.copy(imageReceived)
							imageArrayOld=cv2.resize(imageArray,(imgHNew,imgWNew))
							imageArrayGray=cv2.cvtColor(imageArrayOld, cv2.COLOR_BGR2GRAY)
							images[i,:] = imageArrayGray.flatten()
							print "Collected face: " + str(i)
					#current shape 4000xnumFaces. Testing requires numFacesx4000
					#images = images - np.mean(images, axis = 0) #removes lighting effect averaged across 10 pictures
					images = images - images.mean()
					images /= images.std()

					ss=numpy.zeros(len(participantList))
					for i in range(len(participantList)):
						ss[i] = mm[i].SAMObject.familiarity(images)
						print('Familiarity with ' + participantList[i] + ' given current face is: ' + str(ss[i]))
					maxIdx = np.argmax(ss)
					maxVal = ss[maxIdx]

					threshold = 0.4
					if(maxVal > threshold):
						replyString = participantList[maxIdx]
					else:
						replyString = 'partner'
					#--------------------------------------------------------------------------------------------
				else:
					replyString = 'nack'
					print 'No connections to ' + portsList[labelPort].getName()
			elif('instance' in message):
				#parse all remaining Bottle contents
				if(portsList[instancePort].getInputCount() != 0):
					replyString = 'ack'
				else:
					replyString = 'nack'
					print 'No connections to ' + portsList[instancePort].getName()

			else:
				replyString = 'nack'
				print message + ' is not a valid request'
			
			portsList[svPort].reply(yarp.Bottle(replyString))

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