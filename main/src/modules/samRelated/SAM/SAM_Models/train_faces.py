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

yarpRunning = False
dataPath = sys.argv[1]
modelPath = sys.argv[2]
trainName = sys.argv[3]
mode = sys.argv[4]

#participantList is extracted from number of subdirectories of dataPath
participantList = [f for f in listdir(dataPath) if isdir(join(dataPath, f))]

print '-------------------'
print dataPath
print modelPath
print participantList
print trainName
print mode
print '-------------------'

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
	print 'loading training parameters from config.ini' 
	try:
		parser = SafeConfigParser()
		found = parser.read(dataPath + "/config.ini")

		#load parameters from config file
		if(parser.has_option(trainName, 'imgH')):
			imgH = int(parser.get(trainName, 'imgH') )
		else:
			imgH = 400

		if(parser.has_option(trainName, 'imgW')):
			imgW = int(parser.get(trainName, 'imgW'))
		else:
			imgW = 400

		if(parser.has_option(trainName, 'imgHNew')):
			imgHNew = int(parser.get(trainName, 'imgHNew') )
		else:
			imgHNew = 200

		if(parser.has_option(trainName, 'imgWNew')):
			imgWNew = int(parser.get(trainName, 'imgWNew'))
		else:
			imgWNew = 200

		if(parser.has_option(trainName, 'experiment_number')):
			experiment_number = int(parser.get(trainName, 'experiment_number'))
		elif('.pickle' in modelPath):
			experiment_number = int(modelPath.split('__')[-2].replace('exp','')) + 1
		else:
			experiment_number = 0

		if(parser.has_option(trainName, 'percentTestData')):
			ratioData = int(parser.get(trainName, 'percentTestData'))
		else:
			ratioData = 30

		if(parser.has_option(trainName, 'image_suffix')):
			image_suffix = parser.get(trainName, 'image_suffix')
		else:
			image_suffix = ".ppm"

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

		if(parser.has_option(trainName, 'kernelString')):
			kernelString = parser.get(trainName, 'kernelString')
		else:
			kernelString = "GPy.kern.RBF(Q, ARD=False) + GPy.kern.Bias(Q) + GPy.kern.White(Q)"

	except IOError:
		pass
else:
	print 'loading parameters from ' + modelPath
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

# Creates a SAMpy object
mySAMpy = SAMDriver_interaction(yarpRunning, imgH = imgH, imgW = imgW, imgHNew = imgHNew, imgWNew = imgWNew)

# Location of face data
root_data_dir=dataPath

# Array of participants to be recognised
participant_index=participantList

# Poses used during the data collection
pose_index=['']

# Pose selected for training
pose_selection = 0

print 'modelPath: ' + modelPath

if('.pickle' in modelPath):
	fname = '/'.join(modelPath.split('/')[:-1]) + '/' + dataPath.split('/')[-1] + '__' + trainName + '__' +  model_type + '__exp' + str(experiment_number)
else:
	fname = modelPath + dataPath.split('/')[-1] + '__' + trainName + '__' +  model_type + '__exp' + str(experiment_number) #+ '.pickle'

print fname

# Enable to save the model and visualise GP nearest neighbour matching
save_model = False
economy_save = True
visualise_output = False
test_mode = True

# Reading face data, preparation of data and training of the model
mySAMpy.readData(root_data_dir, participant_index, pose_index)

minImages = mySAMpy.Y.shape[1]
Ntr = int(minImages*ratioData/100)
Ntest = minImages - Ntr

allPersonsY = mySAMpy.Y;
allPersonsL = mySAMpy.L;

for i in range(len(participantList)):
	#print participantList[i]
	mySAMpy.Y = allPersonsY[:,:,i,None]
	mySAMpy.L = allPersonsL[:,:,i,None]
	(Yalli, Lalli, YtestAlli, LtestAlli) = mySAMpy.prepareData(model_type, Ntr, pose_selection, randSeed=experiment_number)

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

for i in range(len(participantList)):
	print('# Considering label: ' + str(participantList[i]))
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

	cur.Ymean = Ymean_cur
	cur.Ystd = Ystd_cur
	# As above but for the labels
	#Lmean_cur = L_cur.mean()
	#Ln_cur = L_cur - Lmean_cur
	#Lstd_cur = Ln_cur.std()
	#Ln_cur /= Lstd_cur
	#Ltestn_cur = Ltest_cur - Lmean_cur
	#Ltestn_cur /= Lstd_cur

	cur.X = None
	cur.Y = {'Y':Yn_cur}
	cur.Ytestn = {'Ytest':Ytestn_cur}
	cur.Ltest = {'Ltest':Ltest_cur}
	print 'training data' + str(cur.Y['Y'].shape)
	print 'testing data' + str(cur.Ytestn['Ytest'].shape)

	fname_cur = fname + '__L' + str(i)
	cur.training(model_num_inducing, model_num_iterations, model_init_iterations, fname_cur, save_model, economy_save, keepIfPresent=False, kernelStr=kernelString)
	mm.append(cur)
	ss = [];
	sstest = [];
	print

#Ntest = 10
print Ntest
result = np.zeros([len(participantList),Ntest,len(participantList)])
responseIdx = np.zeros([result.shape[0],result.shape[1]])
responseVal = np.zeros([result.shape[0],result.shape[1]])
confusionMatrix = np.zeros([result.shape[0],result.shape[0]])

for i in range(result.shape[0]):
	print
	print('Participant ' + str(i) + ': ' + participantList[i])
	print
	currTestData = mm[i].Ytestn['Ytest']
	for j in range(result.shape[1]):
		currImage = currTestData[None,j,:]
		for k in range(result.shape[2]):
			try:
				resultTemp = mm[k].SAMObject.familiarity(currImage)
				result[i,j,k] = resultTemp
			except ValueError:
				print 'Value Error while evaluating familiarity'
				pass
		maxIdx = np.argmax(result[i,j,:])
		responseIdx[i,j] = maxIdx
		responseVal[i,j] = result[i,j,maxIdx]

	for g in range(result.shape[2]):
		confusionMatrix[i][g] = 100*float(np.count_nonzero(responseIdx[i][:]==g))/float(responseIdx.shape[1])
		print('Percentage ' + participantList[i] + ' classified as ' + participantList[g] + ' = ' +  str(confusionMatrix[i][g]))
print

#save model with custom .pickle dictionary by iterating through all nested models
for i in range(len(participantList)):
	fname_cur = fname + '__L' + str(i)
	print 'Saving: ' + fname_cur
	extraParams = dict()
	extraParams['imgH'] = imgH
	extraParams['imgW'] = imgW
	extraParams['imgHNew'] = imgHNew
	extraParams['imgWNew'] = imgWNew
	extraParams['percentTestData'] = ratioData
	extraParams['image_suffix'] = image_suffix
	extraParams['model_num_iterations'] = model_num_iterations
	extraParams['model_init_iterations'] = model_init_iterations
	extraParams['model_type'] = model_type
	extraParams['confusion'] = confusionMatrix[i]
	SAMCore.save_pruned_model(mm[i].SAMObject, fname_cur, economy_save, extraDict=extraParams)
