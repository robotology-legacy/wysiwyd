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
from SAM.SAM_Drivers.SAMDriver_speech import SAMDriver_speech
from SAM.SAM_Core import SAMCore
import GPy
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
from sklearn.metrics import confusion_matrix

def plot_confusion_matrix(cm, targetNames, title='Confusion matrix', cmap=plt.cm.gray):
    plt.imshow(cm, interpolation='nearest', cmap=cmap)
    plt.title(title)
    plt.colorbar()
    tick_marks = np.arange(len(targetNames))
    plt.xticks(tick_marks, targetNames, rotation=45)
    plt.yticks(tick_marks, targetNames)
    plt.tight_layout()
    plt.ylabel('True label')
    plt.xlabel('Predicted label')
    plt.show()

def dist(result, model):
    from scipy.spatial import distance
    dists = np.zeros(model.model.X.shape[0])
    for idx,x in enumerate(model.model.X.mean):
        dists[idx] = distance.euclidean(x,result)
    return np.vstack(dists)

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
		if(parser.has_option(trainName, 'delta')):
			delta = bool(parser.get(trainName, 'delta') )
		else:
			delta = False

		if(parser.has_option(trainName, 'context')):
			context = int(parser.get(trainName, 'context'))
		else:
			context = 2

		if(parser.has_option(trainName, 'n_mixtures')):
			n_mixtures = int(parser.get(trainName, 'n_mixtures') )
		else:
			n_mixtures = 25

		if(parser.has_option(trainName, 'experiment_number')):
			experiment_number = int(parser.get(trainName, 'experiment_number'))
		elif('.pickle' in modelPath):
			experiment_number = int(modelPath.split('__')[-2].replace('exp','')) + 1
		else:
			experiment_number = 0

		if(parser.has_option(trainName, 'percentTestData')):
			ratioData = int(parser.get(trainName, 'percentTestData'))
		else:
			ratioData = 10

		if(parser.has_option(trainName, 'file_suffix')):
			file_suffix = parser.get(trainName, 'file_suffix')
		else:
			file_suffix = ".wav"

		if(parser.has_option(trainName, 'model_type')):
			model_type = parser.get(trainName, 'model_type')
		else:
			model_type = 'bgplvm'

		if(parser.has_option(trainName, 'model_num_inducing')):
			model_num_inducing = int(parser.get(trainName, 'model_num_inducing'))
		else:
			model_num_inducing = 40

		if(parser.has_option(trainName, 'model_num_iterations')):
			model_num_iterations = int(parser.get(trainName, 'model_num_iterations'))
		else:
			model_num_iterations = 2000

		if(parser.has_option(trainName, 'model_init_iterations')):
			model_init_iterations = int(parser.get(trainName, 'model_init_iterations'))
		else:
			model_init_iterations = 500

		if(parser.has_option(trainName, 'kernelString')):
			kernelString = parser.get(trainName, 'kernelString')
		else:
			kernelString = "GPy.kern.RBF(Q, ARD=False) + GPy.kern.Bias(Q) + GPy.kern.White(Q)"

		gmm_atts = None

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

	delta = modelPickle['delta']
	context = modelPickle['context']
	n_mixtures = modelPickle['n_mixtures']
	ratioData = modelPickle['percentTestData']
	file_suffix = modelPickle['file_suffix']
	model_type = modelPickle['model_type']
	gmm_atts = modelPickle['gmm_atts']
	model_num_inducing = modelPickle['num_inducing']
	model_init_iterations = modelPickle['model_init_iterations']
	model_num_iterations = modelPickle['model_num_iterations']
	kernelString = modelPickle['kernelString']

# Creates a SAMpy object

mySAMpy = SAMDriver_speech(yarpRunning, delta=delta, gmm_atts=gmm_atts, context=context, n_mixtures=n_mixtures)

# Location of speech data
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
save_model = True
economy_save = False
visualise_output = False
test_mode = True

# Reading speech data, preparation of data and training of the model
mySAMpy.readData(root_data_dir, participant_index, pose_index)

minImages = mySAMpy.Y.shape[1]
Ntest = int(minImages*ratioData/100)
Ntr = minImages - Ntest

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


cur = SAMDriver_speech(False, delta=delta, context=context, n_mixtures=n_mixtures)
cur.Quser=4

startIDx = 0;
endIDx = Ntr

Y = Yall
L = Lall

startIDx = Ntest*i;
endIDx = (Ntest*(i+1))

Ytest = YtestAll
Ltest = LtestAll

# Center data to zero mean and 1 std
Ymean = Yall.mean()
Yn = Yall - Ymean
Ystd = Yn.std()
Yn /= Ystd
# Normalise test data similarly to training data
Ytestn = Ytest - Ymean
Ytestn /= Ystd

cur.Ymean = Ymean
cur.Ystd = Ystd
# As above but for the labels
#Lmean_cur = L_cur.mean()
#Ln_cur = L_cur - Lmean_cur
#Lstd_cur = Ln_cur.std()
#Ln_cur /= Lstd_cur
#Ltestn_cur = Ltest_cur - Lmean_cur
#Ltestn_cur /= Lstd_cur

cur.X = None
cur.Y = {'Y':Yn}
cur.Ytestn = {'Ytest':Ytestn}
cur.Ltest = {'Ltest':Ltest}
print 'training data' + str(cur.Y['Y'].shape)
print 'testing data' + str(cur.Ytestn['Ytest'].shape)

fname_cur = fname + '__L'# + str(i)
cur.training(model_num_inducing, model_num_iterations, model_init_iterations, fname_cur, save_model, economy_save, keepIfPresent=False, kernelStr=kernelString)
mm.append(cur)
ss = [];
sstest = [];
print

#Ntest = 10
print Ntest
resultTemp = mm[0].SAMObject.pattern_completion(Ytestn)
predictions = np.zeros([len(participantList),Ntest])
confusionMatrix = np.zeros([len(participantList),len(participantList)])


print resultTemp[0].shape

for i in range(len(participantList)):
    curTest = resultTemp[0][(Ntest*i):(Ntest*i+Ntest)]
    
    for j,result in enumerate(curTest):
        predictions[i,j] = L[np.argmin(dist(result,mm[0].SAMObject))]
    

    
    for j in range(len(participantList)):
        confusionMatrix[i,j] = np.count_nonzero(j == predictions[i])# / float(Ntest)
    
    print 'Accuracy for participant',participantList[i], np.count_nonzero(i == predictions[i]) / float(Ntest)

print
print confusionMatrix
    
#     print 'Total correct:', np.count_nonzero(predictions[i,:]==participantList[i])
  
plot_confusion_matrix(confusion_matrix(np.vstack(participantList[i] for i in Ltest),predictions.flatten()), participantList)
plt.figure()
mm[0].SAMObject.model.plot_latent(labels=L,which_indices=SAMCore.most_significant_input_dimensions(mm[0].SAMObject.model,None))
raw_input('Press enter when done')

# print 'Total correct:', np.count_nonzero(predictions[i,:]==participantList[i])
   
# print 'Average accuracy of:', np.count_nonzero(int(participantList) == predictions) / float(len(Ytestn))

if visualise_output:  
	plot_confusion_matrix(confusion_matrix(np.vstack(participantList[i] for i in Ltest),predictions.flatten()), participantList)
	plt.figure()
	mm[0].SAMObject.model.plot_latent(labels=L,which_indices=SAMCore.most_significant_input_dimensions(mm[0].SAMObject.model,None))
	ax = mm[0].SAMObject.model.plot_latent(labels=L)
	y = mm[0].SAMObject.model.Y[0,:]
	data_show = GPy.plotting.matplot_dep.visualize.vector_show(y)
	lvm_dimselect = GPy.plotting.matplot_dep.visualize.lvm(mm[0].SAMObject.model.X.mean[0,:].copy(), mm[0].SAMObject.model, data_show, ax)
	raw_input('Press enter to finish')

#save model with custom .pickle dictionary by iterating through all nested models



fname_cur = fname + '__L' + str(0)
print 'Saving: ' + fname_cur
extraParams = dict()
extraParams['delta'] = delta
extraParams['context'] = context
extraParams['n_mixtures'] = n_mixtures
extraParams['percentTestData'] = ratioData
extraParams['file_suffix'] = file_suffix
extraParams['model_num_iterations'] = model_num_iterations
extraParams['model_init_iterations'] = model_init_iterations
extraParams['model_type'] = model_type
extraParams['gmm_atts'] = mySAMpy.gmm_data
extraParams['confusion'] = confusionMatrix
SAMCore.save_pruned_model(mm[0].SAMObject, fname_cur, economy_save, extraDict=extraParams)
