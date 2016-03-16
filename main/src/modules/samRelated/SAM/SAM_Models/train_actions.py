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
numQ = 6
warnings.simplefilter("ignore")

#--------------------functions------------------------------
def distEuc(a,b):
    temp = a-b
    temp = np.square(temp)
    temp = np.sum(temp,1)
    return np.sqrt(temp)

def qtc_2D(k,l,q,thresh, contactThresh, contact = None):
    d1 = distEuc(k[:-2],l[1:-1])
    d2 = distEuc(k[1:-1],l[1:-1])
    d3 = distEuc(k[2:],l[1:-1])

    for i in range(len(d1)):
        #threshold distance moved
        diff1 = d2[i]-d1[i]
        if(np.abs(diff1) < thresh):
            diff1 = 0

        diff2 = d3[i]-d2[i]
        if(np.abs(diff2) < thresh):
            diff2 = 0

        #convert to qtc
        if(diff1 > 0 and diff2 > 0):
            q[i] = -1
        elif(diff1 < 0 and diff2 < 0):
            q[i] = +1
        else:
            q[i] = 0
                
        #check contact
        if(contact != None):
	        if(d2[i]<contactThresh):
	        	contact[i] = 1

        #check qtc smoothness


def frenetFrame(arr):
    t_num = np.diff(arr,axis=0)
    t = (t_num/np.abs(t_num)).astype(int)

    b_num = np.cross(t[:-1],t[1:])
    b = b_num/np.abs(b_num)
    t = t[1:]

    n = np.cross(b,t)

    frameArr = np.concatenate((t,n,b),axis=1).T
    fArr = frameArr.reshape((3,3,-1),order = 'F')
    return fArr

def qtc_3D(k, l, thresh, q3, q4, q5):
    fFrameK = frenetFrame(k)
    fFrameL = frenetFrame(l)

    for g in range(fFrameK.shape[2]):
        fKinv = np.linalg.pinv(fFrameK[:,:,g])
        R = np.dot(fFrameL[:,:,g],fKinv)
        
        alpha = np.arctan(R[1,0]/R[0,0])
        den = np.sqrt(pow(R[2,1],2) + pow(R[2,2],2))
        
        beta = np.arctan(-R[2,0]/den)
        gamma = np.arctan(R[2,1]/R[2,2])

        #threshold angles
        if(np.abs(alpha) < thresh or math.isnan(alpha)):
            alpha = 0
        if(np.abs(beta) < thresh or math.isnan(beta)):
            beta = 0
        if(np.abs(gamma) < thresh or math.isnan(gamma)):
            gamma = 0

        q3[g] = np.sign(alpha)
        q4[g] = np.sign(beta)
        q5[g] = np.sign(gamma)

def most_common(L):
	# get an iterable of (item, iterable) pairs
	SL = sorted((x, i) for i, x in enumerate(L))
	# print 'SL:', SL
	groups = itertools.groupby(SL, key=operator.itemgetter(0))
	# auxiliary function to get "quality" for an item
	def _auxfun(g):
		item, iterable = g
		count = 0
		min_index = len(L)
		for _, where in iterable:
			count += 1
			min_index = min(min_index, where)
		# print 'item %r, count %r, minind %r' % (item, count, min_index)
		return count, -min_index
	# pick the highest-count/earliest item
	return max(groups, key=_auxfun)[0]

class AR_Driver(SAMDriver):
    def testing(self, testInstance):
		# Returns the predictive mean, the predictive variance and the axis (pp) of the latent space backwards mapping.            
        #mm,vv,pp=self.SAMObject.pattern_completion(testFace, visualiseInfo=visualiseInfo)
        ret=self.SAMObject.pattern_completion(testInstance, visualiseInfo=None)

        mm = ret[0]
        vv = ret[1]
        post = ret[3]        

        # find nearest neighbour of mm and SAMObject.model.X
        dists = np.zeros((self.SAMObject.model.X.shape[0],1))

        for j in range(dists.shape[0]):
            dists[j,:] = distance.euclidean(self.SAMObject.model.X.mean[j,:], mm[0].values)
        nn, min_value = min(enumerate(dists), key=operator.itemgetter(1))
        if self.SAMObject.type == 'mrd':
            #print "With " + str(vv.mean()) +" prob. error the new action is " + self.labelName[int(self.SAMObject.model.bgplvms[1].Y[nn,:])]
            textStringOut=self.labelName[int(self.SAMObject.model.bgplvms[1].Y[nn,:])]

        elif self.SAMObject.type == 'bgplvm':
            #print "With " + str(vv.mean()) +" prob. error the new action is " + self.labelName[int(self.L[nn,:])]
            textStringOut=self.labelName[int(self.L[nn,:])]

        # if(vv.mean()<0.00012):            
        #     print "The action is " + textStringOut
        # elif(vv.mean()>0.00012):
        #     print "I think the action is " + textStringOut + " but I am not sure"      

        # # Plot the training NN of the test image (the NN is found in the INTERNAl, compressed (latent) memory space!!!)
        # if visualiseInfo is not None:
        #     fig_nn = visualiseInfo['fig_nn']
        #     fig_nn = pb.figure(11)
        #     pb.title('Training NN')
        #     fig_nn.clf()
        #     pl_nn = fig_nn.add_subplot(111)
        #     pl_nn.imshow(numpy.reshape(self.SAMObject.recall(nn),(self.imgHeightNew, self.imgWidthNew)), cmap=plt.cm.Greys_r)
        #     pb.title('Training NN')
        #     pb.show()
        #     pb.draw()
        #     pb.waitforbuttonpress(0.1)
        #return pp

        return [textStringOut, vv.mean()]
    
    def readData(self, root_data_dir, participant_index):
		onlyfiles = [f for f in listdir(dataPath) if isfile(join(dataPath, f))]
		dataLogList = [f for f in onlyfiles if 'data' in f]
		dataLogList.sort()
		labelsLogList = [f for f in onlyfiles if 'label' in f]
		labelsLogList.sort()

		numJoints = 9
		data = dict()
		firstPass = True
		jointsList = []
		objectsList = []
		labelsList = []
		numFiles = len(dataLogList)

		print 'loading data from files'
		for k in range(len(dataLogList)):
			print 'data file: ' + str(join(dataPath, dataLogList[k]))
			print 'model file: ' + str(join(dataPath, labelsLogList[k]))
			print
			dataFile = open(join(dataPath, dataLogList[k]),'r')
			labelFile = open(join(dataPath, labelsLogList[k]),'r')

			#number of lines in dataFile
			for i, l in enumerate(dataFile):
					pass
			lenDataFile = i+1

			#number of lines in labelFile
			for i, l in enumerate(labelFile):
					pass
			lenLabelFile = i+1
			dataFile.close()
			labelFile.close()

			if(lenLabelFile != lenDataFile):
				print str(dataLogList[k]) + ' will not be used because its lenght differs from ' + str(labelsLogList[k])
			else:
				dataFile = open(join(dataPath, dataLogList[k]),'r')
				labelFile = open(join(dataPath, labelsLogList[k]),'r')
				labelsList.append([])

				for curr in range(lenDataFile):
					line = dataFile.readline()
					labelLine = labelFile.readline()

					t = line.replace('(','').replace(')','').split(' ')
					del t[0:4]
					#parse skeleton data which has 9 sections by (x,y,z)
					for i in range(numJoints):
						a = i*4
						if(firstPass):
							data[t[a]] = [None]*numFiles
							data[t[a]][k] = (np.array([float(t[a+1]), float(t[a+2]), float(t[a+3])]))
							jointsList.append(t[a])
						else:
							arr =  np.array([float(t[a+1]), float(t[a+2]), float(t[a+3])])
							if(data[t[a]][k] != None):
								data[t[a]][k] = np.vstack((data[t[a]][k],arr))
							else:
								data[t[a]][k] = arr

					currIdx = (numJoints*4 -1)
					numObjs = (len(t) - currIdx)/5

					for i in range(numObjs):
						a = currIdx + 1 + (i*5)
						if(t[a] in data):
							arr = np.array([float(t[a+1]), float(t[a+2]), float(t[a+3])])
							if(data[t[a]][k] != None):
								data[t[a]][k] =  np.vstack((data[t[a]][k],arr))
							else:
								data[t[a]][k] = arr
						else:
							data[t[a]] = [None]*(numFiles+1)
							data[t[a]][k] = np.array([float(t[a+1]), float(t[a+2]), float(t[a+3])])
							data[t[a]][-1] = int(t[a+4])
							objectsList.append(t[a])

					firstPass = False
					try:
						v = labelLine.split(' ')[2].replace('\n','').replace('(','').replace(')','')
					except IndexError:
						print labelLine

					labelsList[k].append(v)

		#compile a list of all unique labels
		print
		print 'Unique labels in labels files:'
		print
		setList = []
		for x in labelsList:
		    setList.append(list(set(x)))
		flattenedList = [val for sublist in setList for val in sublist]
		labels = list(set(flattenedList))
		labels.sort()
		for k in range(0,len(labels)):
			print str(k) + '  ' + labels[k]
		print
		print 'Of these, config.ini specifies [' + ', '.join(ignoreLabels) + '] to be ignored'

		ignoreInds = []
		for k in ignoreLabels:
			ignoreInds.append(labels.index(k))

		#prepare list of indices for labels which will be used
		#important that no other labels are removed because that would interfere with temporal continuity of data
		#no temporal continuity would make QTC calculation with big jumps which is not desirable
		#future work needs to go through data and split it into subsections depend

		doLabels = [x for i,x in enumerate(labels) if i not in ignoreInds]
		indicesList = []
		currIdxList = []
		func = 0
		if(func == 0):
			for ll in labelsList:
				indicesList.append([i for i, x in enumerate(ll) if x in doLabels])
		elif(func == 1):
			for ll in labelsList:
				#iterate over items of ll
				#indicesList stores contiguous regions between ignoredLabels 
				for l in ll:
					if(l in doLabels):
						currIdxList.append(l)
					elif(l not in doLabels):
						if(len(currIdxList) > 0):
							indicesList.append(currIdxList)
							currIdxList = []

		#apply indices
		subsetData = None
		subsetLabels = None
		#CHECK
		subsetData = copy.deepcopy(data)
		subsetLabels = copy.deepcopy(labelsList)
		if(func == 0):
			for k in range(numFiles):
				for j in jointsList + objectsList:	
					subsetData[j][k] = np.squeeze(data[j][k][[indicesList[k]],:])
					subsetLabels[k] = [labelsList[k][i] for i in indicesList[k]]

		#apply indices
		# count = 0
		# off1 = 3
		# off2 = 2
		# off3 = 15
		# off4 = 25
		# for k in range(numFiles):
		# 	for j in jointsList + objectsList:
		# 		count += 1
		# 		print str(count).ljust(off1) + ' Folder ' + str(k).ljust(off2) + ' object: ' + j.ljust(off3) + \
		# 		' data shape: '.ljust(off4) + str(len(data)) + ' ' + str(len(data[j])) + ' ' + str(data[j][k].shape)
		# 		print str(count).ljust(off1) + ' Folder ' + str(k).ljust(off2) + ' object: ' + j.ljust(off3) + \
		# 		' subset data shape: '.ljust(off4) + str(len(subsetData)) + ' ' + str(len(subsetData[j])) + ' ' + str(subsetData[j][k].shape)
		# 		print str(count).ljust(off1) + ' Folder ' + str(k).ljust(off2) + ' object: ' + j.ljust(off3) + \
		# 		' labels shape: '.ljust(off4) + str(len(labelsList)) + '     ' + str(len(labelsList[k]))
		# 		print str(count).ljust(off1) + ' Folder ' + str(k).ljust(off2) + ' object: ' + j.ljust(off3) + \
		# 		' subset labels shape: '.ljust(off4) + str(len(subsetLabels)) + '     ' + str(len(subsetLabels[k]))
		# 		print str(count).ljust(off1) + ' Folder ' + str(k).ljust(off2) + ' object: ' + j.ljust(off3) + \
		# 		' good inds shape: '.ljust(off4) + str(len(indicesList)) + '     ' + str(len(indicesList[k]))
		# 		print

		allObjs = jointsList + objectsList
		print
		print 'Unique joints and objects in data files: '
		print
		print '\n'.join(allObjs)
		print 
		print 'Of these, config.ini specifies [' + ', '.join(ignoreParts) + '] to be ignored'
		print

		remObjs = ignoreParts
		impObjs = copy.deepcopy(allObjs)
		for x in remObjs:
			impObjs.remove(x)
		objCombs = list(combinations(impObjs, 2))
		print 'Creating ' + str(len(objCombs)) + ' combinations out of ' + str(len(impObjs)) + ' objects'  
		print 
		self.objCombs = objCombs
		qtcDataList = dict()
		angleThreshold = self.angleThreshold
		distanceThreshold = self.distanceThreshold
		contactThreshold = self.contactThreshold
		allJoints = []
		#for all combs
		for arr in range(len(subsetData[objCombs[0][0]])):
			print 'Preprocessing file ' + str(arr)
			jointArr = None
			for currComb in objCombs:
				Pk = subsetData[currComb[0]]
				Pl = subsetData[currComb[1]]
				#for all arrays in Pk and Pl
				currPk = Pk[arr]
				currPl = Pl[arr]

				q1 = np.zeros(currPk.shape[0]-2, dtype=np.int)
				q2 = np.zeros(currPk.shape[0]-2, dtype=np.int)
				q3 = np.zeros(currPk.shape[0]-2, dtype=np.int)
				q4 = np.zeros(currPk.shape[0]-2, dtype=np.int)
				q5 = np.zeros(currPk.shape[0]-2, dtype=np.int)
				q6 = np.zeros(currPk.shape[0]-2, dtype=np.int)
				#currPk contains xyz and currPl contains xyz
				#calculate QTC
				#step 1: q1 = {-1,0,+1} Pk relative to Pl
				qtc_2D(currPk,currPl,q1, distanceThreshold, contactThreshold, q6)
				#step 2: q2 = {-1,0,+1} Pl relative to Pk
				qtc_2D(currPl,currPk,q2, distanceThreshold, contactThreshold)
				#step 3: calculate q3, q4 and q5
				qtc_3D(currPk, currPl, angleThreshold, q3, q4, q5)
				if(numQ == 5):
					tempArr = np.vstack((q1,q2,q3,q4,q5))
				elif(numQ == 6):
					tempArr = np.vstack((q1,q2,q3,q4,q5,q6))
				if(jointArr == None):
					jointArr = tempArr.T
				else:
					jointArr = np.hstack((jointArr,tempArr.T))
			allJoints.append(jointArr)

			#updating labelsList
			subsetLabels[arr] = subsetLabels[arr][1:-1]

		#create mask to temporaly segment actions temporally
		mask = np.zeros(allJoints[0].shape[1], dtype = int)
		for i in range(0,len(mask),6):
		    mask[i+0] = 1
		    mask[i+1] = 1

		actionsIdxList = []
		actionsLabelsList = []

		#for each item in list
		print
		print 'Temporal segmentation of data:'
		print
		for k in range(len(allJoints)):
			print 'Segmenting file ' + str(k)
			actionsIdxList.append([])
			actionsLabelsList.append([])
			#create array that masks q2,q3 and q4
			maJoints = allJoints[k]*mask
			#sum abs(rows) of array
			maAbs = np.abs(maJoints)
			maSum = np.sum(maAbs, axis = 1)
			#indices with maAbs =  0 are regions with no movement 
			#.ie action demarcation between actions
			#maSum = maSum[:50]
			actionCount = 0
			actionsIndices = None
			actionsLabels = []
			started = False

			for n in range(len(maSum)):
				if(maSum[n] == 0):
					#here we need to close action if previous is not zero
					#start action if next is not zero
					#ignore otherwise

					#if n-1 not 0 end action
					#check n-1 exists 
					if(n-1 >= 0):
						if(maSum[n-1] != 0):
							#end action including 0 at n
							actionsIndices = np.hstack((actionsIndices, allJoints[k][n]))
							actionsLabels.append(subsetLabels[k][n])
							started = False
							actionsIdxList[k].append(actionsIndices)
							actionsLabelsList[k].append(actionsLabels)
							actionsIndices = None
							actionsLabels = []

					#check n+1 exists
					if(n+1 < len(maSum)):
						#if n+1 exists but not 0 start an action
						if(maSum[n+1] != 0):
							#start action including 0 at n
							actionsIndices = allJoints[k][n]
							actionsLabels.append(subsetLabels[k][n])
							started = True
						#else ignore current index
					#else we are at the end so ignore

				else: #current index is not zero
					if(started):
						#here if started = True we are in middle of action so concatenate
						actionsIndices = np.hstack((actionsIndices, allJoints[k][n]))
						actionsLabels.append(subsetLabels[k][n])
						if(n+1 == len(maSum)):
							actionsIdxList[k].append(actionsIndices)
							actionsLabelsList[k].append(actionsLabels)
					else:
						#here action has not started meaning a zero was not found
						#this occurs if vector does not start with a zero
						actionsIndices = allJoints[k][n]
						actionsLabels.append(subsetLabels[k][n])
						started = True
		#find maximum length vector for SAM
		maxLen = 0
		for n in actionsIdxList:
			for k in n:
				if(k.shape[0] > maxLen):
					maxLen = k.shape[0]
		print
		#create Y and L
		Y = None
		L = []
		for n in range(len(actionsIdxList)):
			for k in range(len(actionsIdxList[n])):
				currLen = len(actionsIdxList[n][k])
				augMat = np.zeros(maxLen-currLen)
				if(Y == None): 
					Y = np.hstack((actionsIdxList[n][k],augMat))
				else:
					Y = np.vstack((Y, np.hstack((actionsIdxList[n][k],augMat))))
				L.append(actionsLabelsList[n][k])

		L2 = [most_common(sublist) for sublist in L]
		Larr = np.zeros(len(L2))
		for f in range(len(L2)):
			Larr[f] = labels.index(L2[f])

		print
		self.Y = Y
		self.L = Larr[:,None]
		self.labelName = labels

    def prepareData(self, model='mrd', Ntr = 50, randSeed=0):    

        Nts=self.Y.shape[0]-Ntr
        np.random.seed(randSeed)
        perm = np.random.permutation(self.Y.shape[0])
        indTs = perm[0:Nts]
        indTs.sort()
        indTr = perm[Nts:Nts+Ntr]
        indTr.sort()
        YtestAll = self.Y[indTs].copy() ##
        self.Ytest = self.Y[indTs]
        LtestAll = self.L[indTs].copy()##
        self.Ltest = self.L[indTs]
        Yall = self.Y[indTr].copy()##
        self.Y = self.Y[indTr]
        Lall = self.L[indTr].copy()##
        self.L = self.L[indTr]
    
        # Center data to zero mean and 1 std
        # self.Ymean = self.Y.mean()
        # self.Yn = self.Y - self.Ymean
        # self.Ystd = self.Yn.std()
        # self.Yn /= self.Ystd
        self.Yn = self.Y
        # Normalise test data similarly to training data
        #self.Ytestn = self.Ytest - self.Ymean
        self.Ytestn = self.Ytest
        #self.Ytestn /= self.Ystd

        if model == 'mrd':    
            self.X=None     
            self.Y = {'Y':self.Yn,'L':self.L}
            self.data_labels = self.L.copy()
        elif model == 'bgplvm':
            self.X=None     
            self.Y = {'Y':self.Yn}
            self.data_labels = self.L.copy()
        return Yall, Lall, YtestAll, LtestAll
#-----------------------------------------------------------

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
		if(parser.has_option(trainName, 'ignoreLabels')):
			ignoreLabels = parser.get(trainName, 'ignoreLabels').split(',')
		else:
			ignoreLabels = ['agent_entry','agent_exit','no_agent']

		if(parser.has_option(trainName, 'ignoreParts')):
			ignoreParts = parser.get(trainName, 'ignoreParts').split(',')
		else:
			ignoreParts = ['partner','car','octopus']

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

		if(parser.has_option(trainName, 'angleThreshold')):
			angThresh = float(parser.get(trainName, 'angleThreshold'))
		else:
			angThresh = 0.01

		if(parser.has_option(trainName, 'distanceThreshold')):
			distThresh = float(parser.get(trainName, 'distanceThreshold'))
		else:
			distThresh = 0.01

		if(parser.has_option(trainName, 'contactThreshold')):
			contThresh = float(parser.get(trainName, 'contactThreshold'))
		else:
			contThresh = 0.01

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

		if(parser.has_option(trainName, 'Q')):
			Quser = int(parser.get(trainName, 'Q'))
		else:
			Quser = 2

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
	ignoreLabels = modelPickle['ignoreLabels']
	ignoreParts = modelPickle['ignoreParts']
	angThresh = modelPickle(['angleThreshold'])
	distThresh = modelPickle(['distanceThreshold'])
	contThresh = modelPickle(['contactThreshold'])
	Quser = modelPickle['Quser']
	ratioData = modelPickle['percentTestData']
	model_type = modelPickle['model_type']
	model_num_inducing = modelPickle['num_inducing']
	model_init_iterations = modelPickle['model_init_iterations']
	model_num_iterations = modelPickle['model_num_iterations']
	kernelString = modelPickle['kernelString']

# # Creates a SAMpy object
mySAMpy = AR_Driver(yarpRunning)

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
mySAMpy.contactThreshold = contThresh
mySAMpy.distanceThreshold = distThresh
mySAMpy.angleThreshold = angThresh

# # Reading face data, preparation of data and training of the model
mySAMpy.readData(dataPath, participantList)

minData = mySAMpy.Y.shape[0]

print mySAMpy.Y.shape
print 'minData = ' + str(minData)
Ntr = int(minData*ratioData/100)
Ntest = minData - Ntr
mySAMpy.Quser = Quser
[Yall,Lall,YtestAll,LtestAll] = mySAMpy.prepareData(model_type, Ntr,randSeed=experiment_number)
mySAMpy.training(model_num_inducing, model_num_iterations, model_init_iterations, fname, save_model, economy_save, keepIfPresent = False) #change to

ss = [];
sstest = [];

print
off1 = 15
off2 = 8
allCount = Yall.shape[0]
# allCount = 10
off3 = len(str(allCount))
#compare with training data
for i in range(allCount):
	currTestData = Yall[i][None,:]
	currLabel = mySAMpy.labelName[int(Lall[i])] 

	[lab, confidence] = mySAMpy.testing(currTestData)
	
	if(currLabel == lab):	
		result = True
	else:
		result = False
	print str(i).rjust(off3) + '/' + str(allCount) + ' Truth: ' + currLabel.ljust(off1) + ' Model: ' + lab.ljust(off1) + ' with ' + str(confidence)[:6].ljust(off2) + ' confidence: ' + str(result)
	ss.append(result)

correctVal = sum(ss)
percCorect = correctVal*100/allCount
print str(percCorect) + " percent correct for training data"
print '\n\n\n\n\n\n\n\n\n'

allCount = YtestAll.shape[0]
# allCount = 10
off3 = len(str(allCount))
for i in range(allCount):
	currTestData = YtestAll[i][None,:]
	currLabel = mySAMpy.labelName[int(LtestAll[i])] 

	[lab, confidence] = mySAMpy.testing(currTestData)
	
	if(currLabel == lab):	
		result = True
	else:
		result = False
	print str(i).rjust(off3) + '/' + str(allCount) + ' Truth: ' + currLabel.ljust(off1) + ' Model: ' + lab.ljust(off1) + ' with ' + str(confidence)[:6].ljust(off2) + ' confidence: ' + str(result)
	sstest.append(result)
	
correctVal = sum(sstest)
percCorect = correctVal*100/allCount
print str(percCorect) + " percent correct for testing data"

#save model with custom .pickle dictionary by iterating through all nested models
fname_cur = fname
print 'Saving: ' + fname_cur
extraParams = dict()
extraParams['YALL'] = Yall
extraParams['LALL'] = Lall
extraParams['YTEST'] = YtestAll
extraParams['LTEST'] = LtestAll
extraParams['objCombs'] = mySAMpy.objCombs
extraParams['ignoreLabels'] = ignoreLabels
extraParams['ignoreParts'] = ignoreParts
extraParams['contactThreshold'] = mySAMpy.contactThreshold
extraParams['angleThreshold'] = mySAMpy.angleThreshold
extraParams['distanceThreshold'] = mySAMpy.distanceThreshold
extraParams['Quser'] = Quser
extraParams['percentTestData'] = ratioData
extraParams['model_num_iterations'] = model_num_iterations
extraParams['model_init_iterations'] = model_init_iterations
extraParams['model_type'] = model_type
extraParams['textLabels'] = mySAMpy.labelName
SAMCore.save_pruned_model(mySAMpy.SAMObject, fname_cur, economy_save, extraDict=extraParams)
