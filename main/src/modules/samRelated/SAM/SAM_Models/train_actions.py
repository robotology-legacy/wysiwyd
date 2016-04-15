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

%matplotlib inline
import matplotlib
import matplotlib.pyplot as plt
import readline
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
from SAM.SAM_Core import staticPose
numQ = 6
warnings.simplefilter("ignore")

#--------------------functions------------------------------
def distEuc(a,b):
    temp = a-b
    temp = np.square(temp)
    temp = np.sum(temp,1)
    return np.sqrt(temp)

def qtc_2D(k,l,q,thresh):
    
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
    
    return d2

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
    alpArr = np.zeros(q3.shape)
    betArr = np.zeros(q3.shape)
    gamArr = np.zeros(q3.shape)
    
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
        alpArr[g] = alpha
        betArr[g] = beta
        gamArr[g] = gamma
        
        q3[g] = np.sign(alpha)
        q4[g] = np.sign(beta)
        q5[g] = np.sign(gamma)
    
    return [alpArr,betArr,gamArr]
            
def checkQTC(qtcArr):
    #array of shape x by 11 with 0 to 4 of 11 being the ones that unique must apply to
    tArr = copy.deepcopy(qtcArr)
    for i in range(1,tArr.shape[0]):
        if(i != 0):
            currRow = tArr[i,:]
            Mod = False
            for q in range(tArr.shape[1]):
                r = tArr[i,q] - tArr[i-1,q]
                if(r == -2 or r == 2):
                    Mod = True
                    currRow[q] = 0
            if(Mod):    
                qtcArr = np.insert(qtcArr,i,currRow,axis=0)    
                Mod = False
    return qtcArr

def uniqueQTC(currQTC):
    #array of shape x by 11 with 0 to 4 of 11 being the ones that unique must apply to
    i = 0
    del1 = False
    del2 = False
    
    while(i < currQTC.shape[0]-2):
        if(i!=0):
            if(np.array_equal(currQTC[i,:5],currQTC[i-1,:5])):
                del1 = True
            if(np.array_equal(currQTC[i+1,:5],currQTC[i-1,:5])):
                del2 = True
        
        if(del1 and not del2):
            currQTC = np.delete(currQTC,i,0)
        elif(del2):
            currQTC = np.delete(currQTC,i,0)
            currQTC = np.delete(currQTC,i+1,0)
            
        i+=1
    return currQTC

def labelize(qin, qout):
    for h in range(qin.shape[0]):
        #x at 0 is depth with negative meaning behind an object
        #y at 1 is width with negative meaning partner-right = ego-left
        #x at 2 is height with negative meaning underneath
        currVec = np.abs(qin[h,:])
        if(currVec.sum() != 0):
            maxIDx = currVec.argmax()
            if(maxIDx == 0):
                if(np.sign(qin[h,maxIDx]) == -1):
                    #behind
                    qout[h] = humanStaticLabels.index('behind')
                elif(np.sign(qin[h,maxIDx]) == 1):
                    #in front
                    qout[h] = humanStaticLabels.index('in front of')
            elif(maxIDx == 1):
                if(np.sign(qin[h,maxIDx]) == -1):
                    #left
                    qout[h] = humanStaticLabels.index('left of')
                elif(np.sign(qin[h,maxIDx]) == 1):
                    #right
                    qout[h] = humanStaticLabels.index('right of')
            elif(maxIDx == 2):
                if(np.sign(qin[h,maxIDx]) == -1):
                    #underneath
                    qout[h] = humanStaticLabels.index('underneath')
                elif(np.sign(qin[h,maxIDx]) == 1):
                    #top of
                    qout[h] = humanStaticLabels.index('on top of')  
        else:
            qout[h] = humanStaticLabels.index('stationary')

def filterMovement(ts, thresh):
    lastMove = 0
    stillThere = True
    
    for timestep in range(1, ts.shape[0]):
        g = np.sqrt(np.sum(np.square(ts[timestep,:]-ts[lastMove,:])))
        if(g >= thresh):
            lastMove = timestep
            stillThere = False
#             print str(g).ljust(20) + ' shift'
        else:
#             print str(g).ljust(20) + ' still there'
            ts[timestep] = ts[lastMove]
#             if(stillThere):
#                 print 'delete'
#                 #np.delete(ts, timestep, axis = 0)
#             else:
#                 print 'no delete'
#                 stillThere = True

def removeStationary(inputArr, labelRemove, indsToCheck):
    i = 0
    while(i < inputArr.shape[0]):
        logic = True 
        for j in indsToCheck: 
            if(inputArr[i][j] == labelRemove):
                logic = logic and True
            else:
                logic = logic and False
        
        if(logic): #if labelRemove was present in all indsToCheck remove row
            inputArr = np.delete(inputArr,i,0)
        else:
            i += 1
    return inputArr

def formatFeatures(inputArr, idx):
    vec = np.zeros((len(humanStaticLabels)+1))
    for i in range(len(inputArr)):
        vec[inputArr[i][idx]] += 1
    vec[-1] = len(inputArr)
    return vec
        
def formatFeatures2(inputArr, idx, valsPossible):
#     print str(valsPossible) + str(len(valsPossible))
#     print str(idx) +str(len(idx))
    if(valsPossible != None):
        vec = np.zeros((len(valsPossible)*len(idx)))
        for b in range(len(idx)):
            for i in range(len(inputArr)):
                
                offset = (len(valsPossible)*b)
                vec[offset + valsPossible.index(inputArr[i][idx[b]])] += 1
#                 print
#         print vec
        return vec
    else:
        return np.array(-1)
    
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

					v = labelLine.split(' ')[2].replace('\n','').replace('(','').replace(')','')
            		if(v == ''):
                		v = 'unknown'
            		labelsList[k].append(v)

					#parse skeleton data which has 9 sections by (x,y,z)
					for i in range(numJoints):
						a = i*4
						if(t[a] == 'shoulderCenter'):
                    		t[a] = 'chest'

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
							objectsList.append(t[a])

					firstPass = False
				dataFile.close()
				labelFile.close()

		print 'data has length = ' + str(len(data)) + ' joints'
		strl =  'each joint has ' + str(len(data['head'])) + ' arrays of shape: '

		for i in data['head']:
		    strl += str(i.shape) + ', '
		print strl

		strl = 'labelsList has length = ' + str(len(labelsList)) + ' with sizes: '
		for i in labelsList:
		    strl += str(len(i)) + ', '
		print strl
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

		# ignoreInds = []
		# for k in ignoreLabels:
		# 	ignoreInds.append(labels.index(k))

		dataStruct = []
		verbose = True
		for arr in range(len(dataFolderList)):
		    for label in labels:
		        if(verbose):
		            print
		            print 'current dataset = ' + str(dataFolderList[arr])
		            print 'curent label: ' + str(label)
		            print
		        idxs = [i for i in range(len(labelsList[arr])) if labelsList[arr][i] == label]
		        actionBlocks = []
		        startIdx = 0
		        if(len(idxs) > 5):
		            for idxIndex in range(len(idxs)):
		                if(idxIndex != 0):
		                    if(idxs[idxIndex] - idxs[idxIndex-1] != 1):
		                        if(idxIndex-startIdx > 5):
		                            for joint in jointsList+objectsList[1:]:
		                                actionData = data[joint][arr][idxs[startIdx:idxIndex]]
		                                dataStruct.append([joint, label, arr, actionData.shape[0], actionData])
		                                if(verbose):
		                                    print '[%s]' % ', '.join(map(str, dataStruct[-1][:-1]))
		                            startIdx = idxIndex

		                    if(idxIndex+2 > len(idxs)):
		                        if(idxs[idxIndex] - idxs[idxIndex-1] != 1):
		                            for joint in jointsList+objectsList[1:]:
		                                actionData = data[joint][arr][idxs[startIdx:idxIndex+1]]
		                                dataStruct.append([joint, label, arr, actionData.shape[0], actionData])
		                                if(verbose):
		                                    print '[%s]' % ', '.join(map(str, dataStruct[-1][:-1]))
		            print '------------------------------------------------------------------------'
		joint = 0
		action = 1
		dataset = 2
		start = 3
		end = 4
		for a in range(len(dataFolderList)):
		    print 'Dataset ' + str(a) + ' : ' + dataFolderList[a]
		    for b in range(len(labels)):
		        if('no' not in labels[b]):
		            y = len([i[1] for i in dataStruct if i[joint] == 'head' and i[dataset] == a and i[action] == labels[b]])
		            print '\t ' + str(labels[b]).ljust(35) + ' = ' + str(y).ljust(3) + ' repetitions'
		    print


		joint = 0
		action = 1
		dataset = 2
		start = 3
		end = 4

		for b in range(len(labels)):
		    if('no' not in labels[b]):
		        y = 0
		        for a in range(len(dataFolderList)):
		            y += len([i[1] for i in dataStruct if i[joint] == 'head' and i[dataset] == a and i[action] == labels[b]])
		        print '\t ' + str(labels[b]).ljust(35) + ' = ' + str(y).ljust(3) + ' repetitions'
		
		#Calibrate contact threshold
		# data has length = 12 joints
		# each joint has 4 arrays of shape: (4637, 3), (4907, 3), (5914, 3), (5440, 3), 
		# labelsList has length = 4 with sizes: 4637, 4907, 5914, 5440, 
		handPos = np.zeros((1,3))
		objectPos = np.zeros((1,3))

		# push_object_car_hand_right

		for i in range(len(labelsList)):
		    for j in range(len(labelsList[i])):
		        if('object' in labelsList[i][j] and 'reach' not in labelsList[i][j] and "no" not in labelsList[i][j]):
		            if('left' in labelsList[i][j]):
		                handPos = np.vstack((handPos,data['handLeft'][i][j,:,None].T))
		                obj = labelsList[i][j].split('_')[2]
		                objectPos = np.vstack((objectPos,data[obj][i][j,:,None].T))
		                
		            if('right' in labelsList[i][j]):
		                handPos = np.vstack((handPos,data['handRight'][i][j,:, None].T))
		                obj = labelsList[i][j].split('_')[2]
		                objectPos = np.vstack((objectPos,data[obj][i][j,:,None].T))

		d = distEuc(handPos, objectPos)
		perc = 98
		print 'Mean of contact distances = ' + str(np.mean(d))
		print 'Median of contact distances = '  + str(np.median(d))
		print 'Max contact distance = ' + str(np.max(d))
		print 'Min contact distance = ' + str(np.min(d))
		print str(perc)+'% percentile contact distance = ' + str(np.percentile(d,perc))

		jointsList.sort()
		modJointsListHand_R = [i for i in jointsList if i != 'elbowRight' and i != 'handRight']
		modJointsListHand_L = [i for i in jointsList if i != 'elbowLeft' and i != 'handLeft']

		modJointsListHand_R = []
		modJointsListHand_L = []
		combinationList = []

		if(len(modJointsListHand_R) > 0):
		    for i in range(len(modJointsListHand_R)):
		        combinationList.append(['handLeft', modJointsListHand_L[i]])
		        combinationList.append(['handRight', modJointsListHand_R[i]])
		    del combinationList[combinationList.index(['handLeft', 'handRight'])]

		for i in objectsList[1:]:
		    combinationList.append(['handLeft',i])
		    combinationList.append(['handRight',i])


		print 'Available joint pairs for hands:'
		print
		for i in combinationList:
		    print '\t' + str(i)

		humanStaticLabels = []
		humanStaticLabels.append('left of')
		humanStaticLabels.append('right of')
		humanStaticLabels.append('on top of')
		humanStaticLabels.append('underneath')
		humanStaticLabels.append('in front of')
		humanStaticLabels.append('behind')
		humanStaticLabels.append('stationary')

		joint = 0
		action = 1
		dataset = 2
		start = 3
		end = 4

		actionsAllowedList = ['lift_object', 'pull_object', 'push_object', 'drop_object', 'carry_object']
		#actionsAllowedList = ['lift_object', 'drop_object']
		actionsAllowed = []

		for ac in actionsAllowedList:
		    temp = [g for g in labels if ac in g and "no" not in g]
		    for h in range(len(temp)):
		        actionsAllowed.append(temp[h])

		for b in range(len(labels)):
		#     print labels[b]
		    if('no' not in labels[b] and labels[b] in actionsAllowed):
		        y = 0
		        for a in range(len(dataFolderList)):
		            y += len([i[1] for i in dataStruct if i[joint] == 'head' and i[dataset] == a and i[action] == labels[b]])
		        print '\t ' + str(labels[b]).ljust(35) + ' = ' + str(y).ljust(3) + ' repetitions'

		#2 joint pairs will be used shoulder-hand and hand-object left and right doesnt matter
		#both combine within a single model
		#step 1 will be training these models with static poses to do clustering with a full GP configuration
		joint = 0
		action = 1
		dataset = 2
		start = 3
		end = 4
		verbose = True

		contactThreshold = 0.2 #between 0.1 and 0.2 value smaller than which contact occurs
		deltaDistanceThreshold = 0.01 #distance to move between frames for point-point relative movement to be considered true1
		angleThreshold = 0.001
		labelToRemove = humanStaticLabels.index('stationary');
		indsToRemove = [18]

		firstPass = True
		handDataStruct = []

		for currComb in combinationList:
		    print str(currComb[0]) +'-' +str(currComb[1])
		    infoK = [i for i in dataStruct if i[joint] == currComb[0]]
		    infoL = [i for i in dataStruct if i[joint] == currComb[1]]
		    for i in range(len(infoK)):
		        if(infoK[i][1] in actionsAllowed):
		            if('hand' in infoK[i][0] and 'object' in infoK[i][1]):
		                if('Left' in infoK[i][0] and 'left' in infoK[i][1]):
		                    condition = True
		                elif('Right' in infoK[i][0] and 'right' in infoK[i][1]):
		                    condition = True
		                else:
		                    condition = False
		                    
		                if(infoL[i][0] not in infoK[i][1]):
		                    condition = False
		                    obj = True
		                    
		            elif('hand' not in infoK[i][0] and 'object' in infoK[i][1]):
		                condition = False
		            else:
		                condition = True
		        else:
		            condition = False
		             
		            
		#         condition = False
		        if(condition):
		            
		            Pk = infoK[i][4]
		            Pl = infoL[i][4]

		            #add gaussian noise on output
		            Pk_Noise = np.random.normal(jointMu, jointSig, Pk.shape)
		            Pl_Noise = np.random.normal(jointMu, jointSig, Pl.shape)

		            Pk = Pk + Pk_Noise
		            Pl = Pl + Pl_Noise

		            q1 = np.zeros((Pk.shape[0]-2,1), dtype=np.int) #motion of k relative to l
		            q2 = np.zeros((Pk.shape[0]-2,1), dtype=np.int) #motion of l relative to k
		            q3 = np.zeros((Pk.shape[0]-2,1), dtype=np.int) #alpha
		            q4 = np.zeros((Pk.shape[0]-2,1), dtype=np.int) #beta
		            q5 = np.zeros((Pk.shape[0]-2,1), dtype=np.int) #gamma
		            q7 = np.zeros((Pk.shape[0]-2,1), dtype=np.int) #contact
		            q8 = np.zeros((Pk.shape[0]-2,1), dtype=np.int) #human static label
		            
		            filterMovement(Pk, deltaDistanceThreshold)
		            filterMovement(Pl, deltaDistanceThreshold)
		            

		            q6 = (Pl[1:-1]-Pk[1:-1]) #direction vector from joint to joint

		            q9 = qtc_2D(Pk,Pl,q1, deltaDistanceThreshold)
		            lowV = np.abs(q9) < contactThreshold
		            q7[lowV] = 1

		            q6 = q6/q9[:,None]

		            qtc_2D(Pl,Pk,q2, deltaDistanceThreshold)
		            [a,b,g] = qtc_3D(Pk, Pl, angleThreshold, q3, q4, q5)
		            
		            labelize(q6,q8)

		            q10 = Pl[1:-1]
		            q11 = Pk[1:-1]
		            
		            q13 = np.ones((Pk.shape[0]-2,1), dtype=np.int)
		            q12mag = distEuc(Pk[1:-1],Pk[:-2])
		            lowV = np.abs(q12mag) < deltaDistanceThreshold
		            q12 = (Pk[1:-1] - Pk[:-2])/q12mag[:,None]
		            q12[lowV] = 0
		            labelize(q12,q13)

		            q15 = np.ones((Pk.shape[0]-2,1), dtype=np.int)
		            q14mag = distEuc(Pl[1:-1],Pl[:-2])
		            lowV = np.abs(q14mag) < deltaDistanceThreshold
		            q14 = (Pl[1:-1] - Pl[:-2])/q14mag[:,None]
		            q14[lowV] = 0
		            labelize(q14,q15)
		                
		            tempQTC = np.hstack((q1,q2,q3,q4,q5,q6[:,0,None],q6[:,1,None],q6[:,2,None],q7,q8,q9[:,None],q10[:,0,None],q10[:,1,None],q10[:,2,None],q11[:,0,None],q11[:,1,None],q11[:,2,None],q13,q15))
		    #         g = tempQTC.shape[0]
		    #         tempQTC2 = uniqueQTC(tempQTC)

		    #         gg = tempQTC2.shape[0]

		    #         tempQTC = checkQTC(tempQTC2)

		    #         ggg = tempQTC.shape[0]
		            
		            tempQTC = removeStationary(tempQTC, labelToRemove, indsToRemove)
		#             v = formatFeatures(tempQTC, 18)
		            handDataStruct.append([currComb[0], currComb[1], infoK[i][1], infoK[i][2], tempQTC.shape[0], tempQTC])

		            if(verbose):
		                print '[%s]' % ', '.join(map(str, handDataStruct[-1][:-1]))
		                #print v
		    #             print str((ggg-gg)).ljust(3) + ' added in check QTC'
		    #             print str((gg-g)).ljust(3) + ' added in unique QT
		#q1,q2         [0:2]   -> relative movement of points in terms of distance from each other (closer / farther)
		#q3-5          [2:5]   -> relative movement of joints in tems of orientation from each other (difference in the paths they are following wrt each other)
		#q6_1 to _3    [5:8]   -> direction vector from hand to other joint
		#q7            [8]     -> contact status
		#q8            [9]     -> instantaneous relative position of K wrt L
		#q9            [10]    -> distance of points
		#q10_1 to _3   [11:14] -> position of l
		#q11_1 to _3   [14:17] -> position of k

		featureSections = dict()
		featureValues = dict()
		featureSections['QTC_Motion'] = range(0,2)
		featureValues['QTC_Motion'] = [-1,0,1]

		featureSections['QTC_Orientation'] = range(2,5)
		featureValues['QTC_Orientation'] = [-1,0,1]

		featureSections['directionVector'] = range(5,8)
		featureValues['directionVector'] = []

		featureSections['contact'] = [8]
		featureValues['contact'] = [0,1]

		featureSections['relativePositionLabel'] = [9]
		featureValues['relativePositionLabel'] = range(len(humanStaticLabels))

		featureSections['euc_distance'] = [10]
		featureValues['euc_distance'] = []

		featureSections['posL'] = range(11,14)
		featureValues['posL'] = []

		featureSections['posK'] = range(14,17)
		featureValues['posK'] = []

		featureSections['selfMovementLabelL'] = [17]
		featureValues['selfMovementLabelL']   = range(len(humanStaticLabels))

		featureSections['selfMovementLabelK'] = [18]
		featureValues['selfMovementLabelK']   = range(len(humanStaticLabels))


		featureToUse = ['contact', 'selfMovementLabelK']
		#selfMovementLabelK movement of hand
		#selfMovementLabelL movement of object
		featureInds = []
		for h in featureToUse:
		    featureInds += featureSections[h]
		    
		    
		print featureInds

		for i in range(len(humanStaticLabels)):
    		print str(i).ljust(2) + ': ' + str(humanStaticLabels[i])

    	for n in range(len(handDataStruct)):
		#     print handDataStruct[n][0:3]
		#     print featureToUse
		#     print handDataStruct[n][5][:,featureInds]
		    v = np.array(len(handDataStruct[n][5]))
		    for j in featureToUse:
		        vec = formatFeatures2(handDataStruct[n][5], featureSections[j], featureValues[j])
		        if(vec[0] !=  -1):
		            v = np.hstack((v,vec))
		    handDataStruct[n][5] = v
		#here model trains recognition of object location wrt each other
		#feature vector => handDataStruct[5][:,5:8](vector from hand to object) and humanlabel handDataStruct[5][:,9]
		#actionsAllowedList = ['lift_object', 'pull_object', 'push_object', 'drop_object',]

		for a in range(len(handDataStruct)):
		    for r in actionsAllowedList:
		        if r in handDataStruct[a][2]:
		            labelNum = actionsAllowedList.index(r)

		    if(a == 0):
		        allPoseY = handDataStruct[a][5]
		        allPoseL = labelNum
		    else:
		        allPoseY = np.vstack((allPoseY, handDataStruct[a][5]))
		        allPoseL = np.vstack((allPoseL, labelNum))
		        
		print allPoseY.shape
		print allPoseL.shape
		#here model trains recognition of object location wrt each other
		#feature vector => handDataStruct[5][:,5:8](vector from hand to object) and humanlabel handDataStruct[5][:,9]
		#actionsAllowedList = ['lift_object', 'pull_object', 'push_object', 'drop_object',]

		for a in range(len(handDataStruct)):
		    for r in actionsAllowedList:
		        if r in handDataStruct[a][2]:
		            labelNum = actionsAllowedList.index(r)

		    if(a == 0):
		        allPoseY = handDataStruct[a][5]
		        allPoseL = labelNum
		    else:
		        allPoseY = np.vstack((allPoseY, handDataStruct[a][5]))
		        allPoseL = np.vstack((allPoseL, labelNum))
		        
		print allPoseY.shape
		print allPoseL.shape




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
