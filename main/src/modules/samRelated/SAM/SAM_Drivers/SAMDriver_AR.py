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
import math
import random

class SAMDriver_AR(SAMDriver):

    def __init__(self, isYarpRunning = False):
        SAMDriver.__init__(self, isYarpRunning)

    def labelize(self, qin, qout):
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
                        qout[h] = self.humanStaticLabels.index('behind')
                    elif(np.sign(qin[h,maxIDx]) == 1):
                        #in front
                        qout[h] = self.humanStaticLabels.index('in front of')
                elif(maxIDx == 1):
                    if(np.sign(qin[h,maxIDx]) == -1):
                        #left
                        qout[h] = self.humanStaticLabels.index('left of')
                    elif(np.sign(qin[h,maxIDx]) == 1):
                        #right
                        qout[h] = self.humanStaticLabels.index('right of')
                elif(maxIDx == 2):
                    if(np.sign(qin[h,maxIDx]) == -1):
                        #underneath
                        qout[h] = self.humanStaticLabels.index('underneath')
                    elif(np.sign(qin[h,maxIDx]) == 1):
                        #top of
                        qout[h] = self.humanStaticLabels.index('on top of')  
            else:
                qout[h] = self.humanStaticLabels.index('stationary')

    def extractFeatures(self, Pk, Pl):
        #add gaussian noise on output
        Pk_Noise = np.random.normal(self.jointMu, self.jointSig, Pk.shape)
        Pl_Noise = np.random.normal(self.jointMu, self.jointSig, Pl.shape)

        Pk = Pk + Pk_Noise
        Pl = Pl + Pl_Noise

        q1 = np.zeros((Pk.shape[0]-2,1), dtype=np.int) #motion of k relative to l
        q2 = np.zeros((Pk.shape[0]-2,1), dtype=np.int) #motion of l relative to k
        q3 = np.zeros((Pk.shape[0]-2,1), dtype=np.int) #alpha
        q4 = np.zeros((Pk.shape[0]-2,1), dtype=np.int) #beta
        q5 = np.zeros((Pk.shape[0]-2,1), dtype=np.int) #gamma
        q7 = np.zeros((Pk.shape[0]-2,1), dtype=np.int) #contact
        q8 = np.zeros((Pk.shape[0]-2,1), dtype=np.int) #human static label

        self.filterMovement(Pk, self.deltaDistanceThreshold)
        self.filterMovement(Pl, self.deltaDistanceThreshold)


        q6 = (Pl[1:-1]-Pk[1:-1]) #direction vector from joint to joint

        q9 = self.qtc_2D(Pk,Pl,q1, self.deltaDistanceThreshold)
        lowV = np.abs(q9) < self.contactThreshold
        q7[lowV] = 1

        q6 = q6/q9[:,None]

        self.qtc_2D(Pl,Pk,q2, self.deltaDistanceThreshold)
        [a,b,g] = self.qtc_3D(Pk, Pl, self.angleThreshold, q3, q4, q5)

        self.labelize(q6,q8)

        q10 = Pl[1:-1]
        q11 = Pk[1:-1]

        q13 = np.ones((Pk.shape[0]-2,1), dtype=np.int)
        q12mag = self.distEuc(Pk[1:-1],Pk[:-2])
        lowV = np.abs(q12mag) < self.deltaDistanceThreshold
        q12 = (Pk[1:-1] - Pk[:-2])/q12mag[:,None]
        q12[lowV] = 0
        self.labelize(q12,q13)

        q15 = np.ones((Pk.shape[0]-2,1), dtype=np.int)
        q14mag = self.distEuc(Pl[1:-1],Pl[:-2])
        lowV = np.abs(q14mag) < self.deltaDistanceThreshold
        q14 = (Pl[1:-1] - Pl[:-2])/q14mag[:,None]
        q14[lowV] = 0
        self.labelize(q14,q15)

        tempQTC = np.hstack((q1,q2,q3,q4,q5,q6[:,0,None],q6[:,1,None],q6[:,2,None],q7,q8,q9[:,None],q10[:,0,None],q10[:,1,None],q10[:,2,None],q11[:,0,None],q11[:,1,None],q11[:,2,None],q13,q15))
        if(self.compressData):
            tempQTC = self.removeStationary(tempQTC, self.labelToRemove, self.indsToRemove)
#             v = formatFeatures(tempQTC, 18)
        return tempQTC

    def configProcessing(self):
        if(self.compressData and set(self.featuresToCompress).issubset(self.featuresToUse)):
            self.labelToRemove = self.humanStaticLabels.index('stationary');
            self.indsToRemove = []
            for i in self.featuresToCompress:
                self.indsToRemove.append(self.featureSections[i])
            print 'Compressing data by removing ', self.humanStaticLabels[self.labelToRemove]
            print 'Applying compression to features:'
            for i in self.indsToRemove:
                print i
        else:
            print 'No compression applied'
            self.compressData = False

        featureInds = []
        print 'Features to use:'
        for h in self.featuresToUse:
            featureInds += self.featureSections[h]
            print '\t' + h

        return featureInds

    def chooseFeatures(self, handDataStruct):
        v = np.array(len(handDataStruct))
        for j in self.featuresToUse:
            vec = self.formatFeatures2(handDataStruct, self.featureSections[j], self.featureValues[j])
            if(vec[0] !=  -1):
                v = np.hstack((v,vec))
        return v
    
    def readData(self, root_data_dir, participant_index):
        #this function reads from data files and prepares a Y and an X
        onlyfiles = [f for f in listdir(root_data_dir) if isfile(join(root_data_dir, f))]
        dataLogList = [f for f in onlyfiles if 'data' in f]
        dataLogList.sort()
        labelsLogList = [f for f in onlyfiles if 'label' in f]
        labelsLogList.sort()
        self.dataLogList = []
        self.labelsLogList = []

        self.numJoints = 9
        data = dict()
        firstPass = True
        jointsList = []
        objectsList = []
        labelsList = []
        numFiles = len(dataLogList)

        print 'loading data from files'
        for k in range(len(dataLogList)):
            print 'data file: ' + str(join(root_data_dir, dataLogList[k]))
            print 'model file: ' + str(join(root_data_dir, labelsLogList[k]))
            print
            dataFile = open(join(root_data_dir, dataLogList[k]),'r')
            self.dataLogList.append(str(join(root_data_dir, dataLogList[k])))
            labelFile = open(join(root_data_dir, labelsLogList[k]),'r')
            self.labelsLogList.append(join(root_data_dir, labelsLogList[k]))

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
                dataFile = open(join(root_data_dir, dataLogList[k]),'r')
                labelFile = open(join(root_data_dir, labelsLogList[k]),'r')
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
                    for i in range(self.numJoints):
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

                    currIdx = (self.numJoints*4 -1)
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
                
        if(self.verbose):
            print 'data has length = ' + str(len(data)) + ' joints'
            strl =  'each joint has ' + str(len(data['head'])) + ' arrays of shape: \n'

            for i in data['head']:
                strl += '\t\t' + str(i.shape) + '\n'
            print strl

            strl = 'labelsList has length = ' + str(len(labelsList)) + ' with sizes: \n'
            for i in labelsList:
                strl += '\t\t' + str(len(i)) + '\n'
            print strl
        
        print
        print 'Unique labels in labels files:'
        print
        setList = []
        for x in labelsList:
            setList.append(list(set(x)))
        flattenedList = [val for sublist in setList for val in sublist]
        labels = list(set(flattenedList))
        labels.sort()
        if(self.verbose):
            print
            print 'Unique labels in labels files:'
            print
            for k in range(0,len(labels)):
                print '\t'+ str(k).ljust(3) + labels[k]
            print
            print 'Of these, config.ini specifies [' + ', '.join(self.ignoreLabels) + '] to be ignored'
        
        ignoreInds = []
        for k in self.ignoreLabels:
            ignoreInds.append(labels.index(k))

        dataStruct = []
        for arr in range(len(dataLogList)):
            for label in labels:
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
                                    startIdx = idxIndex

                            if(idxIndex+2 > len(idxs)):
                                if(idxs[idxIndex] - idxs[idxIndex-1] != 1):
                                    for joint in jointsList+objectsList[1:]:
                                        actionData = data[joint][arr][idxs[startIdx:idxIndex+1]]
                                        dataStruct.append([joint, label, arr, actionData.shape[0], actionData])
        self.joint = 0
        self.action = 1
        self.dataset = 2
        self.start = 3
        self.end = 4
        for a in range(len(dataLogList)):
            print 'Dataset ' + str(a) + ' : ' + dataLogList[a]
            for b in range(len(labels)):
                if('no' not in labels[b]):
                    y = len([i[1] for i in dataStruct if i[self.joint] == 'head' and i[self.dataset] == a and i[self.action] == labels[b]])
                    print '\t ' + str(labels[b]).ljust(35) + ' = ' + str(y).ljust(3) + ' repetitions'
            print
        
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

        d = self.distEuc(handPos, objectPos)
        if(self.verbose):
            print 'Mean of contact distances = ' + str(np.mean(d))
            print 'Median of contact distances = '  + str(np.median(d))
            print 'Max contact distance = ' + str(np.max(d))
            print 'Min contact distance = ' + str(np.min(d))
            print str(self.percentContactThreshold)+'% percentile contact distance = ' + str(np.percentile(d,self.percentContactThreshold))
        
        self.contactThreshold = np.percentile(d,self.percentContactThreshold)
        jointsList.sort()
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
            
        if(self.verbose):
            print 'Available joint pairs:'
            print
            for i in combinationList:
                print '\t' + str(i)
        
        self.humanStaticLabels = []
        self.humanStaticLabels.append('left of')
        self.humanStaticLabels.append('right of')
        self.humanStaticLabels.append('on top of')
        self.humanStaticLabels.append('underneath')
        self.humanStaticLabels.append('in front of')
        self.humanStaticLabels.append('behind')
        self.humanStaticLabels.append('stationary')
        
        self.actionsAllowed = []

        for ac in self.actionsAllowedList:
            temp = [g for g in labels if ac in g and "no" not in g]
            for h in range(len(temp)):
                self.actionsAllowed.append(temp[h])
        
        print
        for b in range(len(labels)):
            if('no' not in labels[b] and labels[b] in self.actionsAllowed):
                y = 0
                for a in range(len(dataLogList)):
                    y += len([i[1] for i in dataStruct if i[self.joint] == 'head' and i[self.dataset] == a and i[self.action] == labels[b]])
                if(self.verbose):
                    print str(labels[b]).ljust(35) + ' = ' + str(y).ljust(3) + ' repetitions'
        
        self.featureSections = dict()
        self.featureValues = dict()
        self.featureSections['QTC_Motion'] = range(0,2)
        self.featureValues['QTC_Motion'] = [-1,0,1]

        self.featureSections['QTC_Orientation'] = range(2,5)
        self.featureValues['QTC_Orientation'] = [-1,0,1]

        self.featureSections['directionVector'] = range(5,8)
        self.featureValues['directionVector'] = []

        self.featureSections['contact'] = [8]
        self.featureValues['contact'] = [0,1]

        self.featureSections['relativePositionLabel'] = [9]
        self.featureValues['relativePositionLabel'] = range(len(self.humanStaticLabels))

        self.featureSections['euc_distance'] = [10]
        self.featureValues['euc_distance'] = []

        self.featureSections['posL'] = range(11,14)
        self.featureValues['posL'] = []

        self.featureSections['posK'] = range(14,17)
        self.featureValues['posK'] = []

        self.featureSections['selfMovementLabelL'] = [17]
        self.featureValues['selfMovementLabelL']   = range(len(self.humanStaticLabels))

        self.featureSections['selfMovementLabelK'] = [18]
        self.featureValues['selfMovementLabelK']   = range(len(self.humanStaticLabels))
        
        featureInds = self.configProcessing()

        firstPass = True
        handDataStruct = []

        for currComb in combinationList:
            print
            print str(currComb[0]) +'-' +str(currComb[1])
            infoK = [i for i in dataStruct if i[self.joint] == currComb[0]]
            infoL = [i for i in dataStruct if i[self.joint] == currComb[1]]
            for i in range(len(infoK)):
                if(infoK[i][1] in self.actionsAllowed):
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

                if(condition):

                    Pk = infoK[i][4]
                    Pl = infoL[i][4]
                    tempQTC = self.extractFeatures(Pk, Pl)
                    handDataStruct.append([currComb[0], currComb[1], infoK[i][1], infoK[i][2], tempQTC.shape[0], tempQTC])

                    if(self.verbose):
                        print '[%s]' % ', '.join(map(str, handDataStruct[-1][:-1]))
        
        for n in range(len(handDataStruct)):     
            handDataStruct[n][5] = self.chooseFeatures(handDataStruct[n][5])

        for a in range(len(handDataStruct)):
            for r in self.actionsAllowedList:
                if r in handDataStruct[a][2]:
                    labelNum = self.actionsAllowedList.index(r)

            if(a == 0):
                self.Y = handDataStruct[a][5]
                self.L = labelNum
            else:
                self.Y = np.vstack((self.Y, handDataStruct[a][5]))
                self.L = np.vstack((self.L, labelNum))
        if(self.verbose):
            print self.Y.shape
            print self.L.shape
            
        self.textLabels = self.actionsAllowedList
        
    def prepareData(self, model='mrd', Ntr = 50, randSeed=0):
        a = 0
        for k in range(len(self.actionsAllowedList)):
            this_label = [i for i in range(self.L.shape[0]) if self.L[i] == k]
            this_Y = self.Y[this_label]
            this_L = self.L[this_label]
            
            print 'Number of samples for ', self.actionsAllowedList[k],' = ', len(this_L)
            numTr = int(Ntr*len(this_L)/100)
            numTs = len(this_L) - numTr
            
            print 'Number of training samples = ', numTr
            print 'Number of testing samples = ', numTs
            
            IdxTr = [ i for i in sorted(random.sample(xrange(len(this_L)),numTr))]
            IdxTs = list(set(range(len(this_L))) - set(IdxTr))
            
            print 'Training'
            print len(IdxTr), numTr
            print IdxTr
            print 'Testing'
            print len(IdxTs), numTs
            print IdxTs
            print 
            print
            
            if(a == 0):
                Yall = np.asarray([this_Y[i] for i in IdxTr])
                Lall = np.asarray([this_L[i] for i in IdxTr])
            else:
                Yall = np.vstack((Yall, np.asarray([this_Y[i] for i in IdxTr])))
                Lall = np.vstack((Lall, np.asarray([this_L[i] for i in IdxTr])))
            
            if(a == 0):
                YtestAll = np.asarray([this_Y[i] for i in IdxTs])
                LtestAll = np.asarray([this_L[i] for i in IdxTs])
            else:
                YtestAll = np.vstack((YtestAll, np.asarray([this_Y[i] for i in IdxTs])))
                LtestAll = np.vstack((LtestAll, np.asarray([this_L[i] for i in IdxTs])))
            a = 1
            
        self.Ytest = YtestAll
        self.Ltest = LtestAll
        self.Y = Yall
        self.L = Lall
        
        # Center data to zero mean and 1 std
        self.Ymean = self.Y.mean()
        self.Yn = self.Y - self.Ymean
        self.Ystd = self.Yn.std()
        self.Yn /= self.Ystd
        # Normalise test data similarly to training data
        self.Ytestn = self.Ytest - self.Ymean
        self.Ytestn /= self.Ystd

        # As above but for the labels
#         self.Lmean = self.L.mean()
#         self.Ln = self.L - self.Lmean
#         self.Lstd = self.Ln.std()
#         self.Ln /= self.Lstd
#         self.Ltestn = self.Ltest - self.Lmean
#         self.Ltestn /= self.Lstd
#         self.L

        if model == 'mrd':    
            self.X=None     
            self.Y = {'Y':self.Yn,'L':self.L}
            self.data_labels = self.L.copy()
        elif model == 'gp':
            self.X=self.Y.copy()
            self.Y = {'L':self.Ln.copy()+0.08*numpy.random.randn(self.Ln.shape[0],self.Ln.shape[1])}
            self.data_labels = None
        elif model == 'bgplvm':
            self.X=None     
            self.Y = {'Y':self.Yn}
            self.data_labels = self.L.copy()
        return Yall, Lall, YtestAll, LtestAll
    
    def testing(self, testInstance, verbose=True):
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
#             print "With " + str(vv.mean()) +" prob. error the new action is " + self.textLabels[int(self.SAMObject.model.bgplvms[1].Y[nn,:])]
            textStringOut=self.textLabels[int(self.SAMObject.model.bgplvms[1].Y[nn,:])]

        elif self.SAMObject.type == 'bgplvm':
#             print "With " + str(vv.mean()) +" prob. error the new action is " + self.textLabels[int(self.L[nn,:])]
            textStringOut=self.textLabels[int(self.L[nn,:])]
        
        if(verbose):
            if(vv.mean()<0.00012):            
                print "The action is " + textStringOut
            elif(vv.mean()>0.00012):
                print "I think the action is " + textStringOut + " but I am not sure"

        return [textStringOut, vv.mean()]

    def distEuc(self, a,b):
        temp = a-b
        temp = np.square(temp)
        temp = np.sum(temp,1)
        return np.sqrt(temp)

    def qtc_2D(self, k,l,q,thresh):
        
        d1 = self.distEuc(k[:-2],l[1:-1])
        d2 = self.distEuc(k[1:-1],l[1:-1])
        d3 = self.distEuc(k[2:],l[1:-1])
        
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

    def frenetFrame(self, arr):
        t_num = np.diff(arr,axis=0)
        t = (t_num/np.abs(t_num)).astype(int)

        b_num = np.cross(t[:-1],t[1:])
        b = b_num/np.abs(b_num)
        t = t[1:]

        n = np.cross(b,t)

        frameArr = np.concatenate((t,n,b),axis=1).T
        fArr = frameArr.reshape((3,3,-1),order = 'F')
        return fArr

    def qtc_3D(self, k, l, thresh, q3, q4, q5):
        fFrameK = self.frenetFrame(k)
        fFrameL = self.frenetFrame(l)
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
                
    def checkQTC(self, qtcArr):
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

    def uniqueQTC(self, currQTC):
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

    def filterMovement(self, ts, thresh):
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

    def removeStationary(self, inputArr, labelRemove, indsToCheck):
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

    def formatFeatures(self, inputArr, idx):
        vec = np.zeros((len(humanStaticLabels)+1))
        for i in range(len(inputArr)):
            vec[inputArr[i][idx]] += 1
        vec[-1] = len(inputArr)
        return vec

    def formatFeatures2(self, inputArr, idx, valsPossible):
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

    # curTestData = Yall[][None,:].tolist()
    def formatDataFunc(self, Ydata):
            yDataList = []
            for j in range(Ydata.shape[0]):
                yDataList.append(Ydata[j][None,:])
            return yDataList

    def sequenceConfig(self, verbose = False):
        self.data = dict()
        self.jointsList = []
        self.objectsList = []
        self.configProcessing()
        self.actionStore = []
        self.verbose = verbose

    def sequenceProcessing(self, dataMessage, mode='live'):
        classification = None
        t = dataMessage.replace('(','').replace(')','').split(' ')[2:]
        if(t > 40):
            del t[0:2]
            #extract data parts
            for i in range(self.numJoints):
                a = i*4
                if(t[a] == 'shoulderCenter'):
                    t[a] = 'chest'

                self.data[t[a]] = (np.array([float(t[a+1]), float(t[a+2]), float(t[a+3])]))
                if(t[a] not in self.jointsList):
                    self.jointsList.append(t[a])

            currIdx = (self.numJoints*4 -1)
            numObjs = (len(t) - currIdx)/5

            for i in range(numObjs):
                a = currIdx + 1 + (i*5)
                self.data[t[a]] = np.array([float(t[a+1]), float(t[a+2]), float(t[a+3])])
                if(t[a] not in self.objectsList):
                    self.objectsList.append(t[a])

            #check contact of either hand with either object
            #generate list of combinations of hands and objects to check for contact
            self.combinationList = []
            self.combinationKeys = []
            for i in self.objectsList[1:]:
                self.combinationList.append(['handLeft',i])
                self.combinationList.append(['handRight',i])
                
                self.combinationKeys.append(','.join(self.combinationList[-2]))
                self.combinationKeys.append(','.join(self.combinationList[-1]))

            Pk = None
            Pl = None
            for i in range(len(self.combinationList)):
                if(self.combinationKeys[i] not in self.data):
                    if(self.verbose):
                        print 'add item', self.combinationKeys[i]
                    self.data[self.combinationKeys[i]] = {'Pk':[None],'Pl':[None],'prevContact':False,'currContact':False,'d':[None]}
               
                if(Pk == None):
                    Pk = self.data[self.combinationList[i][0]].T
                    Pl = self.data[self.combinationList[i][1]].T
                else:
                    Pk = np.vstack((Pk,self.data[self.combinationList[i][0]].T))
                    Pl = np.vstack((Pl,self.data[self.combinationList[i][1]].T))

            d = self.distEuc(Pk,Pl)
            
            for i in range(len(self.combinationList)):
                if(d[i] < self.contactThreshold):
                    self.data[self.combinationKeys[i]]['currContact'] = True
                else:
                    self.data[self.combinationKeys[i]]['currContact'] = False
                
                if(self.data[self.combinationKeys[i]]['currContact']):
                    if(self.data[self.combinationKeys[i]]['prevContact']):
                        self.data[self.combinationKeys[i]]['Pk'].append(Pk[i])
                        self.data[self.combinationKeys[i]]['Pl'].append(Pl[i])
                        if(self.verbose):
                            print i,'Append data', self.combinationList[i]
                    else:
                        self.data[self.combinationKeys[i]]['actionOccuring'] = True
                        if(self.verbose):
                            print i,'Contact between', self.combinationList[i], 'Action started'
                else:
                    if(self.data[self.combinationKeys[i]]['prevContact']):
                        self.data[self.combinationKeys[i]]['actionOccuring']  = False
                        self.data[self.combinationKeys[i]]['actionLen'] = len(self.data[self.combinationKeys[i]]['Pk'])
                        
                        if(self.data[self.combinationKeys[i]]['actionLen'] > 10):
                            if(self.verbose):
                                print i,'Action stopped.', 'Len =', self.data[self.combinationKeys[i]]['actionLen']
                            #processing the action
                            tempQTC = self.extractFeatures(Pk, Pl)
                            tempQTC = self.chooseFeatures(tempQTC)
                            if(self.verbose):
                                print
                            [label, prob] = self.testing(tempQTC[None,:], False)
                            classification = label.split('_')[0] 
                            sentence = "You " + label.split('_')[0] + "ed the " + str(self.combinationList[i][1]) + " with your " + str(self.combinationList[i][0]).replace('hand','') + ' hand'
                            if(self.verbose):
                                print sentence
                            self.actionStore.append(sentence)
                        else:
                            if(self.verbose):
                                print i,'Action stopped.', 'Len =', self.data[self.combinationKeys[i]]['actionLen'], 'Action too short'
                            else:
                                'Action too short'
                            
                        self.data[self.combinationKeys[i]]['Pk'] = [None]
                        self.data[self.combinationKeys[i]]['Pl'] = [None]
                    else:
                        if(self.verbose):
                            print i,'x'

                self.data[self.combinationKeys[i]]['prevContact'] =  self.data[self.combinationKeys[i]]['currContact']
        if(self.verbose):
            print

        if(mode == 'testing'):
            return classification