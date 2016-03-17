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
import datetime
import yarp
import copy
from itertools import combinations 
from ConfigParser import SafeConfigParser
from scipy.spatial import distance
from numpy.linalg import inv
import math
import ipyparallel as ipp


class staticPose(SAMDriver):
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
    
    def readData(self, root_data_dir, participant_index, Y, L, labels):
        self.Y = Y
        self.L = L
        self.textLabels = labels
        
    def prepareData(self, model='mrd', Ntr = 50, randSeed=0):    

        Nts=self.Y.shape[0]-Ntr
        numpy.random.seed(randSeed)
        perm = numpy.random.permutation(self.Y.shape[0])
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
        self.L

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
