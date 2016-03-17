#""""""""""""""""""""""""""""""""""""""""""""""
#The University of Sheffield
#WYSIWYD Project
#
#The superclass of all Drivers. Every driver written for
#SAM must inherit from this class and extend it with any
#specific functionality.
#
#See also: SAMDriver_faces.py, SAMDriver_actions.py
#
#Created on 26 May 2015
#
#@authors: Andreas Damianou, Uriel Martinez, Luke Boorman
#
#""""""""""""""""""""""""""""""""""""""""""""""

from SAM.SAM_Core import SAMCore
import matplotlib.pyplot as plt
#import matplotlib as mp
import pylab as pb
import sys
#import pickle
import numpy
import os
try:
    import yarp
    isYarpRunningGlobal = True
except ImportError:
    isYarpRunningGlobal = False
import cv2
import GPy
import time
from scipy.spatial import distance
import operator
"""
try:
    from SAM import SAM
except ImportError:
    import SAM
"""
import time



#""""""""""""""""
#Class developed for the implementation of the face recognition task in real-time mode.
#""""""""""""""""

class SAMDriver:
    #""""""""""""""""
    #Initilization of the SAM class
    #Inputs:
    #    - isYarprunning: specifies if yarp is used (True) or not(False)
    #    - imgH, imgW: original image width and height
    #    - imgHNewm imgWNew: width and height values to resize the image
    #
    #Outputs: None
    #""""""""""""""""
    def __init__(self, isYarpRunning = False):
        if not isYarpRunningGlobal and isYarpRunning:
            isYarpRunning = False
            print 'Warning! yarp was not found in the system.'

        self.isYarpRunning = isYarpRunning
        self.SAMObject=SAMCore.LFM()        

        self.Y = None
        self.L = None
        self.X = None
        self.Ytest = None
        self.Ltest = None
        self.Ytestn = None
        self.Ltestn = None
        self.Ymean = None
        self.Ystd = None
        self.Yn = None
        self.Ln = None
        self.data_labels = None

        self.model_num_inducing = 0
        self.model_num_iterations = 0
        self.model_init_iterations = 0





    #""""""""""""""""
    #Methods to create the ports for reading images from iCub eyes
    #Inputs: None
    #Outputs: None
    #""""""""""""""""
    def createPorts(self):
        if self.isYarpRunning:
            self.imageDataInputPort = yarp.BufferedPortImageRgb()
            self.outputFacePrection = yarp.Port()
            self.speakStatusPort = yarp.RpcClient();
            self.speakStatusOutBottle = yarp.Bottle()
            self.speakStatusInBottle = yarp.Bottle()
            self.imageInputBottle = yarp.Bottle()

    #""""""""""""""""
    #Method to open the ports. It waits until the ports are connected
    #Inputs: None
    #Outputs: None
    #""""""""""""""""
    def openPorts(self):
        if self.isYarpRunning:
            print "open ports"
            self.imageDataInputPort.open("/sam/face/imageData:i");
            self.outputFacePrection.open("/sam/face/facePrediction:o")
            self.speakStatusPort.open("/sam/face/speakStatus:i")
            self.speakStatusOutBottle.addString("stat")

        #print "Waiting for connection with imageDataInputPort..."
#        while( not(yarp.Network.isConnected(self.inputImagePort,"/sam/imageData:i")) ):
#            print "Waiting for connection with imageDataInputPort..."
#            pass

    def closePorts(self):
        if self.isYarpRunning:
            print "open ports"
            self.actionDataInputPort.close();
            self.outputActionPrediction.close()
            self.speakStatusPort.close()
        #self.speakStatusOutBottle.addString("stat")
        #print "Waiting for connection with actionDataInputPort..."
        #while( not(yarp.Network.isConnected(self.inputActionPort,"/sam/actionData:i")) ):
        #    print "Waiting for connection with actionDataInputPort..."
        #    pass

    #""""""""""""""""
    #Method to train, store and load the learned model to be use for the face recognition task
    #Inputs:
    #    - modelNumInducing:
    #    - modelNumIterations:
    #    - modelInitIterations:
    #    - fname: file name to store/load the learned model
    #    - save_model: enable/disable to save the model
    #
    #Outputs: None
    #""""""""""""""""
    def training(self, modelNumInducing, modelNumIterations, modelInitIterations, fname, save_model, economy_save, keepIfPresent = True, kernelStr=None):
        self.model_num_inducing = modelNumInducing
        self.model_num_iterations = modelNumIterations
        self.model_init_iterations = modelInitIterations
    
        if not os.path.isfile(fname + '.pickle') or economy_save:
            if not os.path.isfile(fname + '.pickle'):
                print("Training for " +str(modelInitIterations) + "|" + str(modelNumIterations) + " iterations...")    
            try:
                ttest = self.Quser == None
            except :
                self.Quser = None

            if self.X is not None:
                Q = self.X.shape[1]
            elif self.Quser is not None:
                Q=self.Quser
            else:
                Q = 2

            if Q > 100:
                #one could parse and execute the string kernelStr for kernel instead of line below
                kernel = GPy.kern.RBF(Q, ARD=False) + GPy.kern.Bias(Q) + GPy.kern.White(Q)
                self.SAMObject.kernelString = kernelStr
            else:
                kernel = None
                self.SAMObject.kernelString = ''
            # Simulate the function of storing a collection of events
            self.SAMObject.store(observed=self.Y, inputs=self.X, Q=Q, kernel=kernel, num_inducing=self.model_num_inducing)
            
            # If data are associated with labels (e.g. face identities), associate them with the event collection
            if self.data_labels is not None:
                self.SAMObject.add_labels(self.data_labels)
            
            if economy_save and os.path.isfile(fname + '.pickle') and keepIfPresent:
                try:
                    print("Try loading economy size SAMObject: " + fname)
                    # Load the model from the economy storage
                    SAMCore.load_pruned_model(fname, economy_save, self.SAMObject.model)
                except ValueError:
                    print("Loading " + fname + " failed.\nParameters not valid. Training new model")
                    self.SAMObject.learn(optimizer='scg',max_iters=self.model_num_iterations, init_iters=self.model_init_iterations, verbose=True)
                    if save_model:
                        print("Saving SAMObject: " + fname)
                        SAMCore.save_pruned_model(self.SAMObject, fname, economy_save)
            elif not os.path.isfile(fname + '.pickle') or not keepIfPresent: 
                # Simulate the function of learning from stored memories, e.g. while sleeping (consolidation).
                self.SAMObject.learn(optimizer='scg',max_iters=self.model_num_iterations, init_iters=self.model_init_iterations, verbose=True)
                if save_model:
                    print("Saving SAMObject: " + fname)
                    SAMCore.save_pruned_model(self.SAMObject, fname, economy_save)
        else:
            print("Loading SAMOBject: " + fname)
            self.SAMObject = SAMCore.load_pruned_model(fname)

    #def load(self, fname,save_model, economy_save):


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
        self.Lmean = self.L.mean()
        self.Ln = self.L - self.Lmean
        self.Lstd = self.Ln.std()
        self.Ln /= self.Lstd
        self.Ltestn = self.Ltest - self.Lmean
        self.Ltestn /= self.Lstd

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

    def testing(self, testInstance, choice, visualiseInfo=None):
        raise NotImplementedError("this needs to be implemented to use the model class")

    def readData(self, root_data_dir, participant_index, *args, **kw):
        raise NotImplementedError("this needs to be implemented to use the model class")
