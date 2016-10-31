# """"""""""""""""""""""""""""""""""""""""""""""
# The University of Sheffield
# WYSIWYD Project
#
# The superclass of all Drivers. Every driver written for
# SAM must inherit from this class and extend it with any
# specific functionality.
#
# See also: SAMDriver_faces.py, SAMDriver_actions.py
#
# Created on 26 May 2015
#
# @authors: Andreas Damianou, Uriel Martinez, Luke Boorman, Daniel Camilleri
#
# """"""""""""""""""""""""""""""""""""""""""""""

from SAM.SAM_Core import SAMCore
from SAM.SAM_Core import SAMTesting
import GPy
import numpy
import os


# """"""""""""""""
# Class developed for the implementation of the face recognition task in real-time mode.
# """"""""""""""""

class SAMDriver:
    # """"""""""""""""
    # Initilization of the SAM class
    # Inputs:
    #    - isYarprunning: specifies if yarp is used (True) or not(False)
    #    - imgH, imgW: original image width and height
    #    - imgHNewm imumber of images:418gWNew: width and height values to resize the image
    #
    # Outputs: None
    # """"""""""""""""
    def __init__(self):
        self.SAMObject = SAMCore.LFM()

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
        self.segTrainConf = None
        self.segTrainPerc = None
        self.segTestConf = None
        self.segTestPerc = None
        self.data_labels = None
        self.paramsDict = dict()
        self.verbose = False
        self.model_mode = False
        self.Quser = None
        self.listOfModels = []
        self.model_type = None
        self.modelLabel = None
        self.textLabels = None
        self.participantList = None
        self.varianceThreshold = None
        self.fname = None
        self.optimiseRecall = True
        self.modelLoaded = False

        self.Yall = None
        self.Lall = None
        self.YtestAll = None
        self.LtestAll = None

        self.classifiers = None
        self.classif_thresh = None

        self.model_num_inducing = 0
        self.model_num_iterations = 0
        self.model_init_iterations = 0

        self.additionalParametersList = []

    # """"""""""""""""
    # Method to load parameters from file loaded in parser from within section trainName
    # and store these parameters in self.paramsDict
    # Inputs:
    #    -parser: SafeConfigParser with pre-read config file
    #    -trainName: Section from which parameters are to be read
    # Outputs: None
    # """"""""""""""""
    def loadParameters(self, parser, trainName):
        raise NotImplementedError("this needs to be implemented to use the model class")

    # """"""""""""""""
    # Method to store variables generated during training which are required during interaction in self.paramsDict
    # Inputs:  None
    # Outputs: None
    # """"""""""""""""
    def saveParameters(self):
        for j in self.additionalParametersList:
            commandString = 'self.paramsDict[\'' + j + '\'] = self.' + j
            try:
                print commandString
                exec commandString
            except:
                pass

    # """"""""""""""""
    # Method for testing the whole dataset for overall performance
    # Inputs:  Number of inputs (variable)
    # Outputs: Must return a square confusion matrix.
    # This matrix is the measure of performance that the bayesian optimiser uses
    # """"""""""""""""
    def testPerformance(self, testModel, Yall, Lall, YtestAll, LtestAll, verbose):

        yTesting = SAMTesting.formatDataFunc(Yall)
        [self.segTrainConf, self.segTrainPerc, _] = SAMTesting.testSegments(testModel, yTesting, Lall, verbose,
                                                                            label='Training')

        yTesting = SAMTesting.formatDataFunc(YtestAll)
        [self.segTestConf, self.segTestPerc, _] = SAMTesting.testSegments(testModel, yTesting, LtestAll, verbose,
                                                                          label='Testing')

        return self.segTestConf

    # """"""""""""""""
    # Method to train, store and load the learned model
    # Inputs:
    #    - modelNumInducing:
    #    - modelNumIterations:
    #    - modelInitIterations:
    #    - fname: file name to store/load the learned model
    #    - save_model: enable/disable to save the model
    #
    # Outputs: None
    # """"""""""""""""
    def training(self, modelNumInducing, modelNumIterations, modelInitIterations, fname, save_model, economy_save,
                 keepIfPresent=True, kernelStr=None):
        self.model_num_inducing = modelNumInducing
        self.model_num_iterations = modelNumIterations
        self.model_init_iterations = modelInitIterations
    
        if not os.path.isfile(fname + '.pickle') or economy_save:
            if not os.path.isfile(fname + '.pickle'):
                print("Training for " + str(modelInitIterations) + "|" + str(modelNumIterations) + " iterations...")
            try:
                self.Quser is None
            except:
                self.Quser = None

            if self.X is not None:
                Q = self.X.shape[1]
            elif self.Quser is not None:
                Q = self.Quser
            else:
                Q = 2

            kernel = None
            if Q > 100:
                if kernelStr is not None:
                    stringKernel = 'kernel = ' + kernelStr
                else:
                    stringKernel = 'kernel = GPy.kern.RBF(Q, ARD=False) + GPy.kern.Bias(Q) + GPy.kern.White(Q)'
                exec stringKernel
                print 'stringKernel: ', stringKernel
                self.SAMObject.kernelString = kernelStr
            else:
                self.SAMObject.kernelString = ''
            # Simulate the function of storing a collection of events
            if self.model_mode != 'temporal':
                self.SAMObject.store(observed=self.Y, inputs=self.X, Q=Q, kernel=kernel,
                                     num_inducing=self.model_num_inducing)
            elif self.model_mode == 'temporal':
                self.SAMObject.model = GPy.models.SparseGPRegression(numpy.hstack((self.X, self.L)), self.Y,
                                                                     num_inducing=self.model_num_inducing)

            # If data are associated with labels (e.g. face identities),
            # associate them with the event collection
            if self.data_labels is not None:
                self.SAMObject.add_labels(self.data_labels)
            
            if economy_save and os.path.isfile(fname + '.pickle') and keepIfPresent:
                try:
                    print("Try loading economy size SAMObject: " + fname)
                    # Load the model from the economy storage
                    SAMCore.load_pruned_model(fname, economy_save, self.SAMObject.model)
                except ValueError:
                    print("Loading " + fname + " failed.\nParameters not valid. Training new model")
                    if self.model_mode != 'temporal':
                        self.SAMObject.learn(optimizer='bfgs', max_iters=self.model_num_iterations,
                                             init_iters=self.model_init_iterations, verbose=True)
                    elif self.model_mode == 'temporal':
                        self.SAMObject.model.optimize('bfgs', max_iters=self.model_num_iterations, messages=True)
                        self.SAMObject.type = 'mrd'
                        self.SAMObject.Q = None
                        self.SAMObject.N = None
                        self.SAMObject.namesList = None
                        self.SAMObject.kernelString = None
                    if save_model:
                        print("Saving SAMObject: " + fname)
                        SAMCore.save_pruned_model(self.SAMObject, fname, economy_save)
            elif not os.path.isfile(fname + '.pickle') or not keepIfPresent: 
                # Simulate the function of learning from stored memories, e.g. while sleeping (consolidation).
                if self.model_mode != 'temporal':
                    self.SAMObject.learn(optimizer='bfgs', max_iters=self.model_num_iterations,
                                         init_iters=self.model_init_iterations, verbose=True)
                elif self.model_mode == 'temporal':
                    self.SAMObject.model.optimize('bfgs', max_iters=self.model_num_iterations, messages=True)
                    self.SAMObject.type = 'mrd'
                    self.SAMObject.__num_views = None
                    self.SAMObject.Q = None
                    self.SAMObject.N = None
                    self.SAMObject.namesList = None
                    self.SAMObject.kernelString = None
                if save_model:
                    print("Saving SAMObject: " + fname)
                    SAMCore.save_pruned_model(self.SAMObject, fname, economy_save)
        else:
            print("Loading SAMObject: " + fname)
            self.SAMObject = SAMCore.load_pruned_model(fname)

    # def load(self, fname,save_model, economy_save):

    # """"""""""""""""
    # Method for randomly splitting data and preparing Y dictionary
    # Inputs:
    #    - model: mrd / bgplvm
    #    - Ntr: Number of training samples
    #    - randSeed: random seed
    #
    # Outputs: None
    # """"""""""""""""
    def prepareData(self, model='mrd', Ntr=50, randSeed=0, normalise=True):
        # currently does not take equal number of samples from different classes

        Nts = self.Y.shape[0] - Ntr
        numpy.random.seed(randSeed)
        perm = numpy.random.permutation(self.Y.shape[0])
        indTs = perm[0:Nts]
        indTr = perm[Nts:Nts + Ntr]
        indTs.sort()
        indTr.sort()
        YtestAll = self.Y[indTs].copy()
        self.Ytest = self.Y[indTs]
        LtestAll = self.L[indTs].copy()
        self.Ltest = self.L[indTs]
        Yall = self.Y[indTr].copy()
        self.Y = self.Y[indTr]
        Lall = self.L[indTr].copy()
        self.L = self.L[indTr]

        if normalise:
            print 'Normalising data'
            # Center data to zero mean and 1 std
            self.Ymean = self.Y.mean()
            self.Yn = self.Y - self.Ymean
            self.Ystd = self.Yn.std()
            self.Yn /= self.Ystd
            # Normalise test data similarly to training data
            self.Ytestn = self.Ytest - self.Ymean
            self.Ytestn /= self.Ystd

            # As above but for the labels
            # self.Lmean = self.L.mean()
            # self.Ln = self.L - self.Lmean
            # self.Lstd = self.Ln.std()
            # self.Ln /= self.Lstd
            # self.Ltestn = self.Ltest - self.Lmean
            # self.Ltestn /= self.Lstd

            if model == 'mrd':
                self.X = None
                self.Y = {'Y': self.Yn, 'L': self.L}
                self.data_labels = self.L.copy()
            elif model == 'gp':
                self.X = self.Y.copy()
                self.Y = {'L': self.Ln.copy() + 0.08 * numpy.random.randn(self.Ln.shape[0], self.Ln.shape[1])}
                self.data_labels = None
            elif model == 'bgplvm':
                self.X = None
                self.Y = {'Y': self.Yn}
                self.data_labels = self.L.copy()
        else:
            print 'Not normalising data'
            if model == 'mrd':
                self.X = None
                self.Y = {'Y': self.Y, 'L': self.L}
                self.data_labels = self.L.copy()
            elif model == 'gp':
                self.X = self.Y.copy()
                self.Y = {'L': self.Ln.copy() + 0.08 * numpy.random.randn(self.Ln.shape[0], self.Ln.shape[1])}
                self.data_labels = None
            elif model == 'bgplvm':
                self.X = None
                self.Y = {'Y': self.Y}
                self.data_labels = self.L.copy()

        return Yall, Lall, YtestAll, LtestAll

    # def testing(self, testInstance, choice, visualiseInfo=None):
    #     raise NotImplementedError("this needs to be implemented to use the model class")

    # """"""""""""""""
    # Method which accepts a data directory, reads all the data in and outputs:
    # self.Y which is a numpy array with n instances of m length feature vectors
    # self.L which is a list of text Labels of length n
    # Inputs:
    #    - root_data_dir: data directory
    #    - participant_index: List of subfolders to consider. Can be left as []
    #
    # Outputs: self.Y, self.L
        # """"""""""""""""
    def readData(self, root_data_dir, participant_index, *args, **kw):
        raise NotImplementedError("this needs to be implemented to use the model class")

    # """"""""""""""""
    # Method which receives a list of data frames and outputs a classification if available
    # or 'no_classification' if it is not
    # Inputs:
    #    - dataList: list of dataFrames collected. Length of list is variable
    #    - thisModel: list of models required for testing
    #
    # Outputs: None
    # """"""""""""""""
    def processLiveData(self, dataList, thisModel, verbose=False):
        raise NotImplementedError("this needs to be implemented to use the model class")

    # """"""""""""""""
    # Method which receives an instance generated from a model via a label and formats the received vector
    # by reshaping it or adding crucial data with it for it to be deciphered at the receiving end
    # Inputs:
    #    - instance: a vector of generated data
    #
    # Outputs:
    #    returns a yarp Bottle or yarp image that is then transmitted
    #    by interactionSamModel from where this function is called from
    # """"""""""""""""
    def formatGeneratedData(self, instance):
        raise NotImplementedError("this needs to be implemented to use the model class")
