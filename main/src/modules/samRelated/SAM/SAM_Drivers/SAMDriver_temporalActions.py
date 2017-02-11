#!/usr/bin/python

# """"""""""""""""""""""""""""""""""""""""""""""
# The University of Sheffield
# WYSIWYD Project
#
# SAMpy class for implementation of SAM module
#
# Created on 26 May 2015
#
# @authors: Uriel Martinez, Luke Boorman, Andreas Damianou
#
# """"""""""""""""""""""""""""""""""""""""""""""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from SAM.SAM_Core import SAM_utils as utils
from SAM.SAM_Core import SAMDriver
import readline
from SAM.SAM_Core import SAM_utils as utils
import yarp
import GPy

# """"""""""""""""
# Class developed for the implementation of the face recognition task in real-time mode.
# """"""""""""""""

class SAMDriver_temporalActions(SAMDriver):
    # """"""""""""""""
    # Initilization of the SAM class
    # Inputs:
    #    - isYarprunning: specifies if yarp is used (True) or not(False)
    #    - imgH, imgW: original image width and height
    #    - imgHNew imgWNew: width and height values to resize the image
    #
    # Outputs: None
    # """"""""""""""""

    def plotMove(self, fig, move, name):
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(move[0, :], move[2, :], move[1, :])
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        ax.set_zlim(np.min(move), np.max(move))
        ax.set_ylim(np.min(move), np.max(move))
        ax.set_xlim(np.min(move), np.max(move))
        ax.set_title(name)

    def __init__(self):
        SAMDriver.__init__(self)
        self.additionalParametersList = []

    def loadParameters(self, parser, trainName):
        print 'here'
        pass

    def saveParameters(self):
        SAMDriver.saveParameters(self)

    def readData(self, root_data_dir, participant_index, *args, **kw):
        funDict = dict()
        funDict['push'] = np.array([1, 1, 3])[:, None]
        funDict['pull'] = np.array([0, 0, -3])[:, None]
        funDict['lift'] = np.array([1, 3, 0])[:, None]
        funDict['drop'] = np.array([-2, -3, 0])[:, None]

        movementLen = 20
        sampling = 1

        moveDict = dict()

        for k, j in enumerate(funDict):
            moveDict[j] = np.hstack([funDict[j]] * movementLen) * np.vstack([np.arange(0, movementLen, sampling)] * 3)

        oneSequence = None

        self.labels = []
        labelsNum = []
        for k, j in enumerate(moveDict):
            if oneSequence is None:
                oneSequence = moveDict[j]
            else:
                oneSequence = np.hstack([oneSequence, oneSequence[:, -1][:, None] + moveDict[j]])
            self.labels = self.labels + [j] * moveDict[j].shape[1]
            labelsNum = labelsNum + [k] * moveDict[j].shape[1]
            fig = plt.figure()
            self.plotMove(fig, moveDict[j], j)

        labelsNum = np.asarray(labelsNum)[:, None]
        fig = plt.figure()
        self.plotMove(fig, oneSequence, 'all')
        self.Y1 = oneSequence.transpose()
        self.U1 = labelsNum

        plt.close('all')
        # create labels for the items which are to be trained on,
        # labelling the ones which are at the transition phase of the window as 'transition'
        # maybe this procedure will impart a certain sense of what a transition is to the model

    def processLiveData(self, dataList, thisModel):

        print 'process live data'
        return None

    def prepareData(self, model='mrd', Ntr=50, randSeed=0):

        self.textLabels = []

        for j in range(len(self.L)):
            if len(set(self.L[j])) > 1:
                self.textLabels.append('transition')
            else:
                self.textLabels.append(self.labels[j])
        print set(self.textLabels)
        # oneSequence has the data in the form of (vectorLen, numberOfTimeSteps)
        # labels is a list of labels
        # labelsNum is an array of labels in number form to be used as control inputs

        idxLists = []
        textLabelsList = list(set(self.textLabels))
        for n in textLabelsList:
            idxLists.append([j for j, k in enumerate(self.textLabels) if k == n])

        Xtr = None
        Ytr = None
        Utr = None
        Xts = None
        Yts = None
        Uts = None

        for j in idxLists:
            idx = np.random.permutation(j)
            trIdx = idx[0:round(Ntr * len(idx) / 100)]
            tsIdx = idx[round(Ntr * len(idx) / 100):]
            if Xtr is None:
                Xtr = self.X[trIdx]
                Xts = self.X[tsIdx]
                Ytr = self.Y[trIdx]
                Yts = self.Y[tsIdx]
                Utr = self.L[trIdx]
                Uts = self.L[tsIdx]
            else:
                Xtr = np.vstack([Xtr, self.X[trIdx]])
                Xts = np.vstack([Xts, self.X[tsIdx]])
                Ytr = np.vstack([Ytr, self.Y[trIdx]])
                Yts = np.vstack([Yts, self.Y[tsIdx]])
                Utr = np.vstack([Utr, self.L[trIdx]])
                Uts = np.vstack([Uts, self.L[tsIdx]])
        print Xtr.shape
        print Ytr.shape
        print Utr.shape
        return Xtr, Ytr, Utr, Xts, Yts, Uts

    def testTemporalPerformance(self, testModel, Xall, Yall, Lall, XtestAll, YtestAll, LtestAll, verbose):
        # Initial window to kick-off free simulation
        x_start = XtestAll[0, :][:, None].T

        # Free simulation
        ygp, varygp = utils.gp_narx(testModel[0].SAMObject.model, x_start, YtestAll.shape[0], LtestAll, self.windowSize)

        return 1000
