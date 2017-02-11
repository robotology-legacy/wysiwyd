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
import readline
import yarp
from SAM.SAM_Core import SAM_utils as utils
import pylab as pb
from SAM.SAM_Core import SAMDriver


# """"""""""""""""
# Class developed for the implementation of the face recognition task in real-time mode.
# """"""""""""""""

class SAMDriver_temporal(SAMDriver):
    # """"""""""""""""
    # Initilization of the SAM class
    # Inputs:
    #    - isYarprunning: specifies if yarp is used (True) or not(False)
    #    - imgH, imgW: original image width and height
    #    - imgHNew imgWNew: width and height values to resize the image
    #
    # Outputs: None
    # """"""""""""""""
    def __init__(self):
        SAMDriver.__init__(self)
        self.additionalParametersList = []

    def loadParameters(self, parser, trainName):
        print 'here'
        pass

    def saveParameters(self):
        SAMDriver.saveParameters(self)

    def readData(self, root_data_dir, participant_index, *args, **kw):
        N1 = 140
        Ntr = np.ceil(N1 / 2)
        X1 = np.random.rand(N1, 1) * 8 * np.pi
        X1.sort(0)
        self.Y1 = np.sin(X1) + np.random.randn(*X1.shape) * 0.02
        self.U1 = self.Y1 ** 2 - 2 * self.Y1 + 5 + np.random.randn(*self.Y1.shape) * 0.005

    def processLiveData(self, dataList, thisModel):

        print 'process live data'
        return None

    def prepareData(self, model='mrd', Ntr=50, randSeed=0):
        Xall = self.X[0:Ntr, :]
        XtestAll = self.X[Ntr:, :]
        Yall = self.Y[0:Ntr, :]
        YtestAll = self.Y[Ntr:, :]
        Lall = self.L[0:Ntr, :]
        LtestAll = self.L[Ntr:, :]

        return Xall, Yall, Lall, XtestAll, YtestAll, LtestAll

    def testTemporalPerformance(self, testModel, Xall, Yall, Lall, XtestAll, YtestAll, LtestAll, verbose):
        # Initial window to kick-off free simulation
        x_start = XtestAll[0, :][:, None].T

        # Free simulation
        ygp, varygp = utils.gp_narx(testModel[0].SAMObject.model, x_start, YtestAll.shape[0], LtestAll, self.windowSize)
        pb.figure()
        pb.plot(YtestAll, 'x-')
        pb.plot(ygp, 'ro-')
        pb.legend(('True', 'Pred'))
        pb.title('NARX-with-exogenous')

        err = np.sum((YtestAll - ygp)**2)

        return err
