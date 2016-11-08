#!/usr/bin/env ipython
# """"""""""""""""""""""""""""""""""""""""""""""
# The University of Sheffield
# WYSIWYD Project
#
# A generic training class that is used to train all models.
# This class can train both single model implementations as well as multiple model implementations
#
# Created on 20 July 2016
#
# @author: Daniel Camilleri
#
# """"""""""""""""""""""""""""""""""""""""""""""
import warnings
import sys
import numpy
import numpy as np
from SAM.SAM_Core import SAMCore
from SAM.SAM_Core import SAMTesting
from SAM.SAM_Core.SAM_utils import initialiseModels
np.set_printoptions(threshold=numpy.nan)
warnings.simplefilter("ignore")

dataPath = sys.argv[1]
modelPath = sys.argv[2]
driverName = sys.argv[3]

mm = initialiseModels(sys.argv[1:4], sys.argv[4])
# mm[0].SAMObject.visualise()

if mm[0].calibrateUnknown or len(mm) > 1:
    SAMTesting.calibrateModelRecall(mm)

overallPerformance = 100000
if mm[0].model_mode != 'temporal':
    overallPerformance = mm[0].testPerformance(mm, mm[0].Yall, mm[0].Lall, mm[0].YtestAll, mm[0].LtestAll, True)
elif mm[0].model_mode == 'temporal':
    overallPerformance = mm[0].testTemporalPerformance(mm, mm[0].Xall, mm[0].Yall, mm[0].Lall,
                                                       mm[0].XtestAll, mm[0].YtestAll, mm[0].LtestAll, True)

numParts = len(mm[0].participantList)
for k in range(numParts):
    mm[k].paramsDict['overallPerformance'] = overallPerformance
    mm[k].paramsDict['ratioData'] = mm[0].ratioData
    mm[k].paramsDict['model_type'] = mm[k].model_type
    mm[k].paramsDict['model_mode'] = mm[0].model_mode
    mm[k].paramsDict['verbose'] = mm[0].verbose
    mm[k].paramsDict['Quser'] = mm[0].Quser
    mm[k].paramsDict['model_num_inducing'] = mm[0].model_num_inducing
    mm[k].paramsDict['model_num_iterations'] = mm[0].model_num_iterations
    mm[k].paramsDict['model_init_iterations'] = mm[0].model_init_iterations
    mm[k].paramsDict['kernelString'] = mm[0].kernelString
    mm[k].paramsDict['economy_save'] = mm[0].economy_save

    if mm[0].model_mode != 'temporal':
        mm[k].paramsDict['textLabels'] = mm[0].textLabels
        mm[k].paramsDict['modelQ'] = mm[0].SAMObject.Q
    elif mm[0].model_mode == 'temporal':
        mm[k].paramsDict['temporalModelWindowSize'] = mm[0].temporalModelWindowSize

    if mm[k].model_type == 'mrd' and mm[k].model_mode != 'temporal':
        print mm[k].Y['L'].shape
        print mm[k].Y['Y'].shape

    if k == 0:
        mm[0].paramsDict['listOfModels'] = mm[0].listOfModels
        mm[0].paramsDict['avgClassTime'] = mm[0].avgClassTime
        mm[0].paramsDict['optimiseRecall'] = mm[0].optimiseRecall
        if numParts > 1:
            mm[0].paramsDict['classifiers'] = mm[0].classifiers
            mm[0].paramsDict['classif_thresh'] = mm[0].classif_thresh
        else:
            if mm[0].X is None:
                mm[0].paramsDict['X'] = mm[0].X
            else:
                mm[0].paramsDict['X'] = mm[0].X.shape

            if mm[0].model_mode != 'temporal':
                mm[0].paramsDict['Y'] = mm[k].Y['Y'].shape

            if mm[0].varianceThreshold is not None:
                if mm[0].useMaxDistance:
                    mm[0].paramsDict['useMaxDistance'] = mm[0].useMaxDistance
                    mm[0].paramsDict['bestDistanceIDX'] = mm[0].bestDistanceIDX
                    mm[0].paramsDict['varianceThreshold'] = mm[0].varianceThreshold
                    mm[0].paramsDict['varianceDirection'] = mm[0].varianceDirection
                else:
                    mm[0].paramsDict['useMaxDistance'] = mm[0].useMaxDistance

    elif numParts > 1:
        # fname = mm[0].listOfModels[k-1]
        if mm[k].X is None:
            mm[k].paramsDict['X'] = mm[k].X
        else:
            mm[k].paramsDict['X'] = mm[k].X.shape
        mm[k].paramsDict['Y'] = mm[k].Y['Y'].shape
    # else:
    #     pass
        # fname = fnameProto

    print
    # save model with custom .pickle dictionary by iterating through all nested models
    print '-------------------'
    print 'Saving: ' + mm[k].fname
    mm[k].saveParameters()
    print 'Keys:'
    print mm[k].paramsDict.keys()
    SAMCore.save_pruned_model(mm[k].SAMObject, mm[k].fname, mm[0].economy_save, extraDict=mm[k].paramsDict)
