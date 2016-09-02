#!/usr/bin/env ipython
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
# """"""""""""""""""""""""""""""""""""""""""""""
# The University of Sheffield
# WYSIWYD Project
#
# A class that implements Bayesian Optimisation of SAM model parameters
#
# Created on 20 July 2016
#
# @author: Daniel Camilleri
#
# """"""""""""""""""""""""""""""""""""""""""""""

=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
#TODO:
#1 Save log of iterations and results
#  for the optimisation process and include routine
#  to load these if they are available in the case
#  that 200 iterations are not enough to find a satisfactory model
#2 move initial number of iterations and max number of iterations to the config file
#3 interface with yarp in order to gracefully terminate training via samSupervisor
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
# """"""""""""""""""""""""""""""""""""""""""""""
# The University of Sheffield
# WYSIWYD Project
#
# A class that implements Bayesian Optimisation of SAM model parameters
#
# Created on 20 July 2016
#
# @author: Daniel Camilleri
#
# """"""""""""""""""""""""""""""""""""""""""""""

<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
import numpy as np
import time
import os
import subprocess
import pickle
import sys
import glob
import shutil
import copy
from ConfigParser import SafeConfigParser

optNotFound = False
try:
    import GPyOpt
except:
    print 'GPyOpt not found'
    optNotFound = True
    pass


class modelOptClass(object):
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
    def __init__(self, fName, dataDir, modelDir, driverName, mode, baseName, persistence, windowed, verbose):
=======
    def __init__(self, fName, dataDir, modelDir, driverName, mode, persistence, windowed, verbose):
>>>>>>> New version of samSupervisor with rpc interface and README
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
    def __init__(self, fName, dataDir, modelDir, driverName, mode, persistence, windowed, verbose):
=======
    def __init__(self, fName, dataDir, modelDir, driverName, mode, baseName, persistence, windowed, verbose):
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
    def __init__(self, fName, dataDir, modelDir, driverName, mode, baseName, persistence, windowed, verbose):
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
        # check package is present
        try:
            import GPyOpt
            self.fName = fName
            self.dataDir = dataDir
            self.modelDir = modelDir
            self.driverName = driverName
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
            self.baseName = baseName
=======
>>>>>>> New version of samSupervisor with rpc interface and README
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
=======
            self.baseName = baseName
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
            self.baseName = baseName
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
            self.persistence = persistence
            self.verbose = verbose
            self.devnull = None
            self.windowed = windowed
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
            if not self.windowed:
                self.devnull = open('/dev/null', 'w')
            self.numEvals = 0
            self.penalty = 10000000000000000000
            self.mode = mode
            self.parser = None
            self.sectionBackup = None
            self.sectionOpt = None
            self.modelPresent = False
            self.bestOptions = None
            self.domain = None
            self.bestError = None
            self.currIterSettings = None
            self.acquisitionFunction = None
            self.trainProcess = None
            self.currFiles = None
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
            if (not self.windowed):
                self.devnull = open('/dev/null', 'w')
            self.numEvals = 0
            self.mode = mode
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
            if not self.windowed:
                self.devnull = open('/dev/null', 'w')
            self.numEvals = 0
            self.penalty = 10000000000000000000
            self.mode = mode
            self.parser = None
            self.sectionBackup = None
            self.sectionOpt = None
            self.modelPresent = False
            self.bestOptions = None
            self.domain = None
            self.bestError = None
            self.currIterSettings = None
            self.acquisitionFunction = None
            self.trainProcess = None
            self.currFiles = None
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
            self.configured = self.configOptimisation()
            print self.configured[1]
        except:
            msg = 'Cannot find GPyOpt package. Make sure it is installed and added to PYTHONPATH'
            print msg
            self.configured = [False, msg]

    def configOptimisation(self):
        self.parser = SafeConfigParser()
        self.parser.optionxform = str
        try:
            # check config file exists
            found = self.parser.read(self.dataDir + "/config.ini")
            if found:
                # open and check if Optimisation section is present in config.ini
                if (self.parser.has_section('Optimisation') == True and self.parser.has_section(
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
                        self.baseName) == True):
                    # create backup of current self.driverName section
                    self.sectionBackup = dict(self.parser.items(self.baseName))
=======
                        self.driverName) == True):
                    # create backup of current self.driverName section
                    self.sectionBackup = dict(self.parser.items(self.driverName))
>>>>>>> New version of samSupervisor with rpc interface and README
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
                        self.driverName) == True):
                    # create backup of current self.driverName section
                    self.sectionBackup = dict(self.parser.items(self.driverName))
=======
                        self.baseName) == True):
                    # create backup of current self.driverName section
                    self.sectionBackup = dict(self.parser.items(self.baseName))
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
                        self.baseName) == True):
                    # create backup of current self.driverName section
                    self.sectionBackup = dict(self.parser.items(self.baseName))
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs

                    # load Optimisation section
                    self.sectionOpt = dict(self.parser.items('Optimisation'))
                    if len(self.sectionOpt) == 1 and 'acquisitionFunction' in self.sectionOpt:
                        return [False,
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
                                'config.ini found, Optimisation and ' + self.baseName +
                                ' sections found but Optimisation section does not contain parameters to optimise']
                    else:
                        # create backup if model in modelDir exists
                        self.modelPresent = self.copyModel('backup', 'normal')

                        # load performance of current model if one is available and track performance
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
                                'config.ini found, Optimisation and ' + self.driverName + ' sections found but Optimisation section does not contain parameters to optimise']
                    else:
                        # create backup if model in modelDir exists
                        self.modelPresent = self.copyModel('backup','normal')

                        # load performance of current model if one is available and set self.bestError to track best model perfomance
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                                'config.ini found, Optimisation and ' + self.baseName +
                                ' sections found but Optimisation section does not contain parameters to optimise']
                    else:
                        # create backup if model in modelDir exists
                        self.modelPresent = self.copyModel('backup', 'normal')

                        # load performance of current model if one is available and track performance
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                        self.bestError = 0
                        try:
                            if len(self.currFiles) > 0:
                                for j in self.currFiles:
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
                                    if '.pickle' in j and '__L' not in j:
                                        modelPickle = pickle.load(open(j, 'rb'))
                                        testConf = modelPickle['overallPerformance']
                                        np.fill_diagonal(testConf, 0)
                                        self.bestError += np.sum(testConf)
                                # after combining all errors into one value copy and rename files to best
                                # which contains the best performing model so far
                                # best model will only be present if correct computation of bestError occurs
                                self.copyModel('best', 'normal')
                                self.bestOptions = copy.deepcopy(self.parser.items(self.baseName))
                            else:
                                print 'No model present'
                                self.bestError = self.penalty
                        except:
                            print 'testConf key not present in .pickle file'
                            self.bestError = self.penalty
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
                                    if '.pickle' in j:
                                        modelPickle = pickle.load(open(j, 'rb'))
                                        testConf = modelPickle['testConf']
                                        np.fill_diagonal(testConf, 0)
                                        self.bestError += np.sum(testConf)
                                # after combining all errors into one value copy and rename files to best which contains the best performing model so far
                                # best model will only be present if correct computation of bestError occurs
                                self.copyModel('best','normal')
                                self.bestOptions = copy.deepcopy(self.parser.items(self.driverName))
                            else:
                                print 'No model present'
                                self.bestError = np.inf
                        except:
                            print 'testConf key not present in .pickle file'
                            self.bestError = np.inf
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                                    if '.pickle' in j and '__L' not in j:
                                        modelPickle = pickle.load(open(j, 'rb'))
                                        testConf = modelPickle['overallPerformance']
                                        np.fill_diagonal(testConf, 0)
                                        self.bestError += np.sum(testConf)
                                # after combining all errors into one value copy and rename files to best
                                # which contains the best performing model so far
                                # best model will only be present if correct computation of bestError occurs
                                self.copyModel('best', 'normal')
                                self.bestOptions = copy.deepcopy(self.parser.items(self.baseName))
                            else:
                                print 'No model present'
                                self.bestError = self.penalty
                        except:
                            print 'testConf key not present in .pickle file'
                            self.bestError = self.penalty
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs

                        # iterate over keys of sectionOpt to create domain of optimisation problem
                        self.acquisitionFunction = 'MPI'
                        self.domain = []  # list of dictionaries
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
                        # armedBanditsMode = True
                        for i, v in self.sectionOpt.iteritems():
                            if i == 'acquisitionFunction':
                                # possible acquisition functions
                                # 'MPI' : maximum probability of improvement
                                # 'EI'  : Expected Improvement
                                # 'UCB' : Upper class bound
                                if v == 'MPI' or v == 'EI' or v == 'UCB':
                                    self.acquisitionFunction = v
                            elif i in self.sectionBackup:
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
                        armedBanditsMode = True
                        for i, v in self.sectionOpt.iteritems():
                            if (i == 'acquisitionFunction'):
                                # possible acquisition functions
                                # 'MPI' : maximum porobability of improvement
                                # 'EI'  : Expected Improvement
                                # 'UCB' : Upper class bound)
                                if (v == 'MPI' or v == 'EI' or v == 'UCB'):
                                    self.acquisitionFunction = v
                            elif (i in self.sectionBackup):
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                        # armedBanditsMode = True
                        for i, v in self.sectionOpt.iteritems():
                            if i == 'acquisitionFunction':
                                # possible acquisition functions
                                # 'MPI' : maximum probability of improvement
                                # 'EI'  : Expected Improvement
                                # 'UCB' : Upper class bound
                                if v == 'MPI' or v == 'EI' or v == 'UCB':
                                    self.acquisitionFunction = v
                            elif i in self.sectionBackup:
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                                opts = v.partition('[')[-1].rpartition(']')[0]
                                sects = opts.split(':')
                                tempDict = dict()
                                if sects[0] == 'discreteInt':
                                    lims = sects[1].split(',')
                                    arr = np.arange(int(lims[0]), int(lims[2]), int(lims[1]))
                                    arr = np.hstack((arr, int(lims[2])))
                                    tempDict['name'] = i
                                    tempDict['type'] = 'discrete'
                                    tempDict['domain'] = arr
                                    tempDict['description'] = sects[0]
                                    self.domain.append(tempDict)
                                elif sects[0] == 'discreteFloat':
                                    lims = sects[1].split(',')
                                    arr = np.arange(float(lims[0]), float(lims[2]), float(lims[1]))
                                    arr = np.hstack((arr, float(lims[2])))
                                    tempDict['name'] = i
                                    tempDict['type'] = 'discrete'
                                    tempDict['domain'] = arr
                                    tempDict['description'] = sects[0]
                                    self.domain.append(tempDict)
                                elif sects[0] == 'continuous':
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
                                    # armedBanditsMode = False
=======
                                    armedBanditsMode = False
>>>>>>> New version of samSupervisor with rpc interface and README
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
                                    armedBanditsMode = False
=======
                                    # armedBanditsMode = False
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
                                    # armedBanditsMode = False
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                                    lims = sects[1].split(',')
                                    tempDict['name'] = i
                                    tempDict['type'] = 'continuous'
                                    tempDict['domain'] = (float(lims[0]), float(lims[1]))
                                    tempDict['description'] = sects[0]
                                    self.domain.append(tempDict)
                                elif sects[0] == 'bool':
                                    tempDict['name'] = i
                                    tempDict['type'] = 'discrete'
                                    tempDict['domain'] = np.array((0, 1))
                                    tempDict['description'] = sects[0]
                                    self.domain.append(tempDict)
                                elif sects[0] == 'combination':
                                    splitList = sects[1].split(',')
                                    for b in splitList:
                                        tempDict = dict()
                                        tempDict['name'] = b
                                        tempDict['type'] = 'discrete'
                                        tempDict['domain'] = np.array((0, 1))
                                        tempDict['description'] = sects[0]
                                        tempDict['groupName'] = i
                                        self.domain.append(tempDict)
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
                                elif sects[0] == 'list':
                                    splitList = sects[1].split(',')
                                    tempDict = dict()
                                    tempDict['name'] = i
                                    tempDict['type'] = 'discrete'
                                    tempDict['domain'] = np.arange(0, len(splitList), 1)
                                    tempDict['description'] = sects[0]
                                    tempDict['groupName'] = i
                                    tempDict['values'] = splitList
                                    self.domain.append(tempDict)
                            else:
                                print 'ignoring ', i

                        # if armedBanditsMode :
                        #     for j in self.domain:
                        #         j['type'] = 'bandit'
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
                            else:
                                print 'ignoring ', i

                                #                         if armedBanditsMode :
                                #                             for j in self.domain:
                                #                                 j['type'] = 'bandit'
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                                elif sects[0] == 'list':
                                    splitList = sects[1].split(',')
                                    tempDict = dict()
                                    tempDict['name'] = i
                                    tempDict['type'] = 'discrete'
                                    tempDict['domain'] = np.arange(0, len(splitList), 1)
                                    tempDict['description'] = sects[0]
                                    tempDict['groupName'] = i
                                    tempDict['values'] = splitList
                                    self.domain.append(tempDict)
                            else:
                                print 'ignoring ', i

                        # if armedBanditsMode :
                        #     for j in self.domain:
                        #         j['type'] = 'bandit'
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs

                        for j in self.domain:
                            print j
                            print

                        return [True, 'Optimisation configuration ready']
                else:
                    return [False,
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
                            'config.ini found at ' + self.dataDir + ' but does not contain Optimisation and/or ' +
                            self.driverName + ' section']
=======
                            'config.ini found at ' + self.dataDir + ' but does not contain Optimisation and/or ' + self.driverName + ' section']
>>>>>>> New version of samSupervisor with rpc interface and README
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
                            'config.ini found at ' + self.dataDir + ' but does not contain Optimisation and/or ' + self.driverName + ' section']
=======
                            'config.ini found at ' + self.dataDir + ' but does not contain Optimisation and/or ' +
                            self.driverName + ' section']
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
                            'config.ini found at ' + self.dataDir + ' but does not contain Optimisation and/or ' +
                            self.driverName + ' section']
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
            else:
                return [False, 'config.ini not present in ' + self.dataDir]
        except:
            return [False, 'Initialising parameters failed']

    def f(self, x):
        self.numEvals += 1
        print 'Trial ', self.numEvals
        for j in range(len(x[0])):
            print self.domain[j]['name'], ' : ', x[0][j]
        print
        self.currIterSettings = self.sectionBackup
        combinationDicts = dict()
        for j in range(len(x[0])):
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
            if self.domain[j]['description'] == 'combination':
                if x[0][j] == 1:
=======
            if (self.domain[j]['description'] == 'combination'):
                if (x[0][j] == 1):
>>>>>>> New version of samSupervisor with rpc interface and README
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
            if (self.domain[j]['description'] == 'combination'):
                if (x[0][j] == 1):
=======
            if self.domain[j]['description'] == 'combination':
                if x[0][j] == 1:
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
            if self.domain[j]['description'] == 'combination':
                if x[0][j] == 1:
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                    val = True
                else:
                    val = False
                if val:
                    if self.domain[j]['groupName'] in combinationDicts:
                        combinationDicts[self.domain[j]['groupName']].append(self.domain[j]['name'])
                    else:
                        combinationDicts[self.domain[j]['groupName']] = [self.domain[j]['name']]
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
            elif self.domain[j]['description'] == 'discreteInt':
                self.parser.set(self.baseName, self.domain[j]['name'], str(int(x[0][j])))
            elif self.domain[j]['description'] == 'list':
                self.parser.set(self.baseName, self.domain[j]['name'], self.domain[j]['values'][int(x[0][j])])
            elif self.domain[j]['description'] == 'bool':
                if x[0][j] == 1:
                    val = 'True'
                else:
                    val = 'False'
                self.parser.set(self.baseName, self.domain[j]['name'], val)
            else:
                self.parser.set(self.baseName, self.domain[j]['name'], str(x[0][j]))
        for name, val in combinationDicts.iteritems():
            self.parser.set(self.baseName, name, ','.join(val) + ',')

        # for t in self.parser.items(self.baseName):
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
            elif (self.domain[j]['description'] == 'discreteInt'):
                self.parser.set(self.driverName, self.domain[j]['name'], str(int(x[0][j])))
            elif (self.domain[j]['description'] == 'bool'):
                if (x[0][j] == 1):
                    val = 'True'
                else:
                    val = 'False'
                self.parser.set(self.driverName, self.domain[j]['name'], val)
            else:
                self.parser.set(self.driverName, self.domain[j]['name'], str(x[0][j]))
        for name, val in combinationDicts.iteritems():
            self.parser.set(self.driverName, name, ','.join(val) + ',')

        # for t in self.parser.items(self.driverName):
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
            elif self.domain[j]['description'] == 'discreteInt':
                self.parser.set(self.baseName, self.domain[j]['name'], str(int(x[0][j])))
            elif self.domain[j]['description'] == 'list':
                self.parser.set(self.baseName, self.domain[j]['name'], self.domain[j]['values'][int(x[0][j])])
            elif self.domain[j]['description'] == 'bool':
                if x[0][j] == 1:
                    val = 'True'
                else:
                    val = 'False'
                self.parser.set(self.baseName, self.domain[j]['name'], val)
            else:
                self.parser.set(self.baseName, self.domain[j]['name'], str(x[0][j]))
        for name, val in combinationDicts.iteritems():
            self.parser.set(self.baseName, name, ','.join(val) + ',')

        # for t in self.parser.items(self.baseName):
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
            # print t

        # print
        # print
        self.parser.write(open(self.dataDir + "/config.ini", 'wb'))

<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
        args = ' '.join([self.dataDir, self.modelDir, self.driverName, 'new', self.baseName])
=======
        args = ' '.join([self.dataDir, self.modelDir, self.driverName, self.mode])
>>>>>>> New version of samSupervisor with rpc interface and README
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
        args = ' '.join([self.dataDir, self.modelDir, self.driverName, self.mode])
=======
        args = ' '.join([self.dataDir, self.modelDir, self.driverName, 'new', self.baseName])
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
        args = ' '.join([self.dataDir, self.modelDir, self.driverName, 'new', self.baseName])
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs

        cmd = self.fName + ' ' + args
        # NEW
        # cmd = trainPath + ' -- ' + args
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
        if self.persistence:
=======
        if (self.persistence):
>>>>>>> New version of samSupervisor with rpc interface and README
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
        if (self.persistence):
=======
        if self.persistence:
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
        if self.persistence:
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
            command = "bash -c \"" + cmd + "; exec bash\""
        else:
            command = "bash -c \"" + cmd + "\""

<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
        if self.verbose:
            print 'cmd: ', cmd

        # if self.windowed:
        deleteModel(self.modelDir, 'exp')
        if True:
=======
        if (self.verbose): print 'cmd: ', cmd

        if (self.windowed):
>>>>>>> New version of samSupervisor with rpc interface and README
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
        if (self.verbose): print 'cmd: ', cmd

        if (self.windowed):
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
        if self.verbose:
            print 'cmd: ', cmd

        # if self.windowed:
        deleteModel(self.modelDir, 'exp')
        if True:
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
            self.trainProcess = subprocess.Popen(['xterm', '-e', command], shell=False)
        else:
            self.trainProcess = subprocess.Popen([cmd], shell=True, stdout=self.devnull, stderr=self.devnull)

        ret = None
        cnt = 0
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
        while ret is None:
            ret = self.trainProcess.poll()
            time.sleep(5)
            cnt += 1
            if cnt > 5:
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
        while (ret == None):
            ret = self.trainProcess.poll()
            time.sleep(5)
            cnt += 1
            if(cnt > 5):
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
        while ret is None:
            ret = self.trainProcess.poll()
            time.sleep(5)
            cnt += 1
            if cnt > 5:
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                print 'Training ...'
                cnt = 0

        currError = 0
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
        print 'poll return:', ret
        # if len(self.currFiles) == 0:
        #     self.modelPresent = self.copyModel('backup', 'normal')
        #     self.copyModel('best', 'normal')
        if ret == 0:
            self.currFiles = [j for j in glob.glob('__'.join(self.modelDir.split('__')[:3]) + '*') if 'exp' in j]
            for j in self.currFiles:
                if '.pickle' in j and '__L' not in j:
                    modelPickle = pickle.load(open(j, 'rb'))
                    testConf = modelPickle['overallPerformance']
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
        if (len(self.currFiles) == 0):
            self.modelPresent = self.copyModel('backup','normal')
            self.copyModel('best', 'normal')
        print 'poll return:', ret
        if (ret == 0):
            for j in self.currFiles:
                if '.pickle' in j:
                    modelPickle = pickle.load(open(j, 'rb'))
                    testConf = modelPickle['testConf']
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
        print 'poll return:', ret
        # if len(self.currFiles) == 0:
        #     self.modelPresent = self.copyModel('backup', 'normal')
        #     self.copyModel('best', 'normal')
        if ret == 0:
            self.currFiles = [j for j in glob.glob('__'.join(self.modelDir.split('__')[:3]) + '*') if 'exp' in j]
            for j in self.currFiles:
                if '.pickle' in j and '__L' not in j:
                    modelPickle = pickle.load(open(j, 'rb'))
                    testConf = modelPickle['overallPerformance']
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                    np.fill_diagonal(testConf, 0)
                    currError += np.sum(testConf)
                    print 'Confusion Matrix: ', testConf
                    print
                    print 'Current cumulative error: ', currError
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
                    if currError < self.bestError:
                        deleteModel(self.modelDir, 'best')
                        self.bestError = currError
                        self.copyModel('best', 'normal')
                        self.bestOptions = copy.deepcopy(self.parser.items(self.baseName))
                        self.parser.write(open(self.dataDir + "/configBest.ini", 'wb'))

        else:
            currError = self.penalty
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
                    if (currError < self.bestError):
                        self.bestError = currError
                        self.copyModel('best', 'normal')
                        self.bestOptions = copy.deepcopy(self.parser.items(self.driverName))
                        self.parser.write(open(self.dataDir + "/configBest.ini", 'wb'))

        else:
            curError = 1000
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                    if currError < self.bestError:
                        deleteModel(self.modelDir, 'best')
                        self.bestError = currError
                        self.copyModel('best', 'normal')
                        self.bestOptions = copy.deepcopy(self.parser.items(self.baseName))
                        self.parser.write(open(self.dataDir + "/configBest.ini", 'wb'))

        else:
            currError = self.penalty
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
            print 'Error training model'
            print 'Current cumulative error: ', currError

        print 'Best Error so far : ', self.bestError
        print
        print '-----------------------------------------------------'
        return currError

    def copyModel(self, newName, direction):
        if os.path.isfile(self.modelDir):
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
            print self.modelDir, ' model file present'
            self.currFiles = [j for j in glob.glob('__'.join(self.modelDir.split('__')[:3]) + '*')
                              if 'backup' not in j and 'best' not in j]
            backupFiles = []
            for k in self.currFiles:
                print 'Original: ', k
                temp = k.split('exp')
                if '__L' in k:
                    temp2 = temp[1].split('__')
                    backupFiles += [temp[0] + newName + '__' + temp2[1]]
                else:
                    temp2 = temp[1].split('.')
                    if 'model' in temp[1]:
                        backupFiles += [temp[0] + newName + '_model.' + temp2[1]]
                    else:
                        backupFiles += [temp[0] + newName + '.' + temp2[1]]
                print 'New:     ', backupFiles[-1]
                print

            if direction == 'reverse':
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
            print self.modelDir , ' model file present'
            if ('L' in self.modelDir.split('__')[-1]):
                self.currFiles = [j for j in glob.glob('__'.join(self.modelDir.split('__')[:-1]) + '*')
                                  if 'backup' not in j and 'best' not in j]
                backupFiles = []
                for k in self.currFiles:
                    print 'Original: ', k
                    temp = k.split('exp')
                    temp2 = temp[1].split('__')
                    backupFiles += [temp[0] + newName + '__' + temp2[1]]
                    print 'New:     ', backupFiles[-1]
                    print
            else:
                self.currFiles = [j for j in glob.glob(self.modelDir.split('.')[0] + '*')
                                  if 'backup' not in j and 'best' not in j]
                backupFiles = []
                for k in self.currFiles:
                    print 'Original: ', k
                    temp = k.split('exp')
                    if 'model' in temp[1]:
                        backupFiles += [temp[0] + newName + '_model.' + temp[1].split('.')[1]]
                    else:
                        backupFiles += [temp[0] + newName + '.' + temp[1].split('.')[1]]
                    print 'New:      ', backupFiles[-1]
                    print
            if(direction == 'reverse'):
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
            print self.modelDir, ' model file present'
            self.currFiles = [j for j in glob.glob('__'.join(self.modelDir.split('__')[:3]) + '*')
                              if 'backup' not in j and 'best' not in j]
            backupFiles = []
            for k in self.currFiles:
                print 'Original: ', k
                temp = k.split('exp')
                if '__L' in k:
                    temp2 = temp[1].split('__')
                    backupFiles += [temp[0] + newName + '__' + temp2[1]]
                else:
                    temp2 = temp[1].split('.')
                    if 'model' in temp[1]:
                        backupFiles += [temp[0] + newName + '_model.' + temp2[1]]
                    else:
                        backupFiles += [temp[0] + newName + '.' + temp2[1]]
                print 'New:     ', backupFiles[-1]
                print

            if direction == 'reverse':
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                for j in range(len(backupFiles)):
                    shutil.copyfile(backupFiles[j], self.currFiles[j])
            else:
                for j in range(len(backupFiles)):
                    shutil.copyfile(self.currFiles[j], backupFiles[j])
            return True
        else:
            print 'No model present'
            self.currFiles = []
            return False


<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
=======
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
def deleteModel(modelDir, newName):
    if os.path.isfile(modelDir):
        print modelDir, ' model file present'
        fileList = [j for j in glob.glob('__'.join(modelDir.split('__')[:3]) + '*') if newName in j]
        for k in fileList:
            os.remove(k)


<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
=======
>>>>>>> New version of samSupervisor with rpc interface and README
=======
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
def main():
    # Initialisation parameters:
    print optNotFound,  ' ', len(sys.argv)
    if len(sys.argv) >= 9 and not optNotFound:
        a = sys.argv[1]
        b = sys.argv[2]
        c = sys.argv[3]
        d = sys.argv[4]
        e = sys.argv[5]
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
        f = sys.argv[6]
        per = sys.argv[7] == 'True'
        wind = sys.argv[8] == 'True'
        verb = sys.argv[9] == 'True'

        optModel = modelOptClass(a, b, c, d, e, f, per, wind, verb)
        if optModel.configured[0]:
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
        per = sys.argv[6] == 'True'
        wind = sys.argv[7] == 'True'
        verb = sys.argv[8] == 'True'

        optModel = modelOptClass(a, b, c, d, e, per, wind, verb)
        if optModel.configured[0] :
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
        f = sys.argv[6]
        per = sys.argv[7] == 'True'
        wind = sys.argv[8] == 'True'
        verb = sys.argv[9] == 'True'

        optModel = modelOptClass(a, b, c, d, e, f, per, wind, verb)
        if optModel.configured[0]:
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
            myBopt = GPyOpt.methods.BayesianOptimization(f=optModel.f,  # function to optimize
                                                         domain=optModel.domain,  # box-constrains of the problem
                                                         initial_design_numdata=10,  # number data initial design
                                                         acquisition_type=optModel.acquisitionFunction)
            max_iter = 200

            myBopt.run_optimization(max_iter)
            optModel.parser.write(open(optModel.dataDir + "/config.ini", 'wb'))
            optModel.copyModel('best', 'reverse')
            return 0
        else:
            return -1
    else:
        print 'GPyOpt package not found or incorrect number of arguments'
        return -1

if __name__ == '__main__':
    main()
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
=======

>>>>>>> New version of samSupervisor with rpc interface and README
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28

=======
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
