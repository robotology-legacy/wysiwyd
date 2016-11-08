#!/usr/bin/env ipython
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
    def __init__(self, fName, dataDir, modelDir, driverName, mode, baseName, persistence, windowed, verbose):
        # check package is present
        try:
            import GPyOpt
            self.fName = fName
            self.dataDir = dataDir
            self.modelDir = modelDir
            self.driverName = driverName
            self.baseName = baseName
            self.persistence = persistence
            self.verbose = verbose
            self.devnull = None
            self.windowed = windowed
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
            self.resultsList = []
            self.currFiles = None
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
                if (self.parser.has_section('Optimisation') and self.parser.has_section(
                        self.baseName)):
                    # create backup of current self.driverName section
                    self.sectionBackup = dict(self.parser.items(self.baseName))

                    # load Optimisation section
                    self.sectionOpt = dict(self.parser.items('Optimisation'))
                    if len(self.sectionOpt) == 1 and 'acquisitionFunction' in self.sectionOpt:
                        return [False,
                                'config.ini found, Optimisation and ' + self.baseName +
                                ' sections found but Optimisation section does not contain parameters to optimise']
                    else:
                        # create backup if model in modelDir exists
                        self.modelPresent = self.copyModel('backup', 'normal')

                        # load performance of current model if one is available and track performance
                        self.bestError = 0
                        try:
                            if len(self.currFiles) > 0:
                                for j in self.currFiles:
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

                        # iterate over keys of sectionOpt to create domain of optimisation problem
                        self.acquisitionFunction = 'MPI'
                        self.domain = []  # list of dictionaries
                        # armedBanditsMode = True
                        self.numPossibilities = 1
                        for i, v in self.sectionOpt.iteritems():
                            if i == 'acquisitionFunction':
                                # possible acquisition functions
                                # 'MPI' : maximum probability of improvement
                                # 'EI'  : Expected Improvement
                                # 'UCB' : Upper class bound
                                if v == 'MPI' or v == 'EI' or v == 'UCB':
                                    self.acquisitionFunction = v
                            elif i in self.sectionBackup:
                                opts = v.partition('[')[-1].rpartition(']')[0]
                                sects = opts.split(':')
                                tempDict = dict()
                                if sects[0] == 'discreteInt':
                                    lims = sects[1].split(',')
                                    arr = np.arange(int(lims[0]), int(lims[2]), int(lims[1]))
                                    arr = np.hstack((arr, int(lims[2])))
                                    self.numPossibilities *= len(arr)
                                    tempDict['name'] = i
                                    tempDict['type'] = 'discrete'
                                    tempDict['domain'] = arr
                                    tempDict['description'] = sects[0]
                                    self.domain.append(tempDict)
                                elif sects[0] == 'discreteFloat':
                                    lims = sects[1].split(',')
                                    arr = np.arange(float(lims[0]), float(lims[2]), float(lims[1]))
                                    arr = np.hstack((arr, float(lims[2])))
                                    self.numPossibilities *= len(arr)
                                    tempDict['name'] = i
                                    tempDict['type'] = 'discrete'
                                    tempDict['domain'] = arr
                                    tempDict['description'] = sects[0]
                                    self.domain.append(tempDict)
                                elif sects[0] == 'continuous':
                                    # armedBanditsMode = False
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
                                    self.numPossibilities *= 2
                                    self.domain.append(tempDict)
                                elif sects[0] == 'combination':
                                    splitList = sects[1].split(',')
                                    for b in splitList:
                                        tempDict = dict()
                                        tempDict['name'] = b
                                        tempDict['type'] = 'discrete'
                                        tempDict['domain'] = np.array((0, 1))
                                        self.numPossibilities *= 2
                                        tempDict['description'] = sects[0]
                                        tempDict['groupName'] = i
                                        self.domain.append(tempDict)
                                elif sects[0] == 'list':
                                    splitList = sects[1].split(',')
                                    tempDict = dict()
                                    tempDict['name'] = i
                                    tempDict['type'] = 'discrete'
                                    tempDict['domain'] = np.arange(0, len(splitList), 1)
                                    tempDict['description'] = sects[0]
                                    tempDict['groupName'] = i
                                    tempDict['values'] = splitList
                                    self.numPossibilities *= len(splitList)
                                    self.domain.append(tempDict)
                            else:
                                print 'ignoring ', i

                        # if armedBanditsMode :
                        #     for j in self.domain:
                        #         j['type'] = 'bandit'

                        for j in self.domain:
                            print j
                            print

                        return [True, 'Optimisation configuration ready']
                else:
                    return [False,
                            'config.ini found at ' + self.dataDir + ' but does not contain Optimisation and/or ' +
                            self.driverName + ' section']
            else:
                return [False, 'config.ini not present in ' + self.dataDir]
        except:
            return [False, 'Initialising parameters failed']

    def f(self, x):
        self.numEvals += 1
        print 'Trial ', self.numEvals, 'out of', self.numPossibilities, 'possibilities'
        for j in range(len(x[0])):
            print self.domain[j]['name'], ' : ', x[0][j]
        print
        self.currIterSettings = self.sectionBackup
        combinationDicts = dict()
        for j in range(len(x[0])):
            if self.domain[j]['description'] == 'combination':
                if x[0][j] == 1:
                    val = True
                else:
                    val = False
                if val:
                    if self.domain[j]['groupName'] in combinationDicts:
                        combinationDicts[self.domain[j]['groupName']].append(self.domain[j]['name'])
                    else:
                        combinationDicts[self.domain[j]['groupName']] = [self.domain[j]['name']]
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
            # print t

        # print
        # print
        self.parser.write(open(self.dataDir + "/config.ini", 'wb'))

        args = ' '.join([self.dataDir, self.modelDir, self.driverName, 'new', self.baseName])

        cmd = self.fName + ' ' + args
        # NEW
        # cmd = trainPath + ' -- ' + args
        if self.persistence:
            command = "bash -c \"" + cmd + "; exec bash\""
        else:
            command = "bash -c \"" + cmd + "\""

        if self.verbose:
            print 'cmd: ', cmd

        # if self.windowed:
        deleteModel(self.modelDir, 'exp')
        if True:
            self.trainProcess = subprocess.Popen(['xterm', '-e', command], shell=False)
        else:
            self.trainProcess = subprocess.Popen([cmd], shell=True, stdout=self.devnull, stderr=self.devnull)

        ret = None
        cnt = 0
        totalTime = 0
        while ret is None:
            ret = self.trainProcess.poll()
            time.sleep(5)
            cnt += 1
            if cnt > 5:
                totalTime += 1
                print 'Training ...', totalTime * 0.5, 'minutes elapsed'
                cnt = 0

        currError = 0
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
                    print 'Confusion Matrix: ', testConf
                    np.fill_diagonal(testConf, 0)
                    currError += np.sum(testConf)
                    print
                    print 'Current cumulative error: ', currError
                    if currError < self.bestError:
                        deleteModel(self.modelDir, 'best')
                        self.bestError = currError
                        self.copyModel('best', 'normal')
                        self.bestOptions = copy.deepcopy(self.parser.items(self.baseName))
                        self.parser.write(open(self.dataDir + "/configBest.ini", 'wb'))

        else:
            currError = self.penalty
            print 'Error training model'
            print 'Current cumulative error: ', currError

        print 'Best Error so far : ', self.bestError
        print
        print '-----------------------------------------------------'
        self.resultsList.append([x, currError])
        return currError

    def copyModel(self, newName, direction):
        if os.path.isfile(self.modelDir):
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


def deleteModel(modelDir, newName):
    if os.path.isfile(modelDir):
        print modelDir, ' model file present'
        fileList = [j for j in glob.glob('__'.join(modelDir.split('__')[:3]) + '*') if newName in j]
        for k in fileList:
            os.remove(k)


def main():
    # Initialisation parameters:
    print optNotFound,  ' ', len(sys.argv)
    if len(sys.argv) >= 9 and not optNotFound:
        a = sys.argv[1]
        b = sys.argv[2]
        c = sys.argv[3]
        d = sys.argv[4]
        e = sys.argv[5]
        f = sys.argv[6]
        per = sys.argv[7] == 'True'
        # per = True
        wind = sys.argv[8] == 'True'
        verb = sys.argv[9] == 'True'

        optModel = modelOptClass(a, b, c, d, e, f, per, wind, verb)
        if optModel.configured[0]:
            myBopt = GPyOpt.methods.BayesianOptimization(f=optModel.f,  # function to optimize
                                                         domain=optModel.domain,  # box-constrains of the problem
                                                         initial_design_numdata=2,  # number data initial design
                                                         acquisition_type=optModel.acquisitionFunction)
            max_iter = 200
            # pickle myBopt to save its initialisation
            # logFilename = os.path.join(b, 'optimiserLog')
            # output = open(logFilename, 'wb')
            # pickle.dump({'optModel', optModel}, output)
            # output.close()

            # try:
            myBopt.run_optimization(max_iter)
            return 0
            # except:
            #     # pickle results list together with optimiser
            #     d = pickle.load(open(logFilename, 'r'))
            #     d['resultList'] = myBopt.resultsList
            #     output = open(logFilename, 'wb')
            #     pickle.dump(d, output)
            #     output.close()
            #     optModel.parser.write(open(optModel.dataDir + "/config.ini", 'wb'))
            #     optModel.copyModel('best', 'reverse')
            #     return -1
        else:
            return -1
    else:
        print 'GPyOpt package not found or incorrect number of arguments'
        return -1

if __name__ == '__main__':
    main()
