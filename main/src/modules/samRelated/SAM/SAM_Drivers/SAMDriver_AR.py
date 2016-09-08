# """"""""""""""""""""""""""""""""""""""""""""""
# The University of Sheffield
# WYSIWYD Project
#
# A driver that implements Action Recognition
#
# Created on 10 April 2016
#
# @author: Daniel Camilleri, Andreas Damianou
#
# """"""""""""""""""""""""""""""""""""""""""""""
import copy
import math
from os import listdir
from os.path import isfile, join
import numpy
import numpy as np
from SAM.SAM_Core import SAMDriver
from SAM.SAM_Core import SAMTesting
np.set_printoptions(threshold=numpy.nan)


class SAMDriver_AR(SAMDriver):

    def __init__(self):
        SAMDriver.__init__(self)
        self.data = dict()
        self.jointsList = []
        self.objectsList = []
        self.dataLogList = []
        self.labelsLogList = []
        # self.combinationList = []
        # self.combinationKeys = []
        self.humanStaticLabels = None
        self.featureInds = None
        self.featureSections = None
        self.featureValues = None
        self.numJoints = 9
        self.labelToRemove = []
        self.indsToRemove = []
        self.actionsAllowed = None
        self.contactThreshold = None
        self.seqConf = None
        self.seqPerc = None
        self.joint = 0
        self.action = 1
        self.dataset = 2
        self.start = 3
        self.end = 4

        self.additionalParametersList = ['Yall', 'Lall', 'Ytest', 'Ltest', 'numJoints',
                                         'Ymean', 'Ystd', 'Y_normalized', 'ignoreStationary',
                                         'humanStaticLabels', 'featureSections', 'featureValues',
                                         'dataLogList', 'labelsLogList', 'segTrainConf', 'segTrainPerc',
                                         'segTestConf', 'segTestPerc', 'seqConf', 'seqPerc']

    def loadParameters(self, parser, trainName):

        if parser.has_option(trainName, 'ignoreLabels'):
            self.paramsDict['ignoreLabels'] = parser.get(trainName, 'ignoreLabels').split(',')
        else:
            self.paramsDict['ignoreLabels'] = ['agent_entry', 'agent_exit', 'no_agent']

        if parser.has_option(trainName, 'ignoreParts'):
            self.paramsDict['ignoreParts'] = parser.get(trainName, 'ignoreParts').split(',')
        else:
            self.paramsDict['ignoreParts'] = ['partner']

        if parser.has_option(trainName, 'actionsAllowedList'):
            self.paramsDict['actionsAllowedList'] = parser.get(trainName, 'actionsAllowedList').split(',')
        else:
            self.paramsDict['actionsAllowedList'] = ['lift_object', 'pull_object', 'push_object', 'drop_object',
                                                     'carry_object']

        if parser.has_option(trainName, 'angleThreshold'):
            self.paramsDict['angleThreshold'] = float(parser.get(trainName, 'angleThreshold'))
        else:
            self.paramsDict['angleThreshold'] = 0.01

        if parser.has_option(trainName, 'percentContactThreshold'):
            self.paramsDict['percentContactThreshold'] = float(parser.get(trainName, 'percentContactThreshold'))
        else:
            self.paramsDict['percentContactThreshold'] = 98.0

        if parser.has_option(trainName, 'featuresToUse'):
            self.paramsDict['featuresToUse'] = []
            temp = parser.get(trainName, 'featuresToUse')
            temp = temp.split(',')
            for j in temp:
                if j != '':
                    self.paramsDict['featuresToUse'].append(j)
        else:
            self.paramsDict['featuresToUse'] = ['contact', 'selfMovementLabelK']

        if parser.has_option(trainName, 'compressData'):
            self.paramsDict['compressData'] = parser.get(trainName, 'compressData')
        else:
            self.paramsDict['compressData'] = True

        if parser.has_option(trainName, 'ignoreStationary'):
            self.paramsDict['ignoreStationary'] = parser.get(trainName, 'ignoreStationary')
        else:
            self.paramsDict['ignoreStationary'] = True

        if parser.has_option(trainName, 'featuresToCompress'):
            self.paramsDict['featuresToCompress'] = parser.get(trainName, 'featuresToCompress').split(',')
        else:
            self.paramsDict['featuresToCompress'] = ['selfMovementLabelK']

        if parser.has_option(trainName, 'maxNumItems'):
            self.paramsDict['maxNumItems'] = int(parser.get(trainName, 'maxNumItems'))
        else:
            self.paramsDict['maxNumItems'] = 20000

        if parser.has_option(trainName, 'deltaDistanceThreshold'):
            self.paramsDict['deltaDistanceThreshold'] = float(parser.get(trainName, 'deltaDistanceThreshold'))
        else:
            self.paramsDict['deltaDistanceThreshold'] = 0.01

        if parser.has_option(trainName, 'jointMu'):
            self.paramsDict['jointMu'] = float(parser.get(trainName, 'jointMu'))
        else:
            self.paramsDict['jointMu'] = 0

        if parser.has_option(trainName, 'jointSig'):
            self.paramsDict['jointSig'] = float(parser.get(trainName, 'jointSig'))
        else:
            self.paramsDict['jointSig'] = 0.0000001

    def saveParameters(self):
        for j in self.additionalParametersList:
            commandString = 'self.paramsDict[\'' + j + '\'] = self.' + j
            try:
                print commandString
                exec commandString
            except:
                pass

    def labelize(self, qin, qout):
        for h in range(qin.shape[0]):
            # x at 0 is depth with negative meaning behind an object
            # y at 1 is width with negative meaning partner-right = ego-left
            # x at 2 is height with negative meaning underneath
            currVec = np.abs(qin[h, :])
            if currVec.sum() != 0:
                maxIDx = currVec.argmax()
                if maxIDx == 0:
                    if np.sign(qin[h, maxIDx]) == -1:
                        # behind
                        qout[h] = self.humanStaticLabels.index('behind')
                    elif np.sign(qin[h, maxIDx]) == 1:
                        # in front
                        qout[h] = self.humanStaticLabels.index('in front of')
                elif maxIDx == 1:
                    if np.sign(qin[h, maxIDx]) == -1:
                        # left
                        qout[h] = self.humanStaticLabels.index('left of')
                    elif np.sign(qin[h, maxIDx]) == 1:
                        # right
                        qout[h] = self.humanStaticLabels.index('right of')
                elif maxIDx == 2:
                    if np.sign(qin[h, maxIDx]) == -1:
                        # underneath
                        qout[h] = self.humanStaticLabels.index('underneath')
                    elif np.sign(qin[h, maxIDx]) == 1:
                        # top of
                        qout[h] = self.humanStaticLabels.index('on top of')
            else:
                qout[h] = self.humanStaticLabels.index('stationary')

    def extractFeatures(self, Pk, Pl):
        # add gaussian noise on output
        Pk_Noise = np.random.normal(self.paramsDict['jointMu'], self.paramsDict['jointSig'], Pk.shape)
        Pl_Noise = np.random.normal(self.paramsDict['jointMu'], self.paramsDict['jointSig'], Pl.shape)

        Pk = Pk + Pk_Noise
        Pl = Pl + Pl_Noise

        q1 = np.zeros((Pk.shape[0]-2, 1), dtype=np.int)  # motion of k relative to l
        q2 = np.zeros((Pk.shape[0]-2, 1), dtype=np.int)  # motion of l relative to k
        q3 = np.zeros((Pk.shape[0]-2, 1), dtype=np.int)  # alpha
        q4 = np.zeros((Pk.shape[0]-2, 1), dtype=np.int)  # beta
        q5 = np.zeros((Pk.shape[0]-2, 1), dtype=np.int)  # gamma
        q7 = np.zeros((Pk.shape[0]-2, 1), dtype=np.int)  # contact
        q8 = np.zeros((Pk.shape[0]-2, 1), dtype=np.int)  # human static label

        self.filterMovement(Pk, self.paramsDict['deltaDistanceThreshold'])
        self.filterMovement(Pl, self.paramsDict['deltaDistanceThreshold'])

        q6 = Pl[1:-1]-Pk[1:-1]  # direction vector from joint to joint

        q9 = self.qtc_2D(Pk, Pl, q1, self.paramsDict['deltaDistanceThreshold'])
        lowV = np.abs(q9) < self.contactThreshold
        q7[lowV] = 1

        q6 = q6/q9[:, None]

        self.qtc_2D(Pl, Pk, q2, self.paramsDict['deltaDistanceThreshold'])
        self.qtc_3D(Pk, Pl, self.paramsDict['angleThreshold'], q3, q4, q5)

        self.labelize(q6, q8)

        q10 = Pl[1:-1]
        q11 = Pk[1:-1]

        q13 = np.ones((Pk.shape[0]-2, 1), dtype=np.int)
        q12mag = self.distEuc(Pk[1:-1], Pk[:-2])
        lowV = np.abs(q12mag) < self.paramsDict['deltaDistanceThreshold']
        q12 = (Pk[1:-1] - Pk[:-2])/q12mag[:, None]
        q12[lowV] = 0
        self.labelize(q12, q13)

        q15 = np.ones((Pk.shape[0]-2, 1), dtype=np.int)
        q14mag = self.distEuc(Pl[1:-1], Pl[:-2])
        lowV = np.abs(q14mag) < self.paramsDict['deltaDistanceThreshold']
        q14 = (Pl[1:-1] - Pl[:-2])/q14mag[:, None]
        q14[lowV] = 0
        self.labelize(q14, q15)

        tempQTC = np.hstack((q1, q2, q3, q4, q5, q6[:, 0, None], q6[:, 1, None], q6[:, 2, None], q7, q8,
                             q9[:, None], q10[:, 0, None], q10[:, 1, None], q10[:, 2, None], q11[:, 0, None],
                             q11[:, 1, None], q11[:, 2, None], q13, q15))
        if self.paramsDict['compressData']:
            tempQTC = self.removeStationary(tempQTC, self.labelToRemove, self.indsToRemove)
#             v = formatFeatures(tempQTC, 18)
        return tempQTC

    def configProcessing(self):
        if self.paramsDict['compressData'] and \
                set(self.paramsDict['featuresToCompress']).issubset(self.paramsDict['featuresToUse']):
            self.labelToRemove = self.humanStaticLabels.index('stationary')
            self.indsToRemove = []
            for i in self.paramsDict['featuresToCompress']:
                self.indsToRemove.append(self.featureSections[i])
            print 'Compressing data by removing ', self.humanStaticLabels[self.labelToRemove]
            print 'Applying compression to features:'
            for i in self.indsToRemove:
                print i
        else:
            print 'No compression applied'
            self.paramsDict['compressData'] = False

        featureInds = []
        print 'Features to use:'
        for h in self.paramsDict['featuresToUse']:
            featureInds += self.featureSections[h]
            print '\t', h, self.featureSections[h]
        print featureInds
        return featureInds

    def chooseFeaturesOld(self, handDataStruct):
        v = np.array(len(handDataStruct))
        for j in self.featureInds:
            vec = self.formatFeatures2(handDataStruct, self.featureSections[j], self.featureValues[j])
            if vec[0] != -1:
                v = np.hstack((v, vec))
        return v

    def chooseFeatures(self, handDataStruct):
        v = np.array(len(handDataStruct))
        vec2 = None
        for j in range(len(self.paramsDict['featuresToUse'])):

            vec = self.formatFeatures2(handDataStruct, self.featureSections[self.paramsDict['featuresToUse'][j]],
                                       self.featureValues[self.paramsDict['featuresToUse'][j]])

            # Remove last index and normalise wr the sum of frames
            if self.paramsDict['ignoreStationary'] and self.paramsDict['featuresToUse'][j] in \
                    ['relativePositionLabel', 'selfMovementLabelL', 'selfMovementLabelK']:
                if j == 0:
                    vec2 = vec[:-1] / v
                else:
                    vec2 = np.hstack((vec2, vec[:-1] / v))
            else:
                if j == 0:
                    vec2 = vec / v
                else:
                    vec2 = np.hstack((vec2, vec/v))
            vec2 = np.nan_to_num(vec2)

        return vec2

    @staticmethod
    def formatFeatures2(inputArr, idx, valsPossible):
        # print str(valsPossible) + str(len(valsPossible))
        # print str(idx) +str(len(idx))
        if len(valsPossible) > 0:  # extract features from discrete data
            vec = np.zeros((len(valsPossible)*len(idx)))
            for b in range(len(idx)):
                for i in range(len(inputArr)):
                    offset = (len(valsPossible)*b)
                    vec[offset + valsPossible.index(inputArr[i][idx[b]])] += 1
    #                 print
    #         print vec
        else:  # extract features from continuous data
            vec = 1

        return vec

    def readData(self, root_data_dir, participant_index, *args, **kw):
        # this function reads from data files and prepares a Y and an X
        onlyfiles = [f for f in listdir(root_data_dir) if isfile(join(root_data_dir, f))]
        dataLogList = [f for f in onlyfiles if 'data' in f]
        dataLogList.sort()
        labelsLogList = [f for f in onlyfiles if 'label' in f]
        labelsLogList.sort()

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
            dataFile = open(join(root_data_dir, dataLogList[k]), 'r')
            self.dataLogList.append(str(join(root_data_dir, dataLogList[k])))
            labelFile = open(join(root_data_dir, labelsLogList[k]), 'r')
            self.labelsLogList.append(join(root_data_dir, labelsLogList[k]))

            # number of lines in dataFile
            for i, l in enumerate(dataFile):
                    pass
            lenDataFile = i+1

            # number of lines in labelFile
            for i, l in enumerate(labelFile):
                    pass
            lenLabelFile = i+1
            dataFile.close()
            labelFile.close()

            if lenLabelFile != lenDataFile:
                print str(dataLogList[k]) + ' will not be used because its lenght differs from ' + str(labelsLogList[k])
            else:
                dataFile = open(join(root_data_dir, dataLogList[k]), 'r')
                labelFile = open(join(root_data_dir, labelsLogList[k]), 'r')
                labelsList.append([])

                for curr in range(lenDataFile):
                    line = dataFile.readline()
                    labelLine = labelFile.readline()

                    t = line.replace('(', '').replace(')', '').split(' ')
                    del t[0:4]

                    v = labelLine.split(' ')[2].replace('\n', '').replace('(', '').replace(')', '')
                    if v == '':
                        v = 'unknown'
                    labelsList[k].append(v)

                    # parse skeleton data which has 9 sections by (x,y,z)
                    for i in range(self.numJoints):
                        a = i*4
                        if t[a] == 'shoulderCenter':
                            t[a] = 'chest'

                        if firstPass:
                            data[t[a]] = [None]*numFiles
                            data[t[a]][k] = (np.array([float(t[a+1]), float(t[a+2]), float(t[a+3])]))
                            jointsList.append(t[a])
                        else:
                            arr = np.array([float(t[a+1]), float(t[a+2]), float(t[a+3])])
                            if data[t[a]][k] is not None:
                                data[t[a]][k] = np.vstack((data[t[a]][k], arr))
                            else:
                                data[t[a]][k] = arr

                    currIdx = (self.numJoints*4 - 1)
                    numObjs = (len(t) - currIdx)/5

                    for i in range(numObjs):
                        a = currIdx + 1 + (i*5)
                        if t[a] in data:
                            arr = np.array([float(t[a+1]), float(t[a+2]), float(t[a+3])])
                            if data[t[a]][k] is not None:
                                data[t[a]][k] = np.vstack((data[t[a]][k], arr))
                            else:
                                data[t[a]][k] = arr
                        else:
                            data[t[a]] = [None]*(numFiles+1)
                            data[t[a]][k] = np.array([float(t[a+1]), float(t[a+2]), float(t[a+3])])
                            objectsList.append(t[a])

                    firstPass = False
                dataFile.close()
                labelFile.close()

        if self.verbose:
            print 'data has length = ' + str(len(data)) + ' joints'
            strl = 'each joint has ' + str(len(data['head'])) + ' arrays of shape: \n'

            for i in data['head']:
                strl += '\t\t' + str(i.shape) + '\n'
            print strl

            strl = 'labelsList has length = ' + str(len(labelsList)) + ' with sizes: \n'
            for i in labelsList:
                strl += '\t\t' + str(len(i)) + '\n'
            print strl

        setList = []
        for x in labelsList:
            setList.append(list(set(x)))
        flattenedList = [val for sublist in setList for val in sublist]
        labels = list(set(flattenedList))
        labels.sort()
        if self.verbose:
            print
            print 'Unique labels in labels files:'
            print
            for k in range(0, len(labels)):
                print '\t' + str(k).ljust(3) + labels[k]
            print
            print 'Of these, config.ini specifies [' + ', '.join(self.paramsDict['ignoreLabels']) + '] to be ignored'

        ignoreInds = []
        for k in self.paramsDict['ignoreLabels']:
            ignoreInds.append(labels.index(k))

        # the following block of code extracts blocks of actions from the log files
        dataStruct = []
        for arr in range(len(dataLogList)):
            for label in labels:
                idxs = [i for i in range(len(labelsList[arr])) if labelsList[arr][i] == label]
                # actionBlocks = []
                startIdx = 0
                if len(idxs) > 5:
                    for idxIndex in range(len(idxs)):
                        if idxIndex != 0:
                            if idxs[idxIndex] - idxs[idxIndex - 1] != 1:
                                if idxIndex - startIdx > 5:
                                    print idxs[startIdx:idxIndex]
                                    for joint in jointsList + objectsList[1:]:
                                        actionData = data[joint][arr][idxs[startIdx:idxIndex]]
                                        dataStruct.append([joint, label, arr, actionData.shape[0], actionData])
                                    startIdx = idxIndex

                            if idxIndex + 2 > len(idxs):
                                if idxIndex - startIdx > 5:
                                    print idxs[startIdx:idxIndex + 1]
                                    for joint in jointsList + objectsList[1:]:
                                        actionData = data[joint][arr][idxs[startIdx:idxIndex + 1]]
                                        dataStruct.append([joint, label, arr, actionData.shape[0], actionData])

        for a in range(len(dataLogList)):
            print 'Dataset ' + str(a) + ' : ' + dataLogList[a]
            for b in range(len(labels)):
                if 'no' not in labels[b]:
                    y = len([i[1] for i in dataStruct if i[self.joint] == 'head' and
                             i[self.dataset] == a and i[self.action] == labels[b]])
                    print '\t ' + str(labels[b]).ljust(35) + ' = ' + str(y).ljust(3) + ' repetitions'
            print

        # Calibrate contact threshold
        # data has length = 12 joints
        # each joint has 4 arrays of shape: (4637, 3), (4907, 3), (5914, 3), (5440, 3),
        # labelsList has length = 4 with sizes: 4637, 4907, 5914, 5440,
        handPos = np.zeros((1, 3))
        objectPos = np.zeros((1, 3))

        # push_object_car_hand_right

        for i in range(len(labelsList)):
            for j in range(len(labelsList[i])):
                if 'object' in labelsList[i][j] and 'reach' not in labelsList[i][j] and "no" not in labelsList[i][j]:
                    if 'left' in labelsList[i][j]:
                        handPos = np.vstack((handPos, data['handLeft'][i][j, :, None].T))
                        obj = labelsList[i][j].split('_')[2]
                        objectPos = np.vstack((objectPos, data[obj][i][j, :, None].T))

                    if 'right' in labelsList[i][j]:
                        handPos = np.vstack((handPos, data['handRight'][i][j, :, None].T))
                        obj = labelsList[i][j].split('_')[2]
                        objectPos = np.vstack((objectPos, data[obj][i][j, :, None].T))

        d = self.distEuc(handPos, objectPos)
        if self.verbose:
            print 'Mean of contact distances = ' + str(np.mean(d))
            print 'Median of contact distances = ' + str(np.median(d))
            print 'Max contact distance = ' + str(np.max(d))
            print 'Min contact distance = ' + str(np.min(d))
            print str(self.paramsDict['percentContactThreshold']) + '% percentile contact distance = ' + str(
                np.percentile(d, self.paramsDict['percentContactThreshold']))

        self.contactThreshold = np.percentile(d, self.paramsDict['percentContactThreshold'])
        jointsList.sort()
        modJointsListHand_R = []
        modJointsListHand_L = []
        combinationList = []

        if len(modJointsListHand_R) > 0:
            for i in range(len(modJointsListHand_R)):
                combinationList.append(['handLeft', modJointsListHand_L[i]])
                combinationList.append(['handRight', modJointsListHand_R[i]])
            del combinationList[combinationList.index(['handLeft', 'handRight'])]

        for i in objectsList[1:]:
            combinationList.append(['handLeft', i])
            combinationList.append(['handRight', i])

        if self.verbose:
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

        for ac in self.paramsDict['actionsAllowedList']:
            temp = [g for g in labels if ac in g and "no" not in g]
            for h in range(len(temp)):
                self.actionsAllowed.append(temp[h])

        print
        for b in range(len(labels)):
            if 'no' not in labels[b] and labels[b] in self.actionsAllowed:
                y = 0
                for a in range(len(dataLogList)):
                    y += len([i[1] for i in dataStruct if i[self.joint] == 'head' and
                              [self.dataset] == a and i[self.action] == labels[b]])
                if self.verbose:
                    print str(labels[b]).ljust(35) + ' = ' + str(y).ljust(3) + ' repetitions'

        self.featureSections = dict()
        self.featureValues = dict()
        self.featureSections['QTC_Motion'] = range(0, 2)
        self.featureValues['QTC_Motion'] = [-1, 0, 1]

        self.featureSections['QTC_Orientation'] = range(2, 5)
        self.featureValues['QTC_Orientation'] = [-1, 0, 1]

        self.featureSections['directionVector'] = range(5, 8)
        self.featureValues['directionVector'] = []

        self.featureSections['contact'] = [8]
        self.featureValues['contact'] = [0, 1]

        self.featureSections['relativePositionLabel'] = [9]
        self.featureValues['relativePositionLabel'] = range(len(self.humanStaticLabels))

        self.featureSections['euc_distance'] = [10]
        self.featureValues['euc_distance'] = []

        self.featureSections['posL'] = range(11, 14)
        self.featureValues['posL'] = []

        self.featureSections['posK'] = range(14, 17)
        self.featureValues['posK'] = []

        self.featureSections['selfMovementLabelL'] = [17]
        self.featureValues['selfMovementLabelL'] = range(len(self.humanStaticLabels))

        self.featureSections['selfMovementLabelK'] = [18]
        self.featureValues['selfMovementLabelK'] = range(len(self.humanStaticLabels))

        self.featureInds = self.configProcessing()

        # firstPass = True
        handDataStruct = []

        for currComb in combinationList:
            print
            print str(currComb[0]) + '-' + str(currComb[1])
            infoK = [i for i in dataStruct if i[self.joint] == currComb[0]]
            infoL = [i for i in dataStruct if i[self.joint] == currComb[1]]
            for i in range(len(infoK)):
                if infoK[i][1] in self.actionsAllowed:
                    if 'hand' in infoK[i][0] and 'object' in infoK[i][1]:
                        if 'Left' in infoK[i][0] and 'left' in infoK[i][1]:
                            condition = True
                        elif 'Right' in infoK[i][0] and 'right' in infoK[i][1]:
                            condition = True
                        else:
                            condition = False

                        if infoL[i][0] not in infoK[i][1]:
                            condition = False
                            # obj = True

                    elif 'hand' not in infoK[i][0] and 'object' in infoK[i][1]:
                        condition = False
                    else:
                        condition = True
                else:
                    condition = False

                if condition:

                    Pk = infoK[i][4]
                    Pl = infoL[i][4]
                    tempQTC = self.extractFeatures(Pk, Pl)
                    handDataStruct.append([currComb[0], currComb[1], infoK[i][1], infoK[i][2],
                                           tempQTC.shape[0], tempQTC])

                    if self.verbose:
                        print '[%s]' % ', '.join(map(str, handDataStruct[-1][:-1]))

        for n in range(len(handDataStruct)):
            handDataStruct[n][5] = self.chooseFeatures(handDataStruct[n][5])

        for a in range(len(handDataStruct)):
            for r in self.paramsDict['actionsAllowedList']:
                if r in handDataStruct[a][2]:
                    labelNum = r

            if a == 0:
                self.Y = handDataStruct[a][5]
                self.L = [labelNum]
            else:
                self.Y = np.vstack((self.Y, handDataStruct[a][5]))
                self.L.append(labelNum)

        if self.verbose:
            print self.Y.shape
            print len(self.L)

    def testPerformance(self, testModel, Yall, Lall, YtestAll, LtestAll, verbose):

        yTrainingData = SAMTesting.formatDataFunc(Yall)
        [self.segTrainConf, self.segTrainPerc] = SAMTesting.testSegments(testModel, yTrainingData, Lall, verbose)

        yTrainingData = SAMTesting.formatDataFunc(YtestAll)
        [self.segTestConf, self.segTestPerc] = SAMTesting.testSegments(testModel, yTrainingData, LtestAll, verbose)

        # self.sequenceConfig()
        # [self.seqConf, self.seqPerc] = self.testSequence(testModel)

        return self.segTestConf

    @staticmethod
    def distEuc(a, b):
        temp = a-b
        temp = np.square(temp)
        temp = np.sum(temp, 1)
        return np.sqrt(temp)

    def qtc_2D(self, k, l, q, thresh):

        d1 = self.distEuc(k[:-2], l[1:-1])
        d2 = self.distEuc(k[1:-1], l[1:-1])
        d3 = self.distEuc(k[2:], l[1:-1])

        for i in range(len(d1)):
            # threshold distance moved
            diff1 = d2[i]-d1[i]
            if np.abs(diff1) < thresh:
                diff1 = 0

            diff2 = d3[i]-d2[i]
            if np.abs(diff2) < thresh:
                diff2 = 0

            # convert to qtc
            if diff1 > 0 and diff2 > 0:
                q[i] = -1
            elif diff1 < 0 and diff2 < 0:
                q[i] = +1
            else:
                q[i] = 0

        return d2

    @staticmethod
    def frenetFrame(arr):
        t_num = np.diff(arr, axis=0)
        t = (t_num/np.abs(t_num)).astype(int)

        b_num = np.cross(t[:-1], t[1:])
        b = b_num/np.abs(b_num)
        t = t[1:]

        n = np.cross(b, t)

        frameArr = np.concatenate((t, n, b), axis=1).T
        fArr = frameArr.reshape((3, 3, -1), order='F')
        return fArr

    def qtc_3D(self, k, l, thresh, q3, q4, q5):
        fFrameK = self.frenetFrame(k)
        fFrameL = self.frenetFrame(l)
        alpArr = np.zeros(q3.shape)
        betArr = np.zeros(q3.shape)
        gamArr = np.zeros(q3.shape)

        for g in range(fFrameK.shape[2]):
            fKinv = np.linalg.pinv(fFrameK[:, :, g])
            R = np.dot(fFrameL[:, :, g], fKinv)

            alpha = np.arctan(R[1, 0]/R[0, 0])
            den = np.sqrt(pow(R[2, 1], 2) + pow(R[2, 2], 2))

            beta = np.arctan(-R[2, 0]/den)
            gamma = np.arctan(R[2, 1]/R[2, 2])

            # threshold angles
            if np.abs(alpha) < thresh or math.isnan(alpha):
                alpha = 0
            if np.abs(beta) < thresh or math.isnan(beta):
                beta = 0
            if np.abs(gamma) < thresh or math.isnan(gamma):
                gamma = 0
            alpArr[g] = alpha
            betArr[g] = beta
            gamArr[g] = gamma

            q3[g] = np.sign(alpha)
            q4[g] = np.sign(beta)
            q5[g] = np.sign(gamma)

        return [alpArr, betArr, gamArr]

    @staticmethod
    def checkQTC(qtcArr):
        # array of shape x by 11 with 0 to 4 of 11 being the ones that unique must apply to
        tArr = copy.deepcopy(qtcArr)
        for i in range(1, tArr.shape[0]):
            if i != 0:
                currRow = tArr[i, :]
                Mod = False
                for q in range(tArr.shape[1]):
                    r = tArr[i, q] - tArr[i-1, q]
                    if r == -2 or r == 2:
                        Mod = True
                        currRow[q] = 0
                if Mod:
                    qtcArr = np.insert(qtcArr, i, currRow, axis=0)
                    # Mod = False
        return qtcArr

    @staticmethod
    def uniqueQTC(currQTC):
        # array of shape x by 11 with 0 to 4 of 11 being the ones that unique must apply to
        i = 0
        del1 = False
        del2 = False

        while i < currQTC.shape[0] - 2:
            if i != 0:
                if np.array_equal(currQTC[i, :5], currQTC[i-1, :5]):
                    del1 = True
                if np.array_equal(currQTC[i+1, :5], currQTC[i-1, :5]):
                    del2 = True

            if del1 and not del2:
                currQTC = np.delete(currQTC, i, 0)
            elif del2:
                currQTC = np.delete(currQTC, i, 0)
                currQTC = np.delete(currQTC, i+1, 0)

            i += 1
        return currQTC

    @staticmethod
    def filterMovement(ts, thresh):
        lastMove = 0
        # stillThere = True

        for timestep in range(1, ts.shape[0]):
            g = np.sqrt(np.sum(np.square(ts[timestep, :]-ts[lastMove, :])))
            if g >= thresh:
                lastMove = timestep
                # stillThere = False
                # print str(g).ljust(20) + ' shift'
            else:
                # print str(g).ljust(20) + ' still there'
                ts[timestep] = ts[lastMove]

                # if(stillThere):
                #     print 'delete'
                #     #np.delete(ts, timestep, axis = 0)
                # else:
                #     print 'no delete'
                #     stillThere = True

    @staticmethod
    def removeStationary(inputArr, labelRemove, indsToCheck):
        i = 0
        while i < inputArr.shape[0]:
            logic = True
            for j in indsToCheck:
                if inputArr[i][j] == labelRemove:
                    logic = logic and True
                else:
                    logic = logic and False

            if logic:  # if labelRemove was present in all indsToCheck remove row
                inputArr = np.delete(inputArr, i, 0)
            else:
                i += 1
        return inputArr

    # def formatFeatures(self, inputArr, idx):
    #     vec = np.zeros((len(humanStaticLabels)+1))
    #     for i in range(len(inputArr)):
    #         vec[inputArr[i][idx]] += 1
    #     vec[-1] = len(inputArr)
    #     return vec

    def sequenceConfig(self, verbose=False):
        self.configProcessing()
        self.verbose = verbose

    def messageChecker(self, dataMessage, mode):

        goAhead = True
        if mode == 'testing':
            t = dataMessage.replace('(', '').replace(')', '').split(' ')[4:]
        elif mode == 'live':
            t = dataMessage.replace('(', '').replace(')', '').split(' ')[2:]
        else:
            print 'Non-existing mode. Choose either live or read'
            t = []

        if len(t) > 45:
            for i in range(self.numJoints):
                    a = i*4
                    goAhead = goAhead and type(t[a]) == str
                    goAhead = goAhead and float(t[a+1]) is not None
                    goAhead = goAhead and float(t[a+2]) is not None
                    goAhead = goAhead and float(t[a+3]) is not None

            currIdx = (self.numJoints*4 - 1)
            numObjs = (len(t) - currIdx)/5

            for i in range(numObjs):
                a = currIdx + 1 + (i*5)
                goAhead = goAhead and type(t[a]) == str
                goAhead = goAhead and float(t[a+1]) is not None
                goAhead = goAhead and float(t[a+2]) is not None
                goAhead = goAhead and float(t[a+3]) is not None
        else:
            goAhead = False

        return [t, goAhead]

    def sequenceProcessing(self, modelList, dataMessage, mode='live', verbose=True):
        classification = None
        sentence = ''
        combinationList = []
        combinationKeys = []

        [t, goAhead] = self.messageChecker(dataMessage, mode)

        # TODO check data message for None
        if goAhead:
            # extract data parts
            for i in range(self.numJoints):
                a = i * 4
                if t[a] == 'shoulderCenter':
                    t[a] = 'chest'

                self.data[t[a]] = (np.array([float(t[a + 1]), float(t[a + 2]), float(t[a + 3])]))
                if t[a] not in self.jointsList:
                    self.jointsList.append(t[a])

            currIdx = (self.numJoints * 4 - 1)
            numObjs = (len(t) - currIdx) / 5

            for i in range(numObjs):
                a = currIdx + 1 + (i * 5)
                self.data[t[a]] = np.array([float(t[a + 1]), float(t[a + 2]), float(t[a + 3])])
                if t[a] not in self.objectsList:
                    self.objectsList.append(t[a])

            # check contact of either hand with either object
            # generate list of combinations of hands and objects to check for contact

            for i in self.objectsList[1:]:
                combinationList.append(['handLeft', i])
                combinationList.append(['handRight', i])

                combinationKeys.append(','.join(combinationList[-2]))
                combinationKeys.append(','.join(combinationList[-1]))

            Pk = None
            Pl = None
            for i in range(len(combinationList)):
                if combinationKeys[i] not in self.data:
                    if self.verbose:
                        print 'add item', combinationKeys[i]
                    self.data[combinationKeys[i]] = {'Pk': [None], 'Pl': [None], 'prevContact': False,
                                                          'currContact': False, 'd': [None]}

                if Pk is None:
                    Pk = self.data[combinationList[i][0]].T
                    Pl = self.data[combinationList[i][1]].T
                else:
                    Pk = np.vstack((Pk, self.data[combinationList[i][0]].T))
                    Pl = np.vstack((Pl, self.data[combinationList[i][1]].T))

            d = self.distEuc(Pk, Pl)  # TODO change to self movement

            for i in range(len(combinationList)):
                if d[i] < self.contactThreshold:
                    self.data[combinationKeys[i]]['currContact'] = True
                else:
                    self.data[combinationKeys[i]]['currContact'] = False

                if self.data[combinationKeys[i]]['currContact']:
                    if self.data[combinationKeys[i]]['prevContact']:
                        self.data[combinationKeys[i]]['Pk'].append(Pk[i])
                        self.data[combinationKeys[i]]['Pl'].append(Pl[i])
                        # if self.verbose:
                        #     print i, 'Append data', combinationList[i]
                    else:
                        self.data[combinationKeys[i]]['actionOccuring'] = True
                        if self.verbose:
                            print i, 'Contact between', combinationList[i], 'Action started'
                else:
                    if self.data[combinationKeys[i]]['prevContact']:
                        self.data[combinationKeys[i]]['actionOccuring'] = False
                        self.data[combinationKeys[i]]['actionLen'] = len(self.data[combinationKeys[i]]['Pk'])

                        if self.data[combinationKeys[i]]['actionLen'] > 10:
                            if self.verbose:
                                print i, 'Action stopped.', 'Len =', self.data[combinationKeys[i]]['actionLen']
                                print
                            # processing the action
                            tempQTC = self.extractFeatures(Pk, Pl)
                            tempQTC = self.chooseFeatures(tempQTC)
                            # if self.verbose:
                            #     print
                            ytest = tempQTC[None, :]
                            # change this to use SAMTesting for segments
                            [label, val] = SAMTesting.testSegment(modelList, ytest, verbose, visualiseInfo=None)
                            classification = label.split('_')[0]
                            if classification == 'unknown':
                                sentence = "You did an " + label + " action on the " + \
                                           str(combinationList[i][1]) + " with your " + \
                                           str(combinationList[i][0]).replace('hand', '') + ' hand'
                            else:
                                sentence = "You " + label.split('_')[0] + "ed the " + str(combinationList[i][1]) +\
                                           " with your " + str(combinationList[i][0]).replace('hand', '') + ' hand'
                            print sentence
                            print '------------------------------------------------------'

                        else:
                            if self.verbose:
                                print i, 'Action stopped.', 'Len =', self.data[combinationKeys[i]][
                                    'actionLen'], 'Action too short'
                            else:
                                'Action too short'

                        self.data[combinationKeys[i]]['Pk'] = [None]
                        self.data[combinationKeys[i]]['Pl'] = [None]
                    else:
                        pass
                        # if self.verbose:
                        #     print i, combinationList[i], 'x'

                self.data[combinationKeys[i]]['prevContact'] = self.data[combinationKeys[i]]['currContact']
            # if self.verbose:
            #     print

            if mode == 'testing':
                return classification
            else:
                return sentence
        else:
            print 'Incorrect message received'
            return None

    @staticmethod
    def testSequence(modelList):
        thisModel = modelList[0]
        off1 = 26
        off2 = 6
        off3 = 20
        lastActualAction = [None] * 2
        # liveLabels = []
        liveLabels = copy.deepcopy(thisModel.textLabels)
        liveLabels.append('None_object')
        liveLabels.append('unknown_object')
        cmSize = len(liveLabels)
        labelTotals = np.zeros((cmSize, 1))
        confMatrix = np.zeros((cmSize, cmSize))

        print '     ', 'Classification'.center(off1), 'Label'.center(off2), 'Verdict'.center(off3)
        i = 0
        for j in range(len(thisModel.dataLogList)):
            xdataFile = open(thisModel.dataLogList[j], 'r')
            for i, l in enumerate(xdataFile):
                pass
            xlenFile = i + 1
            xdataFile.close()

            xDataFile = open(thisModel.dataLogList[j], 'r')
            xLogFile = open(thisModel.labelsLogList[j], 'r')

            for i in range(xlenFile):
                dataMessage = xDataFile.readline()
                actualLabel = xLogFile.readline().replace('(', '').replace(')', '').split(' ')[-1][:-1]
                classification = thisModel.sequenceProcessing(modelList, dataMessage, 'testing')
                proper = False

                for k in thisModel.paramsDict['actionsAllowedList']:
                    if k in actualLabel:
                        proper = proper or True
                    else:
                        proper = proper or False

                if proper:
                    if lastActualAction[0] != actualLabel.split('_')[0]:
                        lastActualAction[1] = lastActualAction[0]
                        lastActualAction[0] = actualLabel.split('_')[0]

                if classification is not None:
                    print str(i).ljust(4), classification.center(off1), str(lastActualAction[0]).center(off2),
                    confMatrix[liveLabels.index(str(lastActualAction[0]) + '_object'),
                               liveLabels.index(classification + '_object')] += 1
                    labelTotals[liveLabels.index(str(lastActualAction[0]) + '_object')] += 1
                    if classification == lastActualAction[0]:
                        print 'Correct'.center(off3)
                    else:
                        print 'Wrong'.center(off3)

        return SAMTesting.calculateData(liveLabels, confMatrix)

    def processLiveData(self, dataList, thisModel):
        # print 'process live data'
        classification = self.sequenceProcessing(thisModel, dataList[-1].toString(), mode='live', verbose=True)
        dataList = []
        if classification == '':
            classification = None
        return classification
