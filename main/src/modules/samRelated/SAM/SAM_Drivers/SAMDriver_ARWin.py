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
from os import listdir
from os.path import isfile, join
import copy
import numpy
import numpy as np
from SAM.SAM_Core import SAMDriver
from SAM.SAM_Core import SAMTesting
from SAM.SAM_Core import SAM_utils as utils
np.set_printoptions(threshold=numpy.nan)


class SAMDriver_ARWin(SAMDriver):

    def __init__(self):
        SAMDriver.__init__(self)
        self.data = dict()
        self.numJoints = 9
        self.dataLogList = []
        self.labelsLogList = []
        self.itemsPerJoint = None
        self.featureSequence = None
        self.handsCombined = None
        self.data2Labels = None
        self.dataVec = None
        self.allDataDict = None
        self.listOfVectorsToClassify = None
        self.seqTestConf = None
        self.seqTestPerc = None
        self.additionalParametersList = ['listOfVectorsToClassify', 'handsCombined', 'featureSequence', 'itemsPerJoint',
                                         'segTrainConf', 'segTrainPerc', 'segTestConf', 'segTestPerc', 'seqTestConf',
                                         'seqTestPerc']

    def loadParameters(self, parser, trainName):

        if parser.has_option(trainName, 'includeParts'):
            self.paramsDict['includeParts'] = parser.get(trainName, 'includeParts').split(',')
            self.paramsDict['includeParts'] = [j for j in self.paramsDict['includeParts'] if j != '']
        else:
            self.paramsDict['includeParts'] = ['object']

        if parser.has_option(trainName, 'actionsAllowedList'):
            self.paramsDict['actionsAllowedList'] = parser.get(trainName, 'actionsAllowedList').split(',')
        else:
            self.paramsDict['actionsAllowedList'] = ['lift_object', 'pull_object', 'push_object', 'drop_object',
                                                     'carry_object']

        if parser.has_option(trainName, 'windowSize'):
            self.paramsDict['windowSize'] = int(parser.get(trainName, 'windowSize'))
        else:
            self.paramsDict['windowSize'] = 5

        if parser.has_option(trainName, 'windowOffset'):
            self.paramsDict['windowOffset'] = int(parser.get(trainName, 'windowOffset'))
        else:
            self.paramsDict['windowOffset'] = 2

        if parser.has_option(trainName, 'moveThresh'):
            self.paramsDict['moveThresh'] = float(parser.get(trainName, 'moveThresh'))
        else:
            self.paramsDict['moveThresh'] = 0.01

        if parser.has_option(trainName, 'binWidth'):
            self.paramsDict['binWidth'] = float(parser.get(trainName, 'binWidth'))
        else:
            self.paramsDict['binWidth'] = 0.001

        if parser.has_option(trainName, 'method'):
            self.paramsDict['method'] = parser.get(trainName, 'method')
        else:
            self.paramsDict['method'] = 'sumProb'

        if parser.has_option(trainName, 'combineHands'):
            self.paramsDict['combineHands'] = parser.get(trainName, 'combineHands') == 'True'
        else:
            self.paramsDict['combineHands'] = False

        if parser.has_option(trainName, 'thresholdMovement'):
            self.paramsDict['thresholdMovement'] = parser.get(trainName, 'thresholdMovement') == 'True'
        else:
            self.paramsDict['thresholdMovement'] = False

        if parser.has_option(trainName, 'sepRL'):
            self.paramsDict['sepRL'] = parser.get(trainName, 'sepRL') == 'True'
        else:
            self.paramsDict['sepRL'] = False

        if parser.has_option(trainName, 'filterData'):
            self.paramsDict['filterData'] = parser.get(trainName, 'filterData') == 'True'
        else:
            self.paramsDict['filterData'] = False

        if parser.has_option(trainName, 'filterWindow'):
            self.paramsDict['filterWindow'] = int(parser.get(trainName, 'filterWindow'))
        else:
            self.paramsDict['filterWindow'] = 5

        if parser.has_option(trainName, 'components'):
            self.paramsDict['components'] = parser.get(trainName, 'components').split(',')
        else:
            self.paramsDict['components'] = ['pos']

        if parser.has_option(trainName, 'reduce'):
            self.paramsDict['reduce'] = parser.get(trainName, 'reduce') == 'True'
        else:
            self.paramsDict['reduce'] = False

        if parser.has_option(trainName, 'flip'):
            self.paramsDict['flip'] = parser.get(trainName, 'flip') == 'True'
        else:
            self.paramsDict['flip'] = False

        if parser.has_option(trainName, 'normaliseWindow'):
            self.paramsDict['normaliseWindow'] = parser.get(trainName, 'normaliseWindow') == 'True'
        else:
            self.paramsDict['normaliseWindow'] = False

    def saveParameters(self):
        for j in self.additionalParametersList:
            commandString = 'self.paramsDict[\'' + j + '\'] = self.' + j
            try:
                print commandString
                exec commandString
            except:
                pass

    def testPerformance(self, testModel, Yall, Lall, YtestAll, LtestAll, verbose):

        yTrainingData = SAMTesting.formatDataFunc(Yall)
        [self.segTrainConf, self.segTrainPerc] = SAMTesting.testSegments(testModel, yTrainingData,
                                                                         Lall, verbose, 'Training')

        yTrainingData = SAMTesting.formatDataFunc(YtestAll)
        [self.segTestConf, self.segTestPerc] = SAMTesting.testSegments(testModel, yTrainingData,
                                                                       LtestAll, verbose, 'Testing')

        yTrainingData = SAMTesting.formatDataFunc(self.dataVec)
        [self.seqTestConf, self.seqTestPerc] = SAMTesting.testSegments(testModel, yTrainingData,
                                                                       self.data2Labels, verbose, 'All')

        return self.seqTestConf

    def diskDataToLiveData(self, root_data_dir):
        onlyfiles = [f for f in listdir(root_data_dir) if isfile(join(root_data_dir, f))]
        dataLogList = [f for f in onlyfiles if 'data' in f]
        dataLogList.sort()
        labelsLogList = [f for f in onlyfiles if 'label' in f]
        labelsLogList.sort()
        rawLabelList = []
        rawDataList = []

        print 'loading data from files'
        self.rawTextData = []
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
            lenDataFile = i + 1

            # number of lines in labelFile
            for i, l in enumerate(labelFile):
                pass
            lenLabelFile = i + 1
            dataFile.close()
            labelFile.close()

            if lenLabelFile != lenDataFile:
                print str(dataLogList[k]) + ' will not be used because its length differs from ' + str(labelsLogList[k])
            else:
                dataFile = open(join(root_data_dir, dataLogList[k]), 'r')
                labelFile = open(join(root_data_dir, labelsLogList[k]), 'r')
                windows = lenDataFile // self.paramsDict['windowSize']

                for curr in range(windows*self.paramsDict['windowSize']):
                    line = dataFile.readline()
                    labelLine = labelFile.readline()
                    self.rawTextData.append(line)
                    # check data line
                    t, goAhead = self.messageChecker(line, 'testing')

                    # check label line
                    v = labelLine.split(' ')[2].replace('\n', '').replace('(', '').replace(')', '')
                    if v == '':
                        v = 'unknown'

                    if goAhead:
                        rawDataList.append(t)
                        rawLabelList.append(v)
                    else:
                        print 'error'

                dataFile.close()
                labelFile.close()

        return rawDataList, rawLabelList

    def convertToDict(self, rawData, mode, verbose):
        data = dict()
        firstPass = True
        jointsList = []
        objectsList = []

        # print '*******************'
        # for j in self.paramsDict:
        #     print j, self.paramsDict[j]
        # print '*******************'

        for t in rawData:
            # parse skeleton data which has 9 sections by (x,y,z)
            for i in range(self.numJoints):
                a = i * 4
                # if t[a] == 'shoulderCenter':
                #     t[a] = 'chest'

                if firstPass:
                    data[t[a]] = [None]
                    data[t[a]] = (np.array([float(t[a + 1]), float(t[a + 2]), float(t[a + 3])]))
                    jointsList.append(t[a])
                else:
                    arr = np.array([float(t[a + 1]), float(t[a + 2]), float(t[a + 3])])
                    if data[t[a]] is not None:
                        data[t[a]] = np.vstack((data[t[a]], arr))
                    else:
                        data[t[a]] = arr

            currIdx = (self.numJoints * 4 - 1)
            numObjs = (len(t) - currIdx) / 5

            for i in range(numObjs):
                a = currIdx + 1 + (i * 5)
                if t[a] in data:
                    arr = np.array([float(t[a + 1]), float(t[a + 2]), float(t[a + 3])])
                    if data[t[a]] is not None:
                        data[t[a]] = np.vstack((data[t[a]], arr))
                    else:
                        data[t[a]] = arr
                else:
                    data[t[a]] = [None]
                    data[t[a]] = np.array([float(t[a + 1]), float(t[a + 2]), float(t[a + 3])])
                    objectsList.append(t[a])

            firstPass = False
        if verbose:
            print 'data has length = ' + str(len(data)) + ' joints'
            print 'each joint has an array of shape ' + str(data['head'].shape)

        if self.paramsDict['filterData'] or 'vel' in self.paramsDict['components'] or \
                                            'acc' in self.paramsDict['components']:
            if verbose:
                print 'Filtering data with hamming window of size', self.paramsDict['filterWindow']
            for j in data.keys():
                t1 = utils.smooth1D(data[j][:, 0], self.paramsDict['filterWindow'])
                t2 = utils.smooth1D(data[j][:, 1], self.paramsDict['filterWindow'])
                t3 = utils.smooth1D(data[j][:, 2], self.paramsDict['filterWindow'])
                data[j] = np.hstack([t1[:, None], t2[:, None], t3[:, None]])

        if verbose:
            print 'data has length = ' + str(len(data)) + ' joints'
            print 'each joint has an array of shape ' + str(data['head'].shape)
        # convert data and number labels into windows.
        # data is still in the form of a dictionary with the joints/objects as keys of the dict
        # Text labels contained in labels
        if verbose:
            print
        noY = mode != 'testing'
        if mode == 'testing':
            offset = self.paramsDict['windowOffset']
        else:
            offset = 1

        data2 = dict()
        printExplanation = True
        for num, key in enumerate(data):
            data2[key] = None
            xx, yy = utils.transformTimeSeriesToSeq(data[key], timeWindow=self.paramsDict['windowSize'],
                                                    offset=offset,
                                                    normalised=self.paramsDict['normaliseWindow'],
                                                    reduced=self.paramsDict['reduce'], noY=noY)

            if self.paramsDict['thresholdMovement'] or 'vel' in self.paramsDict['components'] or 'acc' in \
                    self.paramsDict['components']:
                winSize = xx.shape[1] / 3
                g = xx.size / winSize
                xxshape1 = xx.shape[0]
                xxshape2 = xx.shape[1]

                flatxx = xx.flatten()
                f = flatxx.reshape([g, winSize])
                xx = f.reshape([xxshape1, xxshape2])

                if self.paramsDict['thresholdMovement']:
                    if printExplanation and verbose:
                        print 'thresholding movement <', self.paramsDict['moveThresh']
                    ranges = np.ptp(f, axis=1)
                    a = ranges < self.paramsDict['moveThresh']
                    b = ranges > -self.paramsDict['moveThresh']
                    res = list(np.where(np.logical_and(a, b))[0])
                    if self.paramsDict['normaliseWindow']:
                        f[res] = 0
                    else:
                        for ll in res:
                            f[ll] = f[ll][0]

                if 'vel' in self.paramsDict['components']:
                    if printExplanation and verbose:
                        print 'Adding velocity to the feature vector'
                    xxvel = np.diff(f)
                    xxvel = xxvel.reshape([xxshape1, xxshape2 - 3])
                    xx = np.hstack([xx, xxvel])

                if 'acc' in self.paramsDict['components']:
                    if printExplanation and verbose:
                        print 'Adding acceleration to the feature vector'
                    xxacc = np.diff(f, n=2)
                    xxacc = xxacc.reshape([xxshape1, xxshape2 - 6])
                    xx = np.hstack([xx, xxacc])

            data2[key] = xx
            printExplanation = False

        if verbose:
            print 'data has length = ' + str(len(data2)) + ' joints'
            print 'each joint has an array of shape ' + str(data2['head'].shape)

        return data2, jointsList, objectsList

    def readData(self, root_data_dir, participant_index, *args, **kw):
        self.rawData, labelsList = self.diskDataToLiveData(root_data_dir)
        data2, jointsList, objectsList = self.convertToDict(self.rawData, 'testing', verbose=self.verbose)

        # extract a set of labels
        labels = list(set(labelsList))
        labels.sort()

        print
        # convert text labels into numbers 
        labelNumsList = None
        for n, k in enumerate(labelsList):
            res = [m for m, l in enumerate(labels) if l == k]
            if n == 0:
                labelNumsList = np.array(res)
            else:
                labelNumsList = np.vstack([labelNumsList, res])
        print 'shape of number labels:', labelNumsList.shape

        uu, tmp = utils.transformTimeSeriesToSeq(labelNumsList, self.paramsDict['windowSize'],
                                                 self.paramsDict['windowOffset'], False, False)
        data2NumLabels = uu
        print 'windowed number labels shape:', data2NumLabels.shape

        # now that labels are in windowed form it is time to
        # assign them a text label again that describes them
        # the rule here is if the window appertains to the same label,
        # that label is assigned otherwise it is labelled as transition
        data2Labels = []
        for j in data2NumLabels:
            numItems = list(set(j))
            if len(numItems) == 1:
                l = labels[int(numItems[0])]
                data2Labels.append(l)
            else:
                # Another way to do this would be to label it according to 75% majority
                # This would decrease the region size of the transition blocks
                # which are currently dependant on windowSize
                data2Labels.append('transition')

        print 'windowed data labels compressed:', len(data2Labels)

        print
        # create list of specific joints to be used

        jointsToUse = []
        objectDict = dict()
        handDict = dict()
        for j in self.paramsDict['includeParts']:
            if j == 'object':
                for k in objectsList:
                    if k != 'partner':
                        objectDict[k] = (len(jointsToUse))
                        jointsToUse.append(k)
            elif 'hand' in j:
                handDict[j] = (len(jointsToUse))
                jointsToUse.append(j)
            else:
                jointsToUse.append(j)

        combineObjects = len(objectDict) > 1

        combineHands = len(handDict) > 1

        print jointsToUse
        print objectDict
        print handDict

        # concatenate data for all joints in a single vector
        print
        dataVecAll = None
        for j in jointsToUse:
            if dataVecAll is None:
                dataVecAll = data2[j]
            else:
                dataVecAll = np.hstack([dataVecAll, data2[j]])
        itemsPerJoint = dataVecAll.shape[1] / len(jointsToUse)
        print dataVecAll.shape
        print itemsPerJoint
        self.itemsPerJoint = itemsPerJoint
        print

        # it is now time to combine objects if multiple exist
        # 
        self.featureSequence = ['object']
        print
        combinedObjs = None
        if combineObjects:
            print 'Combining Objects'
            for j in range(len(data2Labels)):
                #         print data2Labels[j]
                if len(data2Labels[j].split('_')) > 2:
                    idxBase = objectDict[data2Labels[j].split('_')[2]] * itemsPerJoint
                else:
                    idxBase = objectDict[objectDict.keys()[0]] * itemsPerJoint

                if combinedObjs is None:
                    combinedObjs = dataVecAll[j, idxBase:idxBase + itemsPerJoint]
                else:
                    combinedObjs = np.vstack([combinedObjs, dataVecAll[j, idxBase:idxBase + itemsPerJoint]])
            print combinedObjs.shape

        print dataVecAll.shape

        print
        # it is now time to combine hands if multiple exist
        combinedHands = None
        if combineHands and self.paramsDict['combineHands']:
            print 'Combining hands'
            self.handsCombined = True
            self.featureSequence.append('hand')
            for j in range(len(data2Labels)):
                if len(data2Labels[j].split('_')) > 2:
                    idxBase = handDict[data2Labels[j].split('_')[3] + data2Labels[j].split('_')[4].capitalize()] * \
                              itemsPerJoint
                else:
                    idxBase = handDict[handDict.keys()[0]] * itemsPerJoint

                if combinedHands is None:
                    combinedHands = dataVecAll[j, idxBase:idxBase + itemsPerJoint]
                else:
                    combinedHands = np.vstack([combinedHands, dataVecAll[j, idxBase:idxBase + itemsPerJoint]])
            print dataVecAll.shape
            print combinedHands.shape
        else:
            self.handsCombined = False

        dataVecReq = None

        if combinedHands is not None:
            dataVecReq = combinedHands

        if combinedObjs is not None:
            if dataVecReq is None:
                dataVecReq = combinedObjs
            else:
                dataVecReq = np.hstack([dataVecReq, combinedObjs])

        print jointsToUse
        for j, item in enumerate(jointsToUse):
            if self.handsCombined:
                if item not in handDict and item not in objectDict:
                    self.featureSequence.append(item)
                    idxBase = j * itemsPerJoint

                    if dataVecReq is None:
                        dataVecReq = dataVecAll[:, idxBase:idxBase + itemsPerJoint]
                    else:
                        dataVecReq = np.hstack([dataVecReq, dataVecAll[:, idxBase:idxBase + itemsPerJoint]])
            else:
                if item not in objectDict:
                    self.featureSequence.append(item)
                    idxBase = j * itemsPerJoint

                    if dataVecReq is None:
                        dataVecReq = dataVecAll[:, idxBase:idxBase + itemsPerJoint]
                    else:
                        dataVecReq = np.hstack([dataVecReq, dataVecAll[:, idxBase:idxBase + itemsPerJoint]])

        print dataVecReq.shape
        print len(data2Labels)
        print
        self.dataVec = copy.deepcopy(dataVecReq)

        data2ShortLabels = []
        for j in data2Labels:
            splitLabel = j.split('_')
            slabel = ('_'.join(splitLabel[:2]))

            if splitLabel[0] == 'push' or splitLabel[0] == 'pull':
                if splitLabel[-1] == 'no':
                    add = splitLabel[-2]
                else:
                    add = splitLabel[-1]

                if add == 'left' and self.paramsDict['flip']:
                    if splitLabel[0] == 'push':
                        splitLabel[0] = 'pull'
                    else:
                        splitLabel[0] = 'push'
                    slabel = ('_'.join(splitLabel[:2]))

                if self.paramsDict['sepRL']:
                    slabel += '_' + add

            data2ShortLabels.append(slabel)

        self.data2Labels = copy.deepcopy(data2ShortLabels)

        if self.paramsDict['sepRL']:
            if 'pull_object' in self.paramsDict['actionsAllowedList']:
                self.paramsDict['actionsAllowedList'].index('pull_object') == 'pull_object_right'
                self.paramsDict['actionsAllowedList'].append('pull_object_left')

            if 'push_object' in self.paramsDict['actionsAllowedList']:
                self.paramsDict['actionsAllowedList'].index('push_object') == 'push_object_right'
                self.paramsDict['actionsAllowedList'].append('push_object_left')

        # remove labels which will not be trained
        listToDelete = []
        for n in reversed(range(len(data2Labels))):
            if len([j for j in self.paramsDict['actionsAllowedList'] if j in data2Labels[n]]) == 0 or \
                            'no' in data2Labels[n]:
                listToDelete.append(n)

        dataVecReq = np.delete(dataVecReq, listToDelete, axis=0)
        npdata2ShortLabels = np.asarray(data2ShortLabels)
        npdata2ShortLabels = np.delete(npdata2ShortLabels, listToDelete, axis=0)
        # find left hand push and pull and label as pull and push respectively
        data2ShortLabels = np.ndarray.tolist(npdata2ShortLabels)

        self.Y = dataVecReq
        self.L = data2ShortLabels
        # print '\n'.join(data2Labels)
        print self.Y.shape
        print len(self.L)

        # now that all joints are in the form of a window, time to create
        # all possible vectors to classify

        self.allDataDict = dict()
        self.allDataDict['Y'] = self.dataVec
        self.allDataDict['L'] = self.data2Labels

        listOfVectorsToClassify = self.listOfClassificationVectors(self.featureSequence, objectsList)
        for j in listOfVectorsToClassify:
            print j

    def listOfClassificationVectors(self, featureSequence, objectsList):
        listOfVectorsToClassify = []
        for j in featureSequence:
            if j == 'object':
                for k in objectsList:
                    if k != 'partner':
                        listOfVectorsToClassify.append([k])

            elif 'hand' in j:
                if self.handsCombined:
                    a = copy.deepcopy(listOfVectorsToClassify)
                    b = copy.deepcopy(listOfVectorsToClassify)
                    for l, m in enumerate(listOfVectorsToClassify):
                        a[l].append('handLeft')
                        b[l].append('handRight')
                        listOfVectorsToClassify = a + b

                else:
                    for l, m in enumerate(listOfVectorsToClassify):
                        listOfVectorsToClassify[l].append(j)

            else:
                for l, m in enumerate(listOfVectorsToClassify):
                    listOfVectorsToClassify[l].append(j)

        return listOfVectorsToClassify

    def messageChecker(self, dataMessage, mode):
        goAhead = True
        if mode == 'testing':
            t = dataMessage.replace('(', '').replace(')', '').split(' ')[4:-1]
        elif mode == 'live':
            t = dataMessage.replace('(', '').replace(')', '').split(' ')[2:-1]
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

    def processLiveData(self, dataList, thisModel, verbose=False, returnUnknown=False, printClass=True):
        # dataList is list of yarp bottles
        mode = 'live'
        sentence = []
        classifs = []
        vecList = []

        if len(dataList) == self.paramsDict['windowSize']:
            dataStrings = []
            for j in range(len(dataList)):
                [t, goAhead] = self.messageChecker(dataList[j].toString(), mode)
                if goAhead:
                    dataStrings.append(t)

            if len(dataStrings) == self.paramsDict['windowSize']:
                data, jointsList, objectsList = self.convertToDict(dataStrings, mode=mode, verbose=False)
                listOfVectorsToClassify = self.listOfClassificationVectors(self.featureSequence, objectsList)

                for j in listOfVectorsToClassify:
                    v = []
                    for k in j:
                        v.append(data[k])
                    vec = np.hstack(v)
                    vecList.append(vec)
                    [label, val] = SAMTesting.testSegment(thisModel, vec, verbose, visualiseInfo=None,
                                                          optimise=thisModel[0].optimiseRecall)
                    classification = label.split('_')[0]
                    classifs.append(classification)
                    if self.paramsDict['flip'] and 'handLeft' in j:
                        if label == 'push':
                            label = 'pull'
                        elif label == 'pull':
                            label = 'push'

                    if classification == 'unknown':
                        sentence.append("You did an " + label + " action with " + str(j))
                    else:
                        sentence.append("You did a " + label + " with " + str(j))  # + " the " + j[0])

                    # if len(j) > 1:
                    #     sentence[-1] += " with your " + j[1].replace('hand', '') + ' hand'

                    if classification == 'unknown' and not returnUnknown:
                        sentence.pop(-1)
                    elif printClass:
                        print sentence[-1]
                    if printClass:
                        print '------------------------------------------------------'

                del dataList[:self.paramsDict['windowOffset']]

                if len(sentence) > 1:
                    # return [str(sentence), data, classifs, vecList]
                    return str(sentence), dataList
                else:
                    # return ['None', data, classifs, vecList]
                    return 'None', dataList
            else:
                print 'Some incorrect messages received'
                return 'None', None
        else:
            return None, None
