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

        self.additionalParametersList = ['listOfVectorsToClassify', 'handsCombined', 'featureSequence', 'itemsPerJoint',
                                         'segTrainConf', 'segTrainPerc', 'segTestConf', 'segTestPerc', 'seqTestConf',
                                         'seqTestPerc']

    def loadParameters(self, parser, trainName):

        if parser.has_option(trainName, 'includeParts'):
            self.paramsDict['includeParts'] = parser.get(trainName, 'includeParts').split(',')
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
            self.paramsDict['windowSize'] = 3

        if parser.has_option(trainName, 'combineHands'):
            self.paramsDict['combineHands'] = parser.get(trainName, 'combineHands') == 'True'
        else:
            self.paramsDict['combineHands'] = False

        if parser.has_option(trainName, 'reduce'):
            self.paramsDict['reduce'] = parser.get(trainName, 'reduce') == 'True'
        else:
            self.paramsDict['reduce'] = False

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

    def readData(self, root_data_dir, participant_index, *args, **kw):
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
            lenDataFile = i + 1

            # number of lines in labelFile
            for i, l in enumerate(labelFile):
                pass
            lenLabelFile = i + 1
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
                        a = i * 4
                        if t[a] == 'shoulderCenter':
                            t[a] = 'chest'

                        if firstPass:
                            data[t[a]] = [None] * numFiles
                            data[t[a]][k] = (np.array([float(t[a + 1]), float(t[a + 2]), float(t[a + 3])]))
                            jointsList.append(t[a])
                        else:
                            arr = np.array([float(t[a + 1]), float(t[a + 2]), float(t[a + 3])])
                            if data[t[a]][k] is not None:
                                data[t[a]][k] = np.vstack((data[t[a]][k], arr))
                            else:
                                data[t[a]][k] = arr

                    currIdx = (self.numJoints * 4 - 1)
                    numObjs = (len(t) - currIdx) / 5

                    for i in range(numObjs):
                        a = currIdx + 1 + (i * 5)
                        if t[a] in data:
                            arr = np.array([float(t[a + 1]), float(t[a + 2]), float(t[a + 3])])
                            if data[t[a]][k] is not None:
                                data[t[a]][k] = np.vstack((data[t[a]][k], arr))
                            else:
                                data[t[a]][k] = arr
                        else:
                            data[t[a]] = [None] * (numFiles + 1)
                            data[t[a]][k] = np.array([float(t[a + 1]), float(t[a + 2]), float(t[a + 3])])
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

        print
        # convert text labels into numbers while keeping data from different log files separated
        labelNumsList = []
        for j in range(len(dataLogList)):
            labelNumsList.append(None)
            for n, k in enumerate(labelsList[j]):
                res = [m for m, l in enumerate(labels) if l == k]
                if n == 0:
                    labelNumsList[j] = np.array(res)
                else:
                    labelNumsList[j] = np.vstack([labelNumsList[j], res])
        print labelNumsList[0].shape
        print data['head'][0].shape

        # convert data and number labels into windows.
        # data is still in the form of a dictionary with the joints/objects as keys of the dict
        # Text labels contained in labels
        print
        data2NumLabels = None
        data2 = dict()
        for num, key in enumerate(data):
            data2[key] = None
            for j in range(len(dataLogList)):
                xx, yy = utils.transformTimeSeriesToSeq(data[key][j], self.paramsDict['windowSize'],
                                                        self.paramsDict['normaliseWindow'], self.paramsDict['reduce'])
                uu, tmp = utils.transformTimeSeriesToSeq(labelNumsList[j], self.paramsDict['windowSize'], False, False)
                if j == 0:
                    data2[key] = xx
                    data2NumLabels = uu
                else:
                    data2[key] = np.vstack([data2[key], xx])
                    data2NumLabels = np.vstack([data2NumLabels, uu])

        print data2['head'].shape
        print data2NumLabels.shape
        print
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

        labels = labels + ['transition']
        print len(data2Labels)

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
        self.featureSequence = ['object']
        print
        combinedObjs = None
        if combineObjects:
            for j in range(len(data2Labels)):
                # print data2Labels[j]
                if len(data2Labels[j].split('_')) > 2:
                    idxBase = objectDict[data2Labels[j].split('_')[2]] * itemsPerJoint
                else:
                    idxBase = objectDict[objectDict.keys()[0]] * itemsPerJoint

                if combinedObjs is None:
                    combinedObjs = dataVecAll[j, idxBase:idxBase + itemsPerJoint]
                else:
                    combinedObjs = np.vstack([combinedObjs, dataVecAll[j, idxBase:idxBase + itemsPerJoint]])

        print dataVecAll.shape
        print combinedObjs.shape

        print
        # it is now time to combine hands if multiple exist
        combinedHands = None
        if combineHands and self.paramsDict['combineHands']:
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
            if item not in handDict and item not in objectDict:
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
            data2ShortLabels.append('_'.join(j.split('_')[:2]))

        self.data2Labels = copy.deepcopy(data2ShortLabels)
        # remove labels which will not be trained
        listToDelete = []
        for n in reversed(range(len(data2Labels))):
            if len([j for j in self.paramsDict['actionsAllowedList'] if j in data2Labels[n]]) == 0 or \
                            'no' in data2Labels[n]:
                listToDelete.append(n)

        for j in listToDelete:
            dataVecReq = np.delete(dataVecReq, j, 0)
            data2ShortLabels.pop(j)

        self.Y = dataVecReq
        self.L = data2ShortLabels
        # print '\n'.join(data2Labels)
        print self.Y.shape
        print len(self.L)

        # now that all joints are in the form of a window, time to create
        # all possible vectors to classify
        self.listOfVectorsToClassify = []

        for j in self.featureSequence:
            if j == 'object':
                for k in objectsList:
                    if k != 'partner':
                        self.listOfVectorsToClassify.append([k])

            elif 'hand' in j:
                if self.handsCombined:
                    a = copy.deepcopy(self.listOfVectorsToClassify)
                    b = copy.deepcopy(self.listOfVectorsToClassify)
                    for l, m in enumerate(self.listOfVectorsToClassify):
                        a[l].append('handLeft')
                        b[l].append('handRight')
                        self.listOfVectorsToClassify = a + b

                else:
                    for l, m in enumerate(self.listOfVectorsToClassify):
                        self.listOfVectorsToClassify[l].append(j)

            else:
                for l, m in enumerate(self.listOfVectorsToClassify):
                    self.listOfVectorsToClassify[l].append(j)

        for j in self.listOfVectorsToClassify:
            print j

    def testPerformance(self, testModel, Yall, Lall, YtestAll, LtestAll, verbose):

        yTrainingData = SAMTesting.formatDataFunc(Yall)
        [self.segTrainConf, self.segTrainPerc] = SAMTesting.testSegments(testModel, yTrainingData, Lall, verbose)

        yTrainingData = SAMTesting.formatDataFunc(YtestAll)
        [self.segTestConf, self.segTestPerc] = SAMTesting.testSegments(testModel, yTrainingData, LtestAll, verbose)

        yTrainingData = SAMTesting.formatDataFunc(self.dataVec)
        [self.seqTestConf, self.seqTestPerc] = SAMTesting.testSegments(testModel, yTrainingData, self.data2Labels, verbose)

        return self.seqTestConf
        # return numpy.ones([3, 3])


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

    def processLiveData(self, dataList, thisModel):
        # dataList is list of yarp bottles
        mode = 'live'
        sentence = []

        if len(dataList) == self.paramsDict['windowSize']:
            dataStrings = []
            for j in dataList:
                [t, goAhead] = self.messageChecker(j.toString(), mode)
                if goAhead:
                    dataStrings.append(t)

            data = dict()
            print 'going ahead'
            if len(dataStrings) >= self.paramsDict['windowSize']:
                jointsList = []
                objectsList = []
                # extract data parts
                for t in dataStrings:
                    for i in range(self.numJoints):
                        a = i * 4
                        if t[a] == 'shoulderCenter':
                            t[a] = 'chest'

                        if t[a] not in jointsList:
                            jointsList.append(t[a])
                            data[t[a]] = (np.array([float(t[a + 1]), float(t[a + 2]), float(t[a + 3])]))
                        else:
                            data[t[a]] = np.vstack(
                                [data[t[a]], np.array([float(t[a + 1]), float(t[a + 2]), float(t[a + 3])])])

                    currIdx = (self.numJoints * 4 - 1)
                    numObjs = (len(t) - currIdx) / 5

                    for i in range(numObjs):
                        a = currIdx + 1 + (i * 5)

                        if t[a] not in objectsList:
                            objectsList.append(t[a])
                            data[t[a]] = np.array([float(t[a + 1]), float(t[a + 2]), float(t[a + 3])])
                        else:
                            data[t[a]] = np.vstack(
                                [data[t[a]], np.array([float(t[a + 1]), float(t[a + 2]), float(t[a + 3])])])

                # convert arrays in dict data into windows
                for j in data:
                    data[j], yy = utils.transformTimeSeriesToSeq(data[j], self.paramsDict['windowSize'],
                                                                 self.paramsDict['normaliseWindow'],
                                                                 self.paramsDict['reduce'], True)

                for j in self.listOfVectorsToClassify:
                    v = []
                    for k in j:
                        v.append(data[k])
                    vec = np.hstack(v)
                    [label, val] = SAMTesting.testSegment(thisModel, vec, True, visualiseInfo=None)
                    classification = label.split('_')[0]
                    if classification == 'unknown':
                        sentence.append("You did an " + label + " action on the " + j[0])
                    else:
                        sentence.append("You " + label.split('_')[0] + " the " + j[0])

                    if len(j) > 1:
                        sentence[-1] += " with your " + j[1].replace('hand', '') + ' hand'
                    print sentence[-1]
                    print '------------------------------------------------------'
                    if classification == 'unknown':
                        sentence.pop(-1)

                if len(sentence) > 1:
                    return str(sentence)
                else:
                    return 'None'
            else:
                print 'Some incorrect messages received'
                return None
        else:
            return None
