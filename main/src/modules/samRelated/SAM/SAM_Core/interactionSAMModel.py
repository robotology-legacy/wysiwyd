#!/usr/bin/env python

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import sys
import time
from ConfigParser import SafeConfigParser
from SAM.SAM_Core import SAMDriver as Driver
from SAM.SAM_Core.SAM_utils import initialiseModels, timeout
import readline
import warnings
import numpy as np
import yarp
import logging
from os.path import join
import os
from operator import itemgetter
import thread
warnings.simplefilter("ignore")
np.set_printoptions(precision=2)


class interactionSAMModel(yarp.RFModule):

    def __init__(self):
        yarp.RFModule.__init__(self)
        self.mm = None
        self.dataPath = None
        self.configPath = None
        self.modelPath = None
        self.driverName = ''
        self.model_type = None
        self.model_mode = None
        self.textLabels = None
        self.classifiers = None
        self.classif_thresh = None
        self.verbose = None
        self.Quser = None
        self.listOfModels = None
        self.portsList = []
        self.svPort = None
        self.labelPort = None
        self.instancePort = None
        self.callSignList = []
        self.inputBottle = yarp.Bottle()
        self.outputBottle = yarp.Bottle()
        self.portNameList = []

        self.rpcConnected = False
        self.dataInConnected = False
        self.dataOutConnected = False
        self.collectionMethod = ''
        self.bufferSize = None

        self.falseCount = 0
        self.noDataCount = 0
        self.inputType = None
        self.outputType = None
        self.errorRate = 50
        self.dataList = []
        self.classificationList = []
        self.closeFlag = False
        self.instancePortName = ''
        self.labelPortName = ''
        self.verboseSetting = False
        self.exitFlag = False
        self.recordingFile = ''
        self.additionalInfoDict = dict()
        self.modelLoaded = False
        self.attentionMode = 'continue'
        self.baseLogFileName = 'interactionErrorLog'
        self.windowedMode = True
        self.modelRoot = None
        self.eventPort = None
        self.eventPortName = None
        self.classTimestamps = None
        self.probClassList = None
        self.recency = None
        self.useRecentClassTime = True
        self.my_mutex = thread.allocate_lock()

    def configure(self, rf):

        stringCommand = 'from SAM.SAM_Drivers import ' + sys.argv[4] + ' as Driver'
        exec stringCommand

        self.mm = [Driver()]
        self.dataPath = sys.argv[1]
        self.modelPath = sys.argv[2]
        self.driverName = sys.argv[4]
        self.configPath = sys.argv[3]
        self.windowedMode = sys.argv[5] == 'True'
        self.modelRoot = self.dataPath.split('/')[-1]

        file_i = 0
        loggerFName = join(self.dataPath, self.baseLogFileName + '_' + str(file_i) + '.log')

        # check if file exists
        while os.path.isfile(loggerFName) and os.path.getsize(loggerFName) > 0:
            loggerFName = join(self.dataPath, self.baseLogFileName + '_' + str(file_i) + '.log')
            file_i += 1

        if self.windowedMode:
            logFormatter = logging.Formatter("[%(levelname)s]  %(message)s")
        else:
            logFormatter = logging.Formatter("\033[31m%(asctime)s [%(name)-33s] [%(levelname)8s]  %(message)s\033[0m")

        rootLogger = logging.getLogger('interaction ' + self.driverName)
        rootLogger.setLevel(logging.DEBUG)

        fileHandler = logging.FileHandler(loggerFName)
        fileHandler.setFormatter(logFormatter)
        rootLogger.addHandler(fileHandler)

        consoleHandler = logging.StreamHandler()
        consoleHandler.setFormatter(logFormatter)
        rootLogger.addHandler(consoleHandler)
        logging.root = rootLogger

        off = 17
        logging.info('Arguments: ' + str(sys.argv))
        logging.info(stringCommand)
        logging.info('Using log' + str(loggerFName))
        logging.info('-------------------')
        logging.info('Interaction Settings:')
        logging.info('Data Path: '.ljust(off) + str(self.dataPath))
        logging.info('Model Path: '.ljust(off) + str(self.modelPath))
        logging.info('Config Path: '.ljust(off) + str(self.configPath))
        logging.info('Driver: '.ljust(off) + str(self.driverName))
        logging.info('-------------------')
        logging.info('Configuring Interaction...')
        logging.info('')

        # parse settings from config file
        parser2 = SafeConfigParser()
        parser2.read(self.configPath)
        proposedBuffer = 5
        if self.modelRoot in parser2.sections():
            self.portNameList = parser2.items(self.dataPath.split('/')[-1])
            logging.info(str(self.portNameList))
            self.portsList = []
            for j in range(len(self.portNameList)):
                if self.portNameList[j][0] == 'rpcbase':
                    self.portsList.append(yarp.Port())
                    self.portsList[j].open(self.portNameList[j][1]+":i")
                    self.svPort = j
                    self.attach(self.portsList[j])
                elif self.portNameList[j][0] == 'callsign':
                    # should check for repeated call signs by getting list from samSupervisor
                    self.callSignList = self.portNameList[j][1].split(',')
                elif self.portNameList[j][0] == 'collectionmethod':
                    self.collectionMethod = self.portNameList[j][1].split(' ')[0]
                    try:
                        proposedBuffer = int(self.portNameList[j][1].split(' ')[1])
                    except ValueError:
                        logging.error('collectionMethod bufferSize is not an integer')
                        logging.error('Should be e.g: collectionMethod = buffered 3')
                        return False

                    if self.collectionMethod not in ['buffered', 'continuous', 'future_buffered']:
                        logging.error('collectionMethod should be set to buffered / continuous / future_buffered')
                        return False
                elif self.portNameList[j][0] == 'recency':
                        try:
                            self.recency = int(self.portNameList[j][1])
                        except ValueError:
                            logging.error('Recency value for ' + str(self.driverName) + ' is not an integer')
                            self.recency = 5
                else:
                    parts = self.portNameList[j][1].split(' ')
                    logging.info(parts)

                    if parts[1].lower() == 'imagergb':
                        self.portsList.append(yarp.BufferedPortImageRgb())
                        # self.portsList[j].open(parts[0])

                    elif parts[1].lower() == 'imagemono':
                        self.portsList.append(yarp.BufferedPortImageMono())
                        # self.portsList[j].open(parts[0])

                    elif parts[1].lower() == 'bottle':
                        self.portsList.append(yarp.BufferedPortBottle())
                        # self.portsList[j].open(parts[0])

                    else:
                        logging.error('Data type ' + str(parts[1]) + ' for ' +
                                      str(self.portNameList[j][0]) + ' unsupported')
                        return False
                    # mrd models with label/instance training will always have:
                    # 1 an input data line which is used when a label is requested
                    # 2 an output data line which is used when a generated instance is required
                    if parts[0][-1] == 'i':
                        self.labelPort = j
                        self.labelPortName = parts[0]
                    elif parts[0][-1] == 'o':
                        self.instancePort = j
                        self.instancePortName = parts[0]

            if self.collectionMethod == 'continuous':
                self.portsList.append(yarp.BufferedPortBottle())
                self.eventPort = len(self.portsList) - 1
                self.eventPortName = '/'.join(self.labelPortName.split('/')[:3])+'/event'
                self.portsList[self.eventPort].open(self.eventPortName)
                self.classTimestamps = []
                if self.recency is None:
                    logging.warning('No recency value specified for ' + self.driverName)
                    logging.warning('Setting value to default of 5 seconds')
                    self.recency = 5

            if self.svPort is None or self.labelPort is None or self.instancePort is None:
                logging.warning('Config file properties incorrect. Should look like this:')
                logging.warning('[Actions]')
                logging.warning('dataIn = /sam/actions/actionData:i Bottle')
                logging.warning('dataOut = /sam/actions/actionData:o Bottle')
                logging.warning('rpcBase = /sam/actions/rpc')
                logging.warning('callSign = ask_action_label, ask_action_instance')
                logging.warning('collectionMethod = buffered 3')

            # self.mm[0].configInteraction(self)
            self.inputType = self.portNameList[self.labelPort][1].split(' ')[1].lower()
            self.outputType = self.portNameList[self.labelPort][1].split(' ')[1].lower()
            self.dataList = []
            self.classificationList = []
            self.probClassList = []
            self.classTimestamps = []
            yarp.Network.init()

            self.mm = initialiseModels([self.dataPath, self.modelPath, self.driverName], 'update', 'interaction')
            self.modelLoaded = True

            if self.mm[0].model_mode != 'temporal':
                self.bufferSize = proposedBuffer
            elif self.mm[0].model_mode == 'temporal':
                self.bufferSize = self.mm[0].temporalModelWindowSize

            self.portsList[self.labelPort].open(self.labelPortName)
            self.portsList[self.instancePort].open(self.instancePortName)
            # self.test()

            return True
        else:
            logging.error('Section ' + str(self.modelRoot) + ' not found in ' + str(self.configPath))
            return False

    def close(self):
        # close ports of loaded models
        logging.info('Exiting ...')
        for j in self.portsList:
            self.closePort(j)
        return False

    @timeout(3)
    def closePort(self, j):
        j.interrupt()
        time.sleep(1)
        j.close()

    def respond(self, command, reply):
        # this method responds to samSupervisor commands
        reply.clear()
        action = command.get(0).asString()

        count = 0
        while not self.modelLoaded:
            count += 1
            time.sleep(0.5)
            if count == 10:
                break

        if self.modelLoaded:
            if action != 'heartbeat' or action != 'information':
                logging.info(action + ' received')
                logging.info('responding to ' + action + ' request')

            if action == "portNames":
                reply.addString('ack')
                reply.addString(self.labelPortName)
                reply.addString(self.instancePortName)
                if self.collectionMethod == 'continuous':
                    reply.addString(self.eventPortName)
            # -------------------------------------------------
            elif action == "reload":
                # send a message to the interaction model to check version of currently loaded model
                # and compare it with that stored on disk. If model on disk is more recent reload model
                # interaction model to return "model reloaded correctly" or "loaded model already up to date"
                logging.info("reloading model")
                try:
                    self.mm = initialiseModels([self.dataPath, self.modelPath, self.driverName],
                                                'update', 'interaction')
                    reply.addString('ack')
                except:
                    reply.addString('nack')
            # -------------------------------------------------
            elif action == "heartbeat":
                reply.addString('ack')
            # -------------------------------------------------
            elif action == "toggleVerbose":
                self.verboseSetting = not self.verboseSetting
                reply.addString('ack')
            # -------------------------------------------------
            # elif action == "attention":
            #     self.attentionMode = command.get(1).asString()
            #     reply.addString('ack')
            # -------------------------------------------------
            elif action == "information":
                if command.size() < 3:
                    reply.addString('nack')
                else:
                    try:
                        self.additionalInfoDict[command.get(1).asString()] = command.get(2).asString()
                        reply.addString('ack')
                    except:
                        reply.addString('nack')
                    logging.info(self.additionalInfoDict)
            # -------------------------------------------------
            elif action == "EXIT":
                reply.addString('ack')
                self.close()
            # -------------------------------------------------
            elif action in self.callSignList:
                logging.info('call sign command recognized')
                if 'label' in action:
                    self.classifyInstance(reply)
                elif 'instance' in action:
                    self.generateInstance(reply, command.get(1).asString())
            # -------------------------------------------------
            else:
                reply.addString("nack")
                reply.addString("Command not recognized")
        else:
            reply.addString("nack")
            reply.addString("Model not loaded")

        return True

    def classifyInstance(self, reply):
        if self.portsList[self.labelPort].getInputCount() > 0:
            if self.verboseSetting:
                logging.info('-------------------------------------')
            if self.collectionMethod == 'buffered':
                if self.modelLoaded:
                    logging.debug('going in process live')
                    thisClass, probClass, dataList = self.mm[0].processLiveData(self.dataList, self.mm,
                                                                                verbose=self.verboseSetting,
                                                                                additionalData=self.additionalInfoDict)
                    logging.debug('exited process live')
                else:
                    thisClass = None
                logging.debug(thisClass)
                logging.debug('object thisclass' + str(thisClass is None))
                logging.debug('object datalist' + str(dataList is None))
                logging.debug('string thisclass' + str(thisClass == 'None'))
                logging.debug('string datalist' + str(dataList == 'None'))
                if thisClass is None or dataList is None:
                    logging.debug('None reply')
                    reply.addString('nack')
                else:
                    logging.debug('correct reply')
                    reply.addString(thisClass)
                logging.debug('finish reply')
                    # reply.addDouble(probClass)
            # -------------------------------------------------
            elif self.collectionMethod == 'continuous':
                # mutex lock classificationList
                self.my_mutex.acquire()
                logging.debug(self.classificationList)
                logging.debug(self.classTimestamps)
                # check last n seconds
                if len(self.classificationList) > 0:
                    if self.useRecentClassTime:
                        minT = self.classTimestamps[-1] - self.recency
                    else:
                        minT = time.time() - self.recency

                    logging.debug('minT ' + str(minT))
                    logging.debug('recency ' + str(self.recency))
                    for index, value in enumerate(self.classTimestamps):
                        logging.debug(str(index) + ' ' + str(value) + ' ' + str(value > minT))
                    validList = [index for index, value in enumerate(self.classTimestamps) if value > minT]
                    logging.debug('validList ' + str(validList))
                    minIdx = min(validList)
                    logging.debug('minIdx ' + str(minIdx))
                    validClassList = self.classificationList[minIdx:]
                    validProbList = self.probClassList[minIdx:]
                    logging.debug('validClassList ' + str(validClassList))
                    logging.debug('classify classList' + str(self.classificationList))
                    logging.debug('classify probclassList' + str(self.probClassList))
                    logging.debug('classify classTimeStamps' + str(self.classTimestamps))

                    if len(validClassList) > 0:

                        # combine all classifications
                        if len(validClassList) == 1:
                            logging.debug('validClassList is of len 1')
                            decision = validClassList[0]
                        else:
                            logging.debug('validClassList is of len ' + str(len(validClassList)))
                            setClass = list(set(validClassList))
                            logging.debug('setClass ' + str(setClass))
                            if len(setClass) == len(validClassList):
                                logging.debug('len setClass = len validClassList' + str(len(setClass)) + ' ' +
                                              str(len(validClassList)))
                                decision = validClassList[validProbList.index(max(validProbList))]
                            else:
                                dictResults = dict()
                                for m in setClass:
                                    logging.debug('currentM ' + str(m))
                                    idxM = [idx for idx, name in enumerate(validClassList) if name == m]
                                    logging.debug('idx ' + str(idxM))
                                    probVals = itemgetter(*idxM)(validProbList)
                                    logging.debug('probs ' + str(probVals))
                                    try:
                                        probSum = sum(probVals)
                                    except TypeError:
                                        probSum = probVals
                                    logging.debug('sum ' + str(probSum))
                                    dictResults[m] = probSum
                                    logging.debug('')
                                logging.debug('dictResults ' + str(dictResults))
                                maxDictProb = max(dictResults.values())
                                logging.debug('maxDictProb = ' + str(maxDictProb))
                                decisions = [key for key in dictResults.keys() if dictResults[key] == maxDictProb]
                                logging.info('Decision: ' + str(decisions))
                                logging.info('We have resolution')
                                decision = ' and '.join(decisions)
                        logging.info('Decision: ' + decision)

                        reply.addString('ack')
                        reply.addString(decision)
                        # reply.addString(validClassList[-1])
                        # self.classificationList.pop(-1)

                        # remove validclassList from self.classificationList / probClassList / classTimeStamps
                        self.classificationList = self.classificationList[:minIdx]
                        self.probClassList = self.probClassList[:minIdx]
                        self.classTimestamps = self.classTimestamps[:minIdx]

                    else:
                        logging.info('No valid classifications')
                        reply.addString('nack')
                    logging.debug('replying ' + reply.toString())
                    logging.debug('classify classList' + str(self.classificationList))
                    logging.debug('classify probclassList' + str(self.probClassList))
                    logging.debug('classify classTimeStamps' + str(self.classTimestamps))
                else:
                    logging.info('No classifications yet')
                    reply.addString('nack')
                self.my_mutex.release()
            # -------------------------------------------------
            elif self.collectionMethod == 'future_buffered':
                self.dataList = []
                for j in range(self.bufferSize):
                    self.dataList.append(self.readFrame())
                if self.modelLoaded:
                    thisClass, probClass, dataList = self.mm[0].processLiveData(self.dataList, self.mm,
                                                                                verbose=self.verboseSetting,
                                                                                additionalData=self.additionalInfoDict)
                else:
                    thisClass = None

                if thisClass is None or dataList is None:
                    reply.addString('nack')
                else:
                    reply.addString(thisClass)
                    # reply.addDouble(probClass)
        else:
            reply.addString('nack')
            reply.addString('No input connections to ' + str(self.portsList[self.labelPort].getName()))
        logging.info('--------------------------------------')

    def generateInstance(self, reply, instanceName):
        if self.portsList[self.instancePort].getOutputCount() != 0:
            if instanceName in self.mm[0].textLabels:
                instance = self.recallFromLabel(instanceName)
                # send generated instance to driver where it is converted into the proper format
                formattedData = self.mm[0].formatGeneratedData(instance)
                # check formattedData is of correct data type
                if str(type(self.portsList[self.instancePort])).split('\'')[1].split('Port')[1] \
                        in str(type(formattedData)):
                    try:
                        img = self.portsList[self.instancePort].prepare()
                        img.copy(formattedData)
                        self.portsList[self.instancePort].write()
                        reply.addString('ack')
                        reply.addString('Generated instance of ' + instanceName + ' as ' +
                                        str(type(formattedData)))
                    except:
                        reply.addString('nack')
                        reply.addString('Failed to write ' + instanceName + ' as ' +
                                        str(type(self.portsList[self.instancePort])))
                else:
                    reply.addString('nack')
                    reply.addString('Output of ' + self.driverName + '.formatGeneratedData is of type: ' +
                                    str(type(formattedData)) + '. Should be type: ' +
                                    str(type(self.portsList[self.instancePort])))
            else:
                reply.addString('nack')
                reply.addString('Instance name not found. Available instance names are: ' + str(self.mm[0].textLabels))
        else:
            reply.addString('nack')
            reply.addString('No outgoing connections on ' + str(self.portsList[self.instancePort].getName()))

    def recallFromLabel(self, label):

        ind = self.mm[0].textLabels.index(label)
        if len(self.mm) > 1:
            indsToChooseFrom = self.mm[ind + 1].SAMObject.model.textLabelPts[ind]
            chosenInd = np.random.choice(indsToChooseFrom, 1)
            yrecall = self.mm[ind + 1].SAMObject.recall(chosenInd)
        else:
            indsToChooseFrom = self.mm[0].SAMObject.model.textLabelPts[ind]
            chosenInd = np.random.choice(indsToChooseFrom, 1)
            yrecall = self.mm[0].SAMObject.recall(chosenInd)

        return yrecall

    def interruptModule(self):
        return True

    def getPeriod(self):
        return 0.1

    def updateModule(self):
        # this method will monitor incoming data

        # test incoming data connection
        out = self.portsList[self.svPort].getOutputCount() + self.portsList[self.svPort].getInputCount()
        if out != 0:
            if not self.rpcConnected:
                logging.info("Connection received")
                logging.info('\n')
                logging.info('-------------------------------------')
                self.rpcConnected = True
                self.falseCount = 0
            else:
                self.dataInConnected = self.portsList[self.labelPort].getInputCount() > 0
                if self.dataInConnected:
                    self.collectData()
                else:
                    self.noDataCount += 1
                    if self.noDataCount == self.errorRate:
                        self.noDataCount = 0
                        logging.info('No data in connection. Waiting for ' +
                                     self.portNameList[self.labelPort][1] + ' to receive a connection')
        else:
            self.rpcConnected = False
            self.falseCount += 1
            if self.falseCount == self.errorRate:
                self.falseCount = 0
                logging.info('Waiting for ' + self.portNameList[self.svPort][1] +
                             ' to receive a connection')

        time.sleep(0.05)
        return True

    def readFrame(self):
        if self.inputType == 'imagergb':
            frame = yarp.ImageRgb()
        elif self.inputType == 'imagemono':
            frame = yarp.ImageMono()
        elif self.inputType == 'bottle':
            frame = yarp.Bottle()
        else:
            return None

        frameRead = self.portsList[self.labelPort].read(True)

        if self.inputType == 'bottle':
            frame.fromString(frameRead.toString())
        elif 'image' in self.inputType:
            frame.copy(frameRead)

        return frame

    def collectData(self):

        self.noDataCount = 0

        if self.collectionMethod == 'buffered':
            frame = self.readFrame()
            # append frame to buffer
            if len(self.dataList) == self.bufferSize:
                # FIFO buffer first item in list most recent
                self.dataList.pop(0)
                self.dataList.append(frame)
            else:
                self.dataList.append(frame)
        # -------------------------------------------------
        elif self.collectionMethod == 'continuous' and self.attentionMode == 'continue':
            # read frame of data
            frame = self.readFrame()
            # append frame to dataList

            if self.dataList is None:
                self.dataList = []

            self.dataList.append(frame)
            # process list of frames for a classification
            dataList = []
            if self.modelLoaded:
                thisClass, probClass, dataList = self.mm[0].processLiveData(self.dataList, self.mm,
                                                                            verbose=self.verboseSetting,
                                                                            additionalData=self.additionalInfoDict)
            else:
                thisClass = None
            # if proper classification
            if thisClass is not None:
                # empty dataList

                # mutex dataList lock
                self.my_mutex.acquire()
                self.dataList = dataList
                # mutex dataList release

                if thisClass != 'None':
                    tStamp = time.time()
                    eventBottle = self.portsList[self.eventPort].prepare()
                    eventBottle.clear()
                    eventBottle.addString('ack')
                    self.portsList[self.eventPort].write()
                    # add classification to classificationList to be retrieved during respond method

                    # mutex classificationList lock

                    # Time based method
                    logging.info('classList len: ' + str(len(self.classificationList)))
                    logging.debug('thisclass ' + str(thisClass))
                    self.classificationList = self.classificationList + thisClass
                    logging.debug('classificationList ' + str(self.classificationList))
                    self.probClassList = self.probClassList + probClass
                    logging.debug('probClass ' + str(self.probClassList))
                    self.classTimestamps = self.classTimestamps + [tStamp]*len(thisClass)
                    logging.debug('self.classTimestamps ' + str(self.classTimestamps))
                    # remove timestamps older than memory duration (self.bufferSize in seconds)
                    logging.debug('last time stamp: ' + str(self.classTimestamps[-1]))
                    minMemT = self.classTimestamps[-1] - self.bufferSize
                    logging.debug('minMemT ' + str(minMemT))
                    goodIdxs = [idx for idx, timeVal in enumerate(self.classTimestamps) if timeVal > minMemT]
                    logging.debug('goodIdxs ' + str(goodIdxs))
                    minIdx = min(goodIdxs)
                    self.classificationList = self.classificationList[minIdx:]
                    self.probClassList = self.probClassList[minIdx:]
                    self.classTimestamps = self.classTimestamps[minIdx:]

                    logging.debug('classificationList ' + str(self.classificationList))
                    logging.debug('probClass ' + str(self.probClassList))
                    logging.debug('self.classTimestamps ' + str(self.classTimestamps))

                    # Old method
                    # if len(self.classificationList) == self.bufferSize:
                    #     # FIFO buffer first item in list is oldest
                    #     self.classificationList.pop(0)
                    #     self.classTimestamps.pop(0)
                    #     self.classificationList.append(thisClass)
                    #     self.classTimestamps.append(tStamp)
                    # else:
                    #     self.classificationList.append(thisClass)
                    #     self.classTimestamps.append(tStamp)

                # mutex release
                self.my_mutex.release()
        # -------------------------------------------------
        elif self.collectionMethod == 'future_buffered':
            pass

    def test(self):
        count = 0
        if self.collectionMethod == 'continuous':
            classifyBlock = self.mm[0].paramsDict['windowSize']
        elif self.collectionMethod == 'buffered':
            classifyBlock = self.bufferSize
        else:
            classifyBlock = 10

        while True:
            out = (self.portsList[self.svPort].getOutputCount() + self.portsList[self.svPort].getInputCount()) > 0
            dataInConnected = self.portsList[self.labelPort].getInputCount() > 0

            if out and dataInConnected:
                if self.collectionMethod == 'future_buffered':
                    reply = yarp.Bottle()
                    self.classifyInstance(reply)
                    logging.info(reply.toString())
                elif self.collectionMethod == 'continuous':
                    self.collectData()
                    count += 1
                    if count == classifyBlock:
                        count = 0
                        reply = yarp.Bottle()
                        self.classifyInstance(reply)
                        logging.info('CLASSIFICATION: ' + reply.toString())

                # self.dataList = []
                # for j in range(self.bufferSize):
                #     self.dataList.append(self.readFrame())

                # if thisClass is None:
                #     logging.info('None')
                # else:
                #     logging.info(thisClass, ' ', likelihood)

            time.sleep(0.05)


def exception_hook(exc_type, exc_value, exc_traceback):
    logging.error("Uncaught exception", exc_info=(exc_type, exc_value, exc_traceback))

sys.excepthook = exception_hook

if __name__ == '__main__':
    plt.ion()
    yarp.Network.init()
    mod = interactionSAMModel()
    rf = yarp.ResourceFinder()
    rf.setVerbose(True)
    # rf.setDefaultContext("samSupervisor")
    # rf.setDefaultConfigFile("default.ini")
    rf.configure(sys.argv)

    mod.runModule(rf)
