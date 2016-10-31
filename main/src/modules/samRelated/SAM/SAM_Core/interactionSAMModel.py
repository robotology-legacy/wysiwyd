#!/usr/bin/env ipython
import matplotlib.pyplot as plt
import sys
import time
from ConfigParser import SafeConfigParser
from SAM.SAM_Core import SAMDriver as Driver
from SAM.SAM_Core import SAM_utils
import readline
import warnings
import numpy as np
import yarp
warnings.simplefilter("ignore")


class interactionSAMModel(yarp.RFModule):

    def __init__(self):
        yarp.RFModule.__init__(self)
        self.mm = None
        self.dataPath = None
        self.configPath = None
        self.modelPath = None
        self.driverName = None
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

    def configure(self, rf):

        print sys.argv
        stringCommand = 'from SAM.SAM_Drivers import ' + sys.argv[4] + ' as Driver'
        print stringCommand
        exec stringCommand

        self.mm = [Driver()]
        self.dataPath = sys.argv[1]
        self.modelPath = sys.argv[2]
        self.driverName = sys.argv[4]
        self.configPath = sys.argv[3]
        self.modelRoot = self.dataPath.split('/')[-1]

        off = 17
        print '-------------------'
        print 'Interaction Settings:'
        print
        print 'Data Path: '.ljust(off), self.dataPath
        print 'Model Path: '.ljust(off), self.modelPath
        print 'Model Root: '.ljust(off),
        print 'Config Path: '.ljust(off), self.configPath
        print 'Driver:'.ljust(off), self.driverName
        print '-------------------'
        print 'Configuring Interaction...'
        print

        # parse settings from config file
        parser2 = SafeConfigParser()
        parser2.read(self.configPath)
        if self.modelRoot in parser2.sections():
            self.portNameList = parser2.items(self.dataPath.split('/')[-1])
            print self.portNameList
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
                        print 'collectionMethod bufferSize is not an integer'
                        print 'Should be e.g: collectionMethod = buffered 3'
                        return False

                    if self.collectionMethod not in ['buffered', 'continuous', 'future_buffered']:
                        print 'collectionMethod should be set to buffered / continuous / future_buffered'
                        return False
                else:
                    parts = self.portNameList[j][1].split(' ')
                    print parts

                    if parts[1].lower() == 'imagergb':
                        self.portsList.append(yarp.BufferedPortImageRgb())
                        self.portsList[j].open(parts[0])

                    elif parts[1].lower() == 'imagemono':
                        self.portsList.append(yarp.BufferedPortImageMono())
                        self.portsList[j].open(parts[0])

                    elif parts[1].lower() == 'bottle':
                        self.portsList.append(yarp.BufferedPortBottle())
                        self.portsList[j].open(parts[0])

                    else:
                        print 'Data type ', parts[1], 'for ', self.portNameList[j][0], ' unsupported'
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

            if self.svPort is None or self.labelPort is None or self.instancePort is None:
                print 'Config file properties incorrect. Should look like this:'
                print '[Actions]'
                print 'dataIn = /sam/actions/actionData:i Bottle'
                print 'dataOut = /sam/actions/actionData:o Bottle'
                print 'rpcBase = /sam/actions/rpc'
                print 'callSign = ask_action_label, ask_action_instance'
                print 'collectionMethod = buffered 3'

            # self.mm[0].configInteraction(self)
            self.inputType = self.portNameList[self.labelPort][1].split(' ')[1].lower()
            self.outputType = self.portNameList[self.labelPort][1].split(' ')[1].lower()
            self.dataList = []
            self.classificationList = []
            yarp.Network.init()

            self.mm = SAM_utils.initialiseModels([self.dataPath, self.modelPath, self.driverName], 'update',
                                                 'interaction')

            if self.mm[0].model_mode != 'temporal':
                self.bufferSize = proposedBuffer
            elif self.mm[0].model_mode == 'temporal':
                self.bufferSize = self.mm[0].temporalModelWindowSize

            # self.test()

            return True
        else:
            print 'Section ' + self.modelRoot + ' not found in ' + self.configPath
            return False

    def close(self):
        # close ports of loaded models
        print 'Exiting ...'
        for j in self.portsList:
            j.close()
        return False

    def respond(self, command, reply):
        # this method responds to samSupervisor commands
        b = yarp.Bottle()
        reply.clear()
        action = command.get(0).asString()
        if action != 'heartbeat':
            print(action + ' received')
            print 'responding to ' + action + ' request'

        if action == "reload":
            # send a message to the interaction model to check version of currently loaded model
            # and compare it with that stored on disk. If model on disk is more recent reload model
            # interaction model to return "model reloaded correctly" or "loaded model already up to date"
            print "reloading model"
            try:
                self.mm = SAM_utils.initialiseModels([self.dataPath, self.modelPath, self.driverName],
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
        elif action == "portNames":
            reply.addString('ack')
            reply.addString(self.labelPortName)
            reply.addString(self.instancePortName)
        # -------------------------------------------------
        elif action == "EXIT":
            reply.addString('ack')
            self.close()
        # -------------------------------------------------
        elif action in self.callSignList:
            if 'label' in action:
                self.classifyInstance(reply)
            elif 'instance' in action:
                self.generateInstance(reply, command.get(1).asString())
        # -------------------------------------------------
        else:
            reply.addString("nack")
            reply.addString("Command not recognized")

        return True

    def classifyInstance(self, reply):
        if self.portsList[self.labelPort].getInputCount() > 0:
            if self.verboseSetting:
                print '-------------------------------------'
            if self.collectionMethod == 'buffered':
                thisClass = self.mm[0].processLiveData(self.dataList, self.mm, self.verboseSetting)
                if thisClass is None:
                    reply.addString('None')
                else:
                    reply.addString(thisClass)
                    # reply.addDouble(likelihood)
            # -------------------------------------------------
            elif self.collectionMethod == 'continuous':
                if len(self.classificationList) > 0:
                    reply.addString(self.classificationList[-1])
                    self.classificationList.pop(-1)
                else:
                    reply.addString('None')
            # -------------------------------------------------
            elif self.collectionMethod == 'future_buffered':
                self.dataList = []
                for j in range(self.bufferSize):
                    self.dataList.append(self.readFrame())
                # thisClass = self.mm[0].processLiveData(self.dataList, self.mm)
                thisClass = self.mm[0].processLiveData(self.dataList, self.mm, self.verboseSetting)
                if thisClass is None:
                    reply.addString('None')
                else:
                    reply.addString(thisClass)
                    # reply.addDouble(likelihood)
        else:
            reply.addString('nack')
            reply.addString('No input connections to ' + str(self.portsList[self.labelPort].getName()))

    def generateInstance(self, reply, instanceName):
        if self.portsList[self.instancePort].getOutputCount() != 0:
            if instanceName in self.mm[0].textLabels:
                instance = self.recallFromLabel(instanceName)
                # send generated instance to driver where it is converted into the proper format
                formattedData = self.mm[0].formatGeneratedData(instance)
                # check formattedData is of correct data type
                if str(type(self.portsList[self.instancePort])).split('\'')[1].split('Port')[1] in str(type(formattedData)):
                    try:
                        img = self.portsList[self.instancePort].prepare()
                        img.copy(formattedData)
                        self.portsList[self.instancePort].write()

                        reply.addString('Generated instance of ' + instanceName + ' as ' +
                                        str(type(formattedData)))
                    except:
                        reply.addString('Failed to write ' + instanceName + ' as ' +
                                        str(type(self.portsList[self.instancePort])))
                else:
                    reply.addString('Output of ' + self.driverName + '.formatGeneratedData is of type: ' +
                                    str(type(formattedData)) + '. Should be type: ' +
                                    str(type(self.portsList[self.instancePort])))
            else:
                reply.addString('Instance name not found. Available instance names are: ' + str(self.mm[0].textLabels))
        else:
            reply.addString('nack')
            reply.addString('No outgoing connections on ' + str(self.portsList[self.instancePort].getName()))

    def recallFromLabel(self, label):

        ind = self.mm[0].textLabels.index(label)
        yrecall = None
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
                print "Connection received"
                print
                print '-------------------------------------'
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
                        print 'No data in connection. Waiting for ' + self.portNameList[self.labelPort][1] + \
                              ' to receive a connection'
        else:
            self.rpcConnected = False
            self.falseCount += 1
            if self.falseCount == self.errorRate:
                self.falseCount = 0
                print 'Waiting for ' + self.portNameList[self.svPort][1] + ' to receive a connection'

        time.sleep(0.05)
        return True

    def readFrame(self):

        if self.inputType == 'imagergb':
            frame = yarp.ImageRgb()
        elif self.inputType == 'imagemono':
            frame = yarp.ImageMono()
        elif self.inputType == 'bottle':
            frame = yarp.Bottle()

        frame = self.portsList[self.labelPort].read(True)

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
        elif self.collectionMethod == 'continuous':
            # read frame of data
            frame = self.readFrame()
            # append frame to dataList
            self.dataList.append(frame)
            # process list of frames for a classification
            thisClass = self.mm[0].processLiveData(self.dataList, self.mm, self.verboseSetting)
            # if proper classification

            if thisClass is not None:
                # empty dataList
                self.dataList = []
                if thisClass != 'None':
                    # add classification to classificationList to be retrieved during respond method
                    print 'classList len:', len(self.classificationList)
                    if len(self.classificationList) == self.bufferSize:
                        # FIFO buffer first item in list is oldest
                        self.classificationList.pop(0)
                        self.classificationList.append(thisClass)
                    else:
                        self.classificationList.append(thisClass)
        # -------------------------------------------------
        elif self.collectionMethod == 'future_buffered':
            pass

    def test(self):
        count = 0
        if self.collectionMethod == 'continuous':
            classifyBlock = 10
        elif self.collectionMethod == 'buffered':
            classifyBlock = self.bufferSize

        while True:
            out = (self.portsList[self.svPort].getOutputCount() + self.portsList[self.svPort].getInputCount()) > 0
            dataInConnected = self.portsList[self.labelPort].getInputCount() > 0

            if out and dataInConnected:
                if self.collectionMethod == 'future_buffered':
                    reply = yarp.Bottle()
                    self.classifyInstance(reply)
                    print reply.toString()
                elif self.collectionMethod == 'continuous':
                    self.collectData()
                    count += 1
                    if count == classifyBlock:
                        count = 0
                        reply = yarp.Bottle()
                        self.classifyInstance(reply)
                        print 'CLASSIFICATION', reply.toString()

                # self.dataList = []
                # for j in range(self.bufferSize):
                #     self.dataList.append(self.readFrame())

                # if thisClass is None:
                #     print 'None'
                # else:
                #     print thisClass, ' ', likelihood

            time.sleep(0.05)

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
