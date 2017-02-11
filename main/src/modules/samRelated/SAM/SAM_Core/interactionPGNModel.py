#!/usr/bin/env ipython
import matplotlib.pyplot as plt
import sys
import time
from SAM.SAM_Core import SAM_utils
import readline
import warnings
from pgmpy.models import BayesianModel
from pgmpy.factors.discrete import TabularCPD
from pgmpy.inference import VariableElimination
import copy
import numpy as np
import SAM_Core.SAMTesting as SAMTesting
from sklearn.preprocessing import normalize
import yarp
import ast
import pickle
yarp.Network()
warnings.simplefilter("ignore")
np.set_printoptions(precision=2)


class interactionPGNModel(yarp.RFModule):

    def __init__(self):
        yarp.RFModule.__init__(self)
        self.portsList = []

    def configure(self, rf):
        # command format will be the following:
        # trainPGClassifier selfName networkStructure
        print sys.argv

        # read network structure and make graph
        # labels in networkStructure identical to model names
        # networkStructure as a string containing a list of tuples

        # selfName = 'actionPGN'
        # netStructureString = "[('Actions3 exp','actionPGN'), ('Actions4','actionPGN')]"

        selfName = sys.argv[1]
        netStructureString = sys.argv[2]

        netStructure = ast.literal_eval(netStructureString)
        print netStructure

        # collect all model names in a list to extract a unique set
        modelList = []
        for k in netStructure:
            modelList += list(k)
        print list(set(modelList))

        # create a port to connect to /sam/rpc:i to query model path for each model name
        portsList = []
        querySupervisorPort = yarp.RpcClient()
        querySupervisorPortName = '/sam/' + selfName + '/queryRpc'
        querySupervisorPort.open(querySupervisorPortName)

        portsList.append({'name': querySupervisorPortName, 'port': querySupervisorPort})
        yarp.Network.connect(querySupervisorPortName, '/sam/rpc:i')
        # ---------------------------------------------------------------------------------------------------------------
        modelDict = dict()
        failFlag = False
        for j in modelList:
            if j != selfName:
                modNameSplit = j.split(' ')
                cmd = yarp.Bottle()
                cmd.addString('dataDir')
                for l in modNameSplit:
                    cmd.addString(l)
                reply = yarp.Bottle()
                querySupervisorPort.write(cmd, reply)
                if reply.get(0).asString() != 'nack':
                    modelDict[modNameSplit[0]] = {'filename': reply.get(1).asString(), 'pickleData': None}
                    # try:
                    # load pickle for the model file
                    currPickle = pickle.load(open(reply.get(1).asString(), 'rb'))
                    # try loading labelComparisonDict from the pickle
                    if 'labelComparisonDict' in currPickle.keys():
                        modelDict[modNameSplit[0]]['pickleData'] = currPickle['labelComparisonDict']
                        print j, 'labelComparisonDict loaded'
                    else:
                        print modNameSplit[0], 'labelComparisonDict not found'
                        failFlag = True

                    if 'overallPerformanceLabels' in currPickle.keys():
                        modelDict[modNameSplit[0]]['labels'] = currPickle['overallPerformanceLabels']
                        print j, 'overallPerformanceLabels loaded'
                    else:
                        print j, 'overallPerformanceLabels not found'
                        failFlag = True
                    # except:
                    #     failFlag = True
                else:
                    failFlag = True

        print 'FAIL?', failFlag
        if failFlag:
            return False

        modelList = modelDict.keys()
        print modelList

        # ---------------------------------------------------------------------------------------------------------------

        # extract unique lists from the collected data
        # the unique list of pickleData[original] represents the possibleClassifications for each model
        modelDict[selfName] = dict()
        modelDict[selfName]['labels'] = []
        selfModelCol = 1

        for j in modelList:
            modelDict[j]['CPD'] = np.zeros([1, len(modelDict[j]['labels'])])
            print j, 'unique labels:', modelDict[j]['labels']
            print j, 'CPD shape', modelDict[j]['CPD'].shape

            modelDict[selfName]['labels'] += modelDict[j]['labels']
            selfModelCol *= len(modelDict[j]['labels'])
            print

        # the possibleClassifications for both models (outputs of the PGN)
        # are the unique list of the model specific labels for all models
        modelDict[selfName]['labels'] = list(set(modelDict[selfName]['labels']))
        modelDict[selfName]['actualLabels'] = modelDict[j]['pickleData']['original']
        modelDict[selfName]['CPD'] = np.zeros([len(modelDict[selfName]['labels']), selfModelCol])
        print selfName, 'unique labels:', modelDict[selfName]['labels']
        print selfName, 'CPD shape', modelDict[selfName]['CPD'].shape

        # check that original classifications of both are identical
        # otherwise cannot combine them with a single node.
        # This is currently a big limitation that will be removed later
        print modelDict[selfName]['labels']
        for j in modelList:
            print j,
            for k in range(len(modelDict[j]['pickleData']['original'])):
                print modelDict[j]['pickleData']['original'][k]
                if modelDict[j]['pickleData']['original'][k] not in modelDict[selfName]['labels']:
                    modelDict[j]['pickleData']['original'][k] = 'unknown'

        for j in modelList:
            if modelDict[j]['pickleData']['original'] != modelDict[selfName]['actualLabels']:
                failFlag = True
                print 'original classifications of', j, 'are not identical to those of', selfName

        if failFlag:
            return False

        # Update netStructureString to reflect changes in the modelList names
        strSections = netStructureString.split("'")
        for k in range(len(strSections)):
            if len(strSections[k]) > 2 and ',' not in strSections[k]:
                strSections[k] = strSections[k].split(' ')[0]
        netStructureString = "'".join(strSections)
        netStructure = ast.literal_eval(netStructureString)
        # ---------------------------------------------------------------------------------------------------------------
        # iterate through actual labels
        # for each actual label, iterate through models
        # for each model find classification label of this model for current actual label
        # get the index of the current classification and add it to its CPD
        # also calculate which item in the joint CPD needs to be incremented

        for j in range(len(modelDict[selfName]['actualLabels'])):
            currActualLabel = modelDict[selfName]['actualLabels'][j]
            row = modelDict[selfName]['labels'].index(currActualLabel)

            colVar = np.zeros([len(modelList)])
            for k in range(len(modelList)):
                cmod = modelList[k]
                if k != 0:
                    pmod = modelList[k-1]
                    colVar *= len(modelDict[pmod]['labels'])

                colVar[k] = modelDict[cmod]['labels'].index(
                                   modelDict[cmod]['pickleData']['results'][j])
                modelDict[cmod]['CPD'][0, colVar[k]] += 1

            col = sum(colVar)
            modelDict[selfName]['CPD'][row, col] += 1

        # take all CPD's and normalise the matrices
        evidenceCard = copy.deepcopy(modelList)
        for j in modelDict:
            if j == selfName:
                # this is a joint CPD matrix
                # normalise columns to have sum = 1
                modelDict[j]['CPD'] = normalize(modelDict[j]['CPD'], axis=0, norm='l1')
            else:
                # normalise sum of matrix = 1
                modelDict[j]['CPD'] /= np.sum(modelDict[j]['CPD'])
                evidenceCard[evidenceCard.index(j)] = len(modelDict[j]['labels'])
            print modelDict[j]['CPD']

        model = BayesianModel(netStructure)

        # create TabularCPD data structure to nest calculated CPD
        for j in modelDict:
            if j == selfName:
                modelDict[j]['cpdObject'] = TabularCPD(variable=j, variable_card=len(modelDict[j]['labels']),
                                                       values=modelDict[j]['CPD'],
                                                       evidence=modelList,
                                                       evidence_card=evidenceCard)
            else:
                modelDict[j]['cpdObject'] = TabularCPD(variable=j,
                                                       variable_card=len(modelDict[j]['labels']),
                                                       values=modelDict[j]['CPD'])

        # Associating the CPDs with the network
        for j in modelDict:
            model.add_cpds(modelDict[j]['cpdObject'])

        # check_model checks for the network structure and CPDs and verifies that the CPDs are correctly
        # defined and sum to 1.
        if not model.check_model():
            print 'Model check returned unsuccessful'
            return False

        infer = VariableElimination(model)
        confMatrix = np.zeros(len(modelDict[selfName]['labels']))
        # iterate over all original data and perform classifications to calculate if accuracy with PGN has increased
        for j in range(len(modelDict[selfName]['actualLabels'])):
            currEvidenceDict = dict()
            for k in modelList:
                currEvidenceDict[k] = modelDict[k]['labels'].index(modelDict[k]['pickleData']['results'][j])

            q = infer.query([selfName], currEvidenceDict)

            inferenceClass = modelDict[selfName]['labels'][np.argmax(q[selfName].values)]
            actualClass = modelDict[selfName]['actualLabels'][j]
            confMatrix[modelDict[selfName].index(actualClass), modelDict[selfName].index(inferenceClass)] += 1

        print "%Accuracy with PGN"
        dCalc = SAMTesting.calculateData(modelDict[selfName]['actualLabels'], confMatrix)

        return True

    def close(self):
        # close ports of loaded models
        print 'Exiting ...'
        for j in self.portsList:
            self.closePort(j)
        return False

    @SAM_utils.timeout(3)
    def closePort(self, j):
        j.interrupt()
        time.sleep(1)
        j.close()

    def respond(self, command, reply):
        # this method responds to samSupervisor commands
        b = yarp.Bottle()
        reply.clear()
        action = command.get(0).asString()
        if action != 'heartbeat':
            print(action + ' received')
            print 'responding to ' + action + ' request'
        # -------------------------------------------------
        elif action == "heartbeat":
            reply.addString('ack')
        # -------------------------------------------------
        elif action == "portNames":
            reply.addString('ack')
            # reply.addString(self.labelPortName)
            # reply.addString(self.instancePortName)
            # if self.collectionMethod == 'continuous':
            #     reply.addString(self.eventPortName)
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

    def interruptModule(self):
        return True

    def getPeriod(self):
        return 0.1

    def updateModule(self):
        # this method will monitor incoming data
        time.sleep(0.05)
        return True

if __name__ == '__main__':
    plt.ion()
    yarp.Network.init()
    mod = interactionPGNModel()
    rf = yarp.ResourceFinder()
    rf.setVerbose(True)
    # rf.setDefaultContext("samSupervisor")
    # rf.setDefaultConfigFile("default.ini")
    rf.configure(sys.argv)

    mod.runModule(rf)
