#!/usr/bin/env ipython
import matplotlib.pyplot as plt
import SAM
import sys
import subprocess
import os
from os import listdir
from os.path import isfile, join, isdir
import glob
import pkgutil
import time
import datetime
import signal
import readline
import yarp
from ConfigParser import SafeConfigParser
import SAM.SAM_Core.SAM_utils as utils
from time import sleep


class SamSupervisorModule(yarp.RFModule):

    def __init__(self):
        yarp.RFModule.__init__(self)

    def configure(self, rf):
        yarp.Network.init()
        self.SIGNALS_TO_NAMES_DICT = dict((getattr(signal, n), n) \
            for n in dir(signal) if n.startswith('SIG') and '_' not in n )
        
        self.terminal = 'xterm'

        rootPath = rf.check("root_path")
        interactionConfPath = rf.check("config_path")

        if(interactionConfPath == False and rootPath == False):
            print "Cannot find .ini settings"
            return False
        else:
            self.rootPath = rf.find("root_path").asString()
            self.interactionConfPath = rf.find("config_path").asString()
            persistence = rf.check("persistence", yarp.Value("False")).asString()
            windowed = rf.check("windowed", yarp.Value("True")).asString()
            verbose = rf.check("verbose", yarp.Value("True")).asString()
            startModels = rf.check("startModels", yarp.Value("True")).asString()

            self.startModels = startModels == 'True'
            self.persistence = True if(persistence == "True") else False
            self.windowed = True if(windowed == "True") else False
            self.verbose = True if(verbose == "True") else False

            print 'Root supervisor path:     \t', self.rootPath
            print 'Model configuration file: \t', self.interactionConfPath
            print 'Bash Persistence set to:  \t', self.persistence
            print 'Windowed set to:          \t', self.windowed
            print 'Verbose set to:           \t', self.verbose

            self.modelPath = self.rootPath + '/Models'
            self.dataPath = self.rootPath + '/Data' 
            #OLD
            # self.trainingFunctionsPath = os.environ.get("WYSIWYD_DIR")+"/bin"
            #NEW
            self.trainingFunctionsPath = SAM.SAM_Drivers.__path__
            self.trainingListHandles = dict() #make this a dict to have a label attached to each subprocess
            self.loadedListHandles = dict()
            self.iter = 0
            self.rpcConnections = []
            self.inputBottle = yarp.Bottle()
            self.sendingBottle = yarp.Bottle()
            self.responseBottle = yarp.Bottle()
            self.outputBottle = yarp.Bottle()

            if(not self.windowed): self.devnull = open('/dev/null', 'w')
            
            out = yarp.Bottle()
            self.checkAvailabilities(out)
            if(self.verbose): print out.toString()

            self.supervisorPort = yarp.Port()
            self.supervisorPort.open('/sam/rpc:i')
            self.attach(self.supervisorPort)

            cmd = 'ipcluster start -n 4'
            command = "bash -c \"" + cmd + "\""

            if self.windowed:
                c = subprocess.Popen([self.terminal, '-e', command], shell=False)
            else:
                c = subprocess.Popen([cmd], shell=True, stdout=self.devnull, stderr=self.devnull)

            self.trainingListHandles['Cluster'] = c

            if len(self.uptodateModels) + len(self.updateModels) > 0:
                if self.verbose:
                    print "Loading models according to " + self.interactionConfPath
                # start loading model configuration according to interactionConfPath file
                
                rfModel = yarp.ResourceFinder()
                rfModel.setVerbose(self.verbose)
                rfModel.setDefaultContext("samSupervisor")
                self.interactionConfFile = rfModel.findFile(self.interactionConfPath);
                
                # Iterate over all sections within the interactionConfPath,
                # create a list and check against the available models
                # warn if model specified in interactionConfPath not loadable
                self.interactionParser = SafeConfigParser()
                self.interactionParser.read(self.interactionConfFile)
                self.interactionSectionList = self.interactionParser.sections()
                if self.verbose:
                    print
                    print self.dataPath
                    print self.interactionSectionList
                    print
                if self.startModels:
                    for j in self.interactionSectionList:
                        command = yarp.Bottle()
                        command.addString("load")
                        command.addString(j)
                        if self.verbose:
                            print command.toString()
                        reply = yarp.Bottle()

                        self.loadModel(reply, command)
                        if self.verbose:
                            print reply.toString()
                            print "-----------------------------------------------"
                            print
                else:
                    print 'Config ready. Awaiting input ...'
            elif len(self.noModels) > 0:
                if self.verbose:
                    print "Models available for training."
                # Train a model according to ineractionConfPath file
            else:
                if self.verbose:
                    print "No available models to load or train"
                # wait for a training command
                
            return True

    def close(self):
        # close ports of loaded models
        for j in self.rpcConnections:
            j[1].write(yarp.Bottle('EXIT'), self.inputBottle)
            j[1].close()

        self.supervisorPort.close()

        for i, v in self.trainingListHandles.iteritems():
            v.send_signal(signal.SIGINT)
            v.wait()   

        for v in self.rpcConnections:
            v[4].send_signal(signal.SIGINT)
            v[4].wait() 

    def checkAvailabilities(self, reply):
        # after finding the root path, go to models folder and compile list of all
        # models together with the last time they were modified
        onlyfiles = [f for f in listdir(self.modelPath) if isfile(join(self.modelPath, f))]

        # find number of .pickle files
        self.modelsList = [s.replace(".pickle", "") for s in onlyfiles
                           if ".pickle" in s and '~' not in s and '__L' not in s]
        if self.verbose:
            print 'Models available:                ' + ', '.join(self.modelsList)

        # likewise go to data folder and compile list of all folders and last time they were modified
        dataList = [f for f in listdir(self.dataPath) if isdir(join(self.dataPath, f))]
        if self.verbose:
            print "Data folders available:          " + ', '.join(dataList)

        # likewise parse training functions folder
        # OLD
        # self.functionsList = [f.replace(".py","") for f in listdir(self.trainingFunctionsPath)
        #                       if isfile(join(self.trainingFunctionsPath, f)) if ".py" in f if '~' not in f]
        # self.functionsList.sort()
        # NEW
        self.functionsList = []
        for importer, modname, ispkg in pkgutil.iter_modules(SAM.SAM_Drivers.__path__): 
           if 'SAMDriver_' in modname:
               self.functionsList += [modname]
        self.functionsList.sort()

        if self.verbose:
            print "Training functions available:    " + ', '.join(self.functionsList)

        # format of training functions is expected to be train_modelName_anythingElseToDistinguish
        # therefore data folders must contain .ini file pointing towards the preferred algorithm to be chosen
        model_params = ["model_options"]
        if self.verbose:
            print '-------------------'
            print 'Finding trainable data ...'
            print
        # exit if no training functions have been found
        if len(self.functionsList) == 0:
            if self.verbose:
                print "No training functions found. Exiting ..."
            return False
        else:
            self.trainableModels = []
            # check which data folders are trainable i.e training functions available
            for f in dataList:
                loc = join(self.dataPath, f)
                if self.verbose:
                    print "Checking " + loc + " ..."
                try:
                    parser = SafeConfigParser()
                    found = parser.read(loc + "/config.ini")
                    if not found:
                        if self.verbose:
                            print "config.ini not found for " + f
                        pass
                    else:
                        if parser.has_section(model_params[0]):
                            try:
                                # NEW
                                trainOptions = parser.get(model_params[0], 'driver').split(',')
                                # OLD
                                # trainOptions = parser.get(model_params[0], 'train').split(',')
                                # check training function exists
                                availableFuncs = [s for s in trainOptions for g in self.functionsList if s == g]
                                if len(availableFuncs) != 0:
                                    if self.verbose:
                                        print "Training functions for data " + f + " are " + ','.join(trainOptions)
                                        print "Corresponding functions available: " + ','.join(availableFuncs)
                                    
                                    if len(availableFuncs) > 1:
                                        if self.verbose:
                                            print "The first function will be chosen: " + availableFuncs[0]
                                    # find latest modified date of directory and subdirectories
                                    # thus checking for addition of new data
                                    t = []
                                    for dirName, dirs, filenames in os.walk(loc):
                                        t.append(os.path.getmtime(dirName))
                                    lastMod = max(t)
                                    if self.verbose: print "Data folder last modified: %s" % time.ctime(lastMod)
                                    # format of trainableModels is: dataFolder name, corresponding training function,
                                    # date data last modified, train boolean
                                    self.trainableModels += [[f, availableFuncs[0], lastMod, True]]
                                else:
                                    if self.verbose:
                                        print "Training functions for data " + f + \
                                                            " not found. Will not train " + f
                            except:
                                print "No option 'driver' in section: 'model_options' for " + f
                        else:
                            if self.verbose:
                                print "Training parameters for data " + f + " not found. Will not train " \
                                      + f + "\nCheck config.ini is formatted correctly"
                except IOError:
                    pass
                if self.verbose: print
            if self.verbose:
                print '-------------------'
                print 'Checking corresponding models'
                print
            # compare models and data folders. Assuming model names = folder names
            # check if model exists
            for f in self.trainableModels:
                t = []
                currModels = []
                for g in self.modelsList:
                    if(f[0]+'_' in g and '~' not in g):
                        #compare time of model and data
                        currModels.append(g)
                        g += ".pickle"
                        loc = join(self.modelPath,g)
                        t.append(os.path.getmtime(loc))
                if(len(t) > 0):
                    lastMod = max(t)
                    currModels = currModels[t.index(lastMod)]
                    self.trainableModels[self.trainableModels.index(f)].append(currModels)
                    if(self.verbose): print f[0] + " Model last modified: %s" % time.ctime(lastMod)
                    if(lastMod < f[2]):
                        tdiff =  datetime.datetime.fromtimestamp(f[2]).replace(microsecond = 0) - datetime.datetime.fromtimestamp(lastMod).replace(microsecond = 0)
                        if(self.verbose): print f[0] +' Model outdated by ' + str(tdiff) + '. Will be trained'
                    else:
                        if(self.verbose): print f[0] +' Model up-to-date'
                        f[3] = False
                else:
                    self.trainableModels[self.trainableModels.index(f)].append('')
                    if(self.verbose): print f[0] + ' Model not found. Training Required'
                if(self.verbose): print
            if(self.verbose):
                print '-------------------'
                print

            # provide option to train now or on close
            # if train now provide option to change experiment number or leave default
            self.updateModels = [s for s in self.trainableModels if s[3] == True if s[4] != '']
            self.updateModelsNames = [s[0] for s in self.trainableModels if s[3] == True if s[4] != '']

            self.noModels = [s for s in self.trainableModels if s[3] == True if s[4] == '']
            self.noModelsNames = [s[0] for s in self.trainableModels if s[3] == True if s[4] == '']

            self.uptodateModels = [s for s in self.trainableModels if s[3] == False]
            self.uptodateModelsNames = [s[0] for s in self.trainableModels if s[3] == False] 
            
            reply.addVocab(yarp.Vocab_encode("many"))
            reply.addString(str(len(self.uptodateModels)) + " Models up-to-date " + str(self.uptodateModelsNames))
            reply.addString(str(len(self.updateModels)) + " Models require an update " + str(self.updateModelsNames))
            reply.addString(str(len(self.noModels)) + " new models to train " + str(self.noModelsNames))
            reply.addString('')

            for j in self.updateModelsNames + self.uptodateModelsNames + self.noModelsNames:
                rep = yarp.Bottle()
                cmd = yarp.Bottle()
                cmd.addString('check')
                cmd.addString(j)
                self.checkModel(rep, cmd)
                a = str(rep.toString())
                reply.addString(a)

            return True

    def respond(self, command, reply):

        helpMessage = ["Commands are: ", "\tcheck_all", "\tcheck modelName", "\tclose modelName", \
                       "\tdelete modelName", "\thelp", "\tload modelName",  "\toptimise modelName", "\tquit", \
                       "\ttrain modelName", "\tlist_callSigns"]
        b = yarp.Bottle()
        self.checkAvailabilities(b)
        reply.clear()

        if command.get(0).asString() == "check_all":
            self.checkAvailabilities(reply)
        elif command.get(0).asString() == "check":
            self.checkModel(reply,command)
        elif command.get(0).asString() == "close":
            self.closeModel(reply,command)
        elif command.get(0).asString() == "delete":
            self.deleteModel(reply, command)
        elif command.get(0).asString() == "help":
            reply.addVocab(yarp.Vocab_encode("many"))
            for i in helpMessage:
                reply.addString(i)
        elif command.get(0).asString() == "load":
            self.loadModel(reply, command)
        elif command.get(0).asString() == "quit":
            reply.addString("quitting")
            return False 
        elif command.get(0).asString() == "train":
            self.train(reply, command)
        elif command.get(0).asString() == "optimise":
            self.optimise(reply, command)
        elif command.get(0).asString() == "list_callSigns":
            reply.addVocab(yarp.Vocab_encode("many"))
            for e in self.rpcConnections:
                repStr = e[0] + " Model: \t"
                for f in e[3]:
                    repStr += f + "\t"
                reply.addString(repStr)

        elif any(command.get(0).asString() in e[3] for e in self.rpcConnections):
            if 'instance' in command.get(0).asString() and command.size() != 2:
                reply.addString("Instance name required. e.g. ask_face_instance Daniel")
            else:
                try:
                    self.forwardCommand(command, reply)
                except utils.TimeoutError:
                    reply.addString('Failed to respond within timeout')

        else:
            reply.addVocab(yarp.Vocab_encode("many"))
            reply.addString("Wrong command. ")
            for i in helpMessage:
                reply.addString(i)
            reply.addString("Call signs available:")
            for e in self.rpcConnections:
                repStr = "\t" + e[0] + " Model: \t"
                for f in e[3]:
                    repStr += f + "\t"
                reply.addString(repStr)
        return True

    @utils.timeout(10)
    def forwardCommand(self, command, reply):
        for e in self.rpcConnections:
            if command.get(0).asString() in e[3]:
                e[1].write(command, reply)

    @staticmethod
    def signal_handler(signum, frame):
        raise Exception("Timed out!")

    def interruptModule(self):
        return True

    def closeModel(self, reply, command):

        if command.size() != 2:
            reply.addString("Model name required. e.g. close Actions")
        else:
            alreadyOpen = False
            conn = -1
            for k in range(len(self.rpcConnections)):
                if self.rpcConnections[k][0] == command.get(1).asString():
                    alreadyOpen = True
                    conn = k

            print "Already open = ", alreadyOpen
            if self.verbose:
                print command.get(1).asString()
            if alreadyOpen:
                self.rpcConnections[conn][1].write(yarp.Bottle('EXIT'), self.inputBottle)
                self.rpcConnections[conn][1].interrupt()
                time.sleep(1)
                self.rpcConnections[conn][1].close()
                time.sleep(1)
                self.rpcConnections[conn][4].send_signal(signal.SIGINT)
                self.rpcConnections[conn][4].wait()
                del self.rpcConnections[conn]
                reply.addString(command.get(1).asString() + " model closed.")
            else:
                reply.addString(command.get(1).asString() + " model is not running.")
        return True

    def loadModel(self, reply, command):
        parser = SafeConfigParser()

        if command.size() != 2:
            reply.addString("Model name required. e.g. load Actions")
        elif command.get(1).asString() in self.trainingListHandles:
            reply.addString("Cannot load model. Model in training")
        elif command.get(1).asString() in self.noModelsNames:
            reply.addString("Cannot load model. Model training available but not yet trained.")
        elif command.get(1).asString() in self.uptodateModelsNames+self.updateModelsNames:
            ret = parser.read(join(self.dataPath,command.get(1).asString(),"config.ini"))
            if len(ret) > 0:
                # OLD
                # if(parser.has_option('model_options', 'interaction')):
                #    interactionFunction = parser.get('model_options', 'interaction').split(',')
                # NEW
                if parser.has_option('model_options', 'driver') and parser.has_option('model_options', 'modelNameBase'):
                    interactionFunction = parser.get('model_options', 'driver').split(',')
                    modelNameBase = parser.get('model_options', 'modelNameBase')
                    
                    interactionFunction = [s for s in interactionFunction for g in self.functionsList if s == g]
                    if len(interactionFunction) != 0:
                        j = [s for s in self.trainableModels if s[0] == command.get(1).asString()][0]

                        interfacePortName = self.interactionParser.get(j[0],'rpcBase') + ':o'
                        callSignList = self.interactionParser.get(j[0],'callSign').replace(' ', '').split(',')
                        
                        # check if the model is already loaded
                        alreadyOpen = False
                        conn = -1
                        for k in range(len(self.rpcConnections)):
                            if self.rpcConnections[k][0] == j[0]:
                                alreadyOpen = True
                                conn = k

                        print "Loading ", interfacePortName, " with ", callSignList
                        if alreadyOpen:
                            if self.verbose:
                                print "Model already open"
                            # check it is functioning correctly
                            correctOperation = True if self.rpcConnections[conn][1].getOutputCount() > 0 else False
                            if self.verbose:
                                print "correct operation = ", correctOperation
                                print 
                        else:
                            if self.verbose:
                                print "Model not open"
                        
                        if alreadyOpen and correctOperation:
                            rep = yarp.Bottle()
                            cmd = yarp.Bottle()
                            cmd.addString("reload")
                            # print self.rpcConnections[conn][0], 'reload'
                            self.rpcConnections[conn][1].write(cmd, rep)
                            if rep.get(0).asString() == 'ack':
                                reply.addString(command.get(1).asString() + " model re-loaded correctly")
                            else:
                                reply.addString(command.get(1).asString() + " model did not re-loaded correctly")
                        elif alreadyOpen and not correctOperation:
                            # terminate model by finding process in self.rpcConnections[4]
                            alreadyOpen = False
                            rep = yarp.Bottle()
                            cmd = yarp.Bottle()
                            cmd.addString("close")
                            cmd.addString(command.get(1).asString())
                            self.closeModel(rep, cmd)
                            reply.addString(command.get(1).asString() + " model terminated ")

                        if not alreadyOpen:
                            interfacePort = yarp.RpcClient()
                            interfacePort.open(interfacePortName)
                            
                            # OLD
                            # args = ' '.join([join(self.dataPath,j[0]), join(self.modelPath, j[4]),
                            #                  self.interactionConfFile])
                            # cmd = 'ipython ' + join(self.trainingFunctionsPath, interactionFunction[0]+'.py') + \
                            #       ' -- ' + args
                            # NEW
                            args = ' '.join([join(self.dataPath, j[0]), join(self.modelPath, j[4]),
                                             self.interactionConfFile, interactionFunction[0]])
                            cmd = 'interactionSAMModel.py' + ' -- ' + args

                            if self.verbose:
                                print
                                print "cmd = ", cmd
                                print
                            if self.persistence:
                                command = "bash -c \"" + cmd + "; exec bash\""
                            else:
                                command = "bash -c \"" + cmd + "\""

                            if self.windowed:
                                c = subprocess.Popen(['xterm', '-e', command], shell=False)
                            else:
                                c = subprocess.Popen([cmd], shell=True, stdout=self.devnull, stderr=self.devnull)

                            self.rpcConnections.append([j[0], interfacePort, interfacePortName[:-1], callSignList, c])
                            # pause here

                            noConn = True
                            iters = 0
                            if self.verbose:
                                print 'connecting ' + self.rpcConnections[-1][2]+'o' + \
                                      ' with ' + self.rpcConnections[-1][2]+'i'
                            while noConn:
                                noConn = yarp.Network.connect(self.rpcConnections[-1][2]+'o',
                                                              self.rpcConnections[-1][2]+'i')
                                noConn = not noConn
                                time.sleep(1)
                                iters += 1
                                if iters >= 20:
                                    break

                            if noConn:
                                reply.addString("Failure to load " + str(interactionFunction[0]) + " model")
                                rep = yarp.Bottle()
                                cmd = yarp.Bottle()
                                cmd.addString("close")
                                cmd.addString(self.rpcConnections[-1][0])
                                self.closeModel(rep, cmd)
                            else:
                                # then execute an interaction model check to verify correct startup
                                reply.addString(str(interactionFunction[0]) + " model loaded at " +
                                                interfacePortName + " with call signs " + str(callSignList))
                    else:
                        reply.addString('No interaction function found in ' + command.get(1).asString() +
                                        ' model path. Skipping model')
                else:
                    reply.addString('Parameters "driver" and "modelBaseName" not found in config.ini')
            else:
                reply.addString("Failed to retrieve " + command.get(1).asString() + " model. Model not trained")
        else:
            reply.addString(command.get(1).asString() + " model does not exist")
        del parser

    def checkModel(self, reply, command):
        reply.clear()
        #update to show which models are loaded or not and which are currently training
        repStr = ''
        if(command.size() != 2):
            repStr += "Model name required. e.g. check Actions"
        elif(command.get(1).asString() in self.trainingListHandles):
            repStr += command.get(1).asString()+" in training"
        elif(command.get(1).asString() in self.uptodateModelsNames):
            repStr += command.get(1).asString()+" is up-to-date"
        elif(command.get(1).asString() in self.updateModelsNames):
            repStr += command.get(1).asString()+" requires update"
        elif(command.get(1).asString() in self.noModelsNames):
            repStr += command.get(1).asString()+" has no model"
        else:
            repStr += command.get(1).asString()+" model not present"

        if(any(e[0] == command.get(1).asString() for e in self.rpcConnections)):
            repStr += " and is loaded"
        elif(command.get(1).asString() in self.uptodateModelsNames + self.updateModelsNames):
            repStr += " and is not loaded"
        reply.addString(repStr)
        return True
        
    def train(self, reply, command):
        reply.clear()
        
        if(command.size() != 2):
            reply.addString("Model name required. e.g. train Actions")
        elif(str(command.get(1).asString()) in self.uptodateModelsNames):
            reply.addString(command.get(1).asString() + " is already up to date.")
        elif(str(command.get(1).asString()) in self.trainingListHandles):
            reply.addString(command.get(1).asString() + " is already being trained.")
        elif(command.get(1).asString() in self.updateModelsNames or command.get(1).asString() in self.noModelsNames):
            reply.addString("Training " + command.get(1).asString() + " model ...")
            modelToTrain = [s for s in self.updateModels + self.noModels if s[0] == command.get(1).asString()][0]
            if(self.verbose): print modelToTrain
            self.train_model(modelToTrain)
        else:
            reply.addString(command.get(1).asString() + " model not available to train")
        
        return True

    def optimise(self, reply, command):
        reply.clear()
        
        if(command.size() != 2):
            reply.addString("Model name required. e.g. optimise Actions")
        elif(str(command.get(1).asString()) in self.trainingListHandles):
            reply.addString(command.get(1).asString() + " is already being trained.")
        elif(command.get(1).asString() in self.noModelsNames):
            reply.addString("Train " + command.get(1).asString() + " model before optimising")
        elif(command.get(1).asString() in self.updateModelsNames or command.get(1).asString() in self.uptodateModelsNames):
            reply.addString("Optimising " + command.get(1).asString() + " model ...")
            modelToTrain = [s for s in self.updateModels + self.uptodateModels if s[0] == command.get(1).asString()][0]
            if(self.verbose): print modelToTrain
            self.optimise_model(modelToTrain)
        else:
            reply.addString(command.get(1).asString() + " model not available to optimise")
        
        return True

    def deleteModel(self, reply, command):
        reply.clear()

        alreadyOpen = False
        for k in self.rpcConnections:
            if(k[0] == command.get(1).asString()):
                alreadyOpen = True
        
        if(command.size() != 2):
            reply.addString("Model name required. e.g. delete Actions")
        elif(command.get(1).asString() in self.trainingListHandles):
            reply.addString("Cannot delete model. Model in training")
        elif(alreadyOpen):
            reply.addString("Cannot delete model. Model currently loaded")
        elif(command.get(1).asString() in self.updateModelsNames or command.get(1).asString() in self.uptodateModelsNames):
            reply.addString(str(command.get(1).asString()) + " model deleted.")
            modelToDelete = [s for s in self.updateModels + self.uptodateModels if s[0] == command.get(1).asString()][0][4]
            if('L' in  modelToDelete.split('__')[-1]):
                modelToDelete = '__'.join(modelToDelete.split('__')[:-1])
            print modelToDelete
            filesToDelete = glob.glob(join(self.modelPath, modelToDelete + '*'))

            for i in filesToDelete:
                os.remove(i)

            b = yarp.Bottle()
            self.checkAvailabilities(b)
        else:
            reply.addString(str(command.get(1).asString()) + " model not present")
        return True

    def train_model(self, mod):
        if(self.verbose):
            print "Training Models:"
            print

        #OLD
        #n = mod[1] + '.py'
        #trainPath = join(self.trainingFunctionsPath, n) 
        #NEW
        trainPath = 'trainSAMModel.py'

        if(self.verbose):
            print 'Training ' + mod[0] + ' ...'
            print 'Opening ' + trainPath
            print
        dPath = join(self.dataPath, mod[0])
        if(mod[4] != ''):
            mPath = join(self.modelPath, mod[4]) + '.pickle'
        else:
            mPath = join(self.modelPath, mod[4])

        if(self.verbose): print mPath

        # #open separate ipython for training
        # #this will allow separate training across different computers in future
        #OLD
        # if(mod[0] in self.updateModelsNames):
        #      args = ' '.join([dPath, mPath, mod[1], 'update'])
        # else:
        #      args = ' '.join([dPath, mPath, mod[1], 'new'])
        #NEW
        if(mod[0] in self.updateModelsNames):
           args = ' '.join([dPath, mPath, mod[1], 'update', mod[0]])
        else:
           args = ' '.join([dPath, mPath, mod[1], 'new', mod[0]])
		
        if(self.verbose): print 'args: ', args

        #OLD
        #cmd = 'ipython ' + trainPath + ' -- ' + args
        #NEW
        cmd = trainPath + ' -- ' + args
        if(self.persistence):
            command = "bash -c \"" + cmd + "; exec bash\""
        else:
            command = "bash -c \"" + cmd + "\""
        
        if(self.verbose): print 'cmd: ', cmd

        if(self.windowed):
            c = subprocess.Popen([self.terminal, '-e', command], shell=False)
        else:
            c = subprocess.Popen([cmd], shell=True, stdout=self.devnull, stderr=self.devnull)
        
        self.trainingListHandles[mod[0]] = c

        return True

    def optimise_model(self, mod):
        if(self.verbose):
            print "Training Models:"
            print

        # OLD
        # n = mod[1] + '.py'
        # trainPath = join(self.trainingFunctionsPath, n) 
        # NEW
        trainPath = 'trainSAMModel.py'

        if(self.verbose):
            print 'Optimising ' + mod[0] + ' ...'
            print
        dPath = join(self.dataPath, mod[0])
        if(mod[4] != ''):
            mPath = join(self.modelPath, mod[4]) + '.pickle'
        else:
            mPath = join(self.modelPath, mod[4])

        if(self.verbose): print mPath

        # #open separate ipython for training
        # #this will allow separate training across different computers in future
        #OLD
        #args = ' '.join([dPath, mPath, mod[1], 'new', 'False', 'False', 'True'])
        #NEW
        args = ' '.join([dPath, mPath, mod[1], 'new', mod[0], 'False', 'False', 'True'])
        
        if(self.verbose): print 'args: ', args

        cmd = 'samOptimiser.py ' + trainPath + ' ' + args
        if(self.persistence):
            command = "bash -c \"" + cmd + "; exec bash\""
        else:
            command = "bash -c \"" + cmd + "\""
        
        if(self.verbose): print 'cmd: ', cmd

        c = subprocess.Popen([self.terminal, '-e', command], shell=False)
        self.trainingListHandles[mod[0]] = c

        return True

    def getPeriod(self):
        return 0.1

    def onlineModelCheck(self):
        # check communication with loaded models
        readyList = []
        for i, v in self.trainingListHandles.iteritems():
            if i != 'Cluster':
                ret = v.poll()
                if ret is not None:
                    if ret == 0:
                        readyList += [i]
                        print i, 'terminated successfully'
                        b = yarp.Bottle()
                        self.checkAvailabilities(b)
                    else:
                        readyList += [i]
                        print i, 'terminated with ', self.SIGNALS_TO_NAMES_DICT[abs(ret)]
                else:
                    # if(self.verbose): print i, "still training "
                    pass
        
        for i in readyList:
            del self.trainingListHandles[i]

    def updateModule(self):
        if self.iter == 10:
            self.onlineModelCheck()
            self.iter = 0
        self.iter += 1
        time.sleep(0.05)
        return True

if __name__ == '__main__':

    plt.ion()
    yarp.Network.init() 
    mod = SamSupervisorModule()
    rf = yarp.ResourceFinder()
    rf.setVerbose(True)
    rf.setDefaultContext("samSupervisor")
    rf.setDefaultConfigFile("default.ini")
    rf.configure(sys.argv)

    mod.runModule(rf)
