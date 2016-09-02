#!/usr/bin/env ipython
import matplotlib.pyplot as plt
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
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

=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
from SAM.SAM_Core import SAMCore
from SAM.SAM_Drivers import SAMDriver_interaction
import pylab as pb
import sys 
from sys import executable
import subprocess
from subprocess import Popen, PIPE
import pickle
import shlex
import os
from os import listdir, walk, system
from os.path import isfile, join, isdir
import glob
import pkgutil
import numpy
import time
import operator
import numpy as np
import datetime
import signal
import yarp
from ConfigParser import SafeConfigParser

class SamSupervisorModule(yarp.RFModule):

<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
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

<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
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
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
            persistence = rf.check("persistence", yarp.Value("False")).asString()
            windowed = rf.check("windowed", yarp.Value("True")).asString()
            verbose = rf.check("verbose", yarp.Value("True")).asString()
=======
            persistence = rf.check("persistence",yarp.Value("False")).asString()
            windowed = rf.check("windowed",yarp.Value("True")).asString()
            verbose = rf.check("verbose",yarp.Value("True")).asString()
>>>>>>> New version of samSupervisor with rpc interface and README
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
            persistence = rf.check("persistence",yarp.Value("False")).asString()
            windowed = rf.check("windowed",yarp.Value("True")).asString()
            verbose = rf.check("verbose",yarp.Value("True")).asString()
=======
            persistence = rf.check("persistence", yarp.Value("False")).asString()
            windowed = rf.check("windowed", yarp.Value("True")).asString()
            verbose = rf.check("verbose", yarp.Value("True")).asString()
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
            persistence = rf.check("persistence", yarp.Value("False")).asString()
            windowed = rf.check("windowed", yarp.Value("True")).asString()
            verbose = rf.check("verbose", yarp.Value("True")).asString()
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs

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
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
            # self.trainingFunctionsPath = os.environ.get("WYSIWYD_DIR")+"/bin"
            #NEW
            self.trainingFunctionsPath = SAM.SAM_Drivers.__path__
=======
            self.trainingFunctionsPath = os.environ.get("WYSIWYD_DIR")+"/bin"
            #NEW
			#self.trainingFunctionsPath = SAM.SAM_Drivers.__path__
>>>>>>> New version of samSupervisor with rpc interface and README
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
            self.trainingFunctionsPath = os.environ.get("WYSIWYD_DIR")+"/bin"
            #NEW
			#self.trainingFunctionsPath = SAM.SAM_Drivers.__path__
=======
            # self.trainingFunctionsPath = os.environ.get("WYSIWYD_DIR")+"/bin"
            #NEW
            self.trainingFunctionsPath = SAM.SAM_Drivers.__path__
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
            # self.trainingFunctionsPath = os.environ.get("WYSIWYD_DIR")+"/bin"
            #NEW
            self.trainingFunctionsPath = SAM.SAM_Drivers.__path__
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
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

<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
            if self.windowed:
=======
            if(self.windowed):
>>>>>>> New version of samSupervisor with rpc interface and README
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
            if(self.windowed):
=======
            if self.windowed:
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
            if self.windowed:
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                c = subprocess.Popen([self.terminal, '-e', command], shell=False)
            else:
                c = subprocess.Popen([cmd], shell=True, stdout=self.devnull, stderr=self.devnull)

            self.trainingListHandles['Cluster'] = c

<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
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
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
            if(len(self.uptodateModels) + len(self.updateModels) > 0):
                if(self.verbose): print "Loading models according to " + self.interactionConfPath
                #start loading model configuration according to interactionConfPath file
                
                rfModel = yarp.ResourceFinder()
                rfModel.setVerbose(self.verbose);
                rfModel.setDefaultContext("samSupervisor");
                self.interactionConfFile = rfModel.findFile(self.interactionConfPath);
                
                #Iterate over all sections within the interactionConfPath,
                #create a list and check against the available models
                #warn if model specified in interactionConfPath not loadable
                self.interactionParser = SafeConfigParser()
                self.interactionParser.read(self.interactionConfFile)
                self.interactionSectionList = self.interactionParser.sections()
                if(self.verbose):
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
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
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                    print
                    print self.dataPath
                    print self.interactionSectionList
                    print
                for j in self.interactionSectionList:
                    command = yarp.Bottle()
                    command.addString("load")
                    command.addString(j)
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
                    if self.verbose:
                        print command.toString()
                    reply = yarp.Bottle()

                    self.loadModel(reply, command)
                    if self.verbose:
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
                    if(self.verbose): print command.toString()
                    reply = yarp.Bottle()

                    self.loadModel(reply, command)
                    if(self.verbose):
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                    if self.verbose:
                        print command.toString()
                    reply = yarp.Bottle()

                    self.loadModel(reply, command)
                    if self.verbose:
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                        print reply.toString()
                        print "-----------------------------------------------"
                        print
                
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
            elif len(self.noModels) > 0:
                if self.verbose:
                    print "Models available for training."
                # Train a model according to ineractionConfPath file
            else:
                if self.verbose:
                    print "No available models to load or train"
                # wait for a training command
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
            elif(len(self.noModels) > 0):
                if(self.verbose):print "Models available for training."
                #Train a model according to ineractionConfPath file
            else:
                if(self.verbose):print "No available models to load or train" 
                #wait for a training command
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
            elif len(self.noModels) > 0:
                if self.verbose:
                    print "Models available for training."
                # Train a model according to ineractionConfPath file
            else:
                if self.verbose:
                    print "No available models to load or train"
                # wait for a training command
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                
            return True

    def close(self):
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
        # close ports of loaded models
=======
        #for i in self.rpcConnections
        #close ports of loaded models
>>>>>>> New version of samSupervisor with rpc interface and README
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
        #for i in self.rpcConnections
        #close ports of loaded models
=======
        # close ports of loaded models
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
        # close ports of loaded models
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
        for j in self.rpcConnections:
            j[1].write(yarp.Bottle('EXIT'), self.inputBottle)
            j[1].close()

        self.supervisorPort.close()

<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
        for i, v in self.trainingListHandles.iteritems():
=======
        for i,v in self.trainingListHandles.iteritems():
>>>>>>> New version of samSupervisor with rpc interface and README
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
        for i,v in self.trainingListHandles.iteritems():
=======
        for i, v in self.trainingListHandles.iteritems():
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
        for i, v in self.trainingListHandles.iteritems():
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
            v.send_signal(signal.SIGINT)
            v.wait()   

        for v in self.rpcConnections:
            v[4].send_signal(signal.SIGINT)
            v[4].wait() 

<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
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
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
    def checkAvailabilities(self,reply):
        #after finding the root path, go to models folder and compile list of all
        #models together with the last time they were modified
        onlyfiles = [f for f in listdir(self.modelPath) if isfile(join(self.modelPath, f))]

        #find number of .pickle files
        self.modelsList = [s.replace(".pickle","") for s in onlyfiles if ".pickle" in s if '~' not in s]
        if(self.verbose): print 'Models available:                ' + ', '.join(self.modelsList)

        #likewise go to data folder and compile list of all folders and last time they were modified
        dataList = [f for f in listdir(self.dataPath) if isdir(join(self.dataPath, f))]
        if(self.verbose): print "Data folders available:          " + ', '.join(dataList)

        #likewise parse training functions folder
        #OLD
        self.functionsList = [f.replace(".py","") for f in listdir(self.trainingFunctionsPath) if isfile(join(self.trainingFunctionsPath, f)) if ".py" in f if '~' not in f]
        self.functionsList.sort()
		#NEW
        #self.functionsList = []
        #for importer, modname, ispkg in pkgutil.iter_modules(SAM.SAM_Drivers.__path__): 
        #    if('SAMDriver_' in modname):
        #        self.functionsList += [modname]
        #self.functionsList.sort()

        if(self.verbose): print "Training functions available:    " + ', '.join(self.functionsList)

        #format of training functions is expected to be train_modelName_anythingElseToDistinguish
        #therefore data folders must contain .ini file pointing towards the preferred algorithm to be chosen
        model_params = ["model_options"]
        if(self.verbose):
            print '-------------------'
            print 'Finding trainable data ...'
            print
        #exit if no training functions have been found
        if(len(self.functionsList) == 0):
            if(self.verbose): print "No training functions found. Exiting ..."
            return False
        else:
            self.trainableModels = []
            #check which data folders are trainable i.e training functions available
            for f in dataList:
                loc = join(self.dataPath, f)
                if(self.verbose): print "Checking " + loc + " ..."
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
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
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                try:
                    parser = SafeConfigParser()
                    found = parser.read(loc + "/config.ini")
                    if not found:
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
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
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
                        if(self.verbose): print "config.ini not found for " + f
                        pass
                    else:
                        if( parser.has_section(model_params[0]) == True ):
                            try:
                                #NEWW
                                #trainOptions = parser.get(model_params[0], 'driver').split(',')
                                #OLD
                                trainOptions = parser.get(model_params[0], 'train').split(',')
                                #check training function exists 
                                availableFuncs = [s for s in trainOptions for g in self.functionsList if s == g]
                                if(len(availableFuncs) != 0):
                                    if(self.verbose): print "Training functions for data " + f + " are " + ','.join(trainOptions)
                                    if(self.verbose): print "Corresponding functions available: " + ','.join(availableFuncs)
                                    
                                    if(len(availableFuncs) > 1):
                                        if(self.verbose): print "The first function will be chosen: " + availableFuncs[0]
                                    #find latest modified date of directory and subdirectories thus checking for addition of new data
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
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
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                                    t = []
                                    for dirName, dirs, filenames in os.walk(loc):
                                        t.append(os.path.getmtime(dirName))
                                    lastMod = max(t)
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
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
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
                                    if(self.verbose): print "Data folder last modified: %s" % time.ctime(lastMod)
                                    #format of trainableModels is: dataFolder name, correspoding training function, date data last modified, train boolean
                                    self.trainableModels += [[f, availableFuncs[0], lastMod, True]]
                                else:
                                    if(self.verbose): print "Training functions for data " + f + " not found. Will not train " + f 
                            except:
                                print "No option 'driver' in section: 'model_options' for " + f
                        else:
                            if(self.verbose): print "Training parameters for data " + f + " not found. Will not train " + f + "\nCheck config.ini is formatted correctly"
                except IOError:
                    pass
                if(self.verbose): print 
            if(self.verbose):
                print '-------------------'
                print 'Checking corresponding models'
                print
            #compare models and data folders. Assuming model names = folder names
            #check if model exists
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
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
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
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

<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
            # provide option to train now or on close
            # if train now provide option to change experiment number or leave default
=======
            #provide option to train now or on close 
            #if train now provide option to change experiment number or leave default
>>>>>>> New version of samSupervisor with rpc interface and README
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
            #provide option to train now or on close 
            #if train now provide option to change experiment number or leave default
=======
            # provide option to train now or on close
            # if train now provide option to change experiment number or leave default
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
            # provide option to train now or on close
            # if train now provide option to change experiment number or leave default
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
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
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
                       "\tdelete modelName", "\thelp", "\tload modelName",  "\toptimise modelName", "\tquit", \
                       "\ttrain modelName", "\tlist_callSigns"]
=======
                      "\tdelete modelName", "\thelp", "\tload modelName",  "\toptimise modelName", "\tquit", \
                      "\ttrain modelName", "\tlist_callSigns"]
>>>>>>> New version of samSupervisor with rpc interface and README
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
                      "\tdelete modelName", "\thelp", "\tload modelName",  "\toptimise modelName", "\tquit", \
                      "\ttrain modelName", "\tlist_callSigns"]
=======
                       "\tdelete modelName", "\thelp", "\tload modelName",  "\toptimise modelName", "\tquit", \
                       "\ttrain modelName", "\tlist_callSigns"]
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
                       "\tdelete modelName", "\thelp", "\tload modelName",  "\toptimise modelName", "\tquit", \
                       "\ttrain modelName", "\tlist_callSigns"]
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
        b = yarp.Bottle()
        self.checkAvailabilities(b)
        reply.clear()

<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
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
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
        if(command.get(0).asString() == "check_all"):
            self.checkAvailabilities(reply)
        elif(command.get(0).asString() == "check"):
            self.checkModel(reply,command)
        elif(command.get(0).asString() == "close"):
            self.closeModel(reply,command)
        elif(command.get(0).asString() == "delete"):
            self.deleteModel(reply, command)
        elif(command.get(0).asString() == "help"):
            reply.addVocab(yarp.Vocab_encode("many"))
            for i in helpMessage:
                reply.addString(i)
        elif(command.get(0).asString() == "load"):
            self.loadModel(reply, command)
        elif (command.get(0).asString() == "quit"):
            reply.addString("quitting")
            return False 
        elif(command.get(0).asString() == "train"):
            self.train(reply, command)
        elif(command.get(0).asString() == "optimise"):
            self.optimise(reply, command)
        elif(command.get(0).asString() == "list_callSigns"):
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
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
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
            reply.addVocab(yarp.Vocab_encode("many"))
            for e in self.rpcConnections:
                repStr = e[0] + " Model: \t"
                for f in e[3]:
                    repStr += f + "\t"
                reply.addString(repStr)

<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
        elif any(command.get(0).asString() in e[3] for e in self.rpcConnections):
            try:
                self.forwardCommand(command, reply)
            except utils.TimeoutError:
                reply.addString('Failed to respond within timeout')

=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
        elif(any(command.get(0).asString() in e[3] for e in self.rpcConnections)):
            for e in self.rpcConnections:
                if(command.get(0).asString() in e[3]):
                    e[1].write(command, reply)
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
        elif any(command.get(0).asString() in e[3] for e in self.rpcConnections):
            try:
                self.forwardCommand(command, reply)
            except utils.TimeoutError:
                reply.addString('Failed to respond within timeout')

<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
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

<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
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
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
    def interruptModule(self):
        return True

    def closeModel(self,reply,command):

        if(command.size() != 2):
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
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
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
            reply.addString("Model name required. e.g. close Actions")
        else:
            alreadyOpen = False
            conn = -1
            for k in range(len(self.rpcConnections)):
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
                if self.rpcConnections[k][0] == command.get(1).asString():
=======
                if(self.rpcConnections[k][0] == command.get(1).asString()):
>>>>>>> New version of samSupervisor with rpc interface and README
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
                if(self.rpcConnections[k][0] == command.get(1).asString()):
=======
                if self.rpcConnections[k][0] == command.get(1).asString():
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
                if self.rpcConnections[k][0] == command.get(1).asString():
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                    alreadyOpen = True
                    conn = k

            print "Already open = ", alreadyOpen
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
            if self.verbose:
                print command.get(1).asString()
            if alreadyOpen:
                self.rpcConnections[conn][1].write(yarp.Bottle('EXIT'), self.inputBottle)
                self.rpcConnections[conn][1].interrupt()
                time.sleep(1)
=======
            if(self.verbose): print command.get(1).asString()
            if(alreadyOpen):
                self.rpcConnections[conn][1].write(yarp.Bottle('EXIT'), self.inputBottle)
>>>>>>> New version of samSupervisor with rpc interface and README
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
            if(self.verbose): print command.get(1).asString()
            if(alreadyOpen):
                self.rpcConnections[conn][1].write(yarp.Bottle('EXIT'), self.inputBottle)
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
            if self.verbose:
                print command.get(1).asString()
            if alreadyOpen:
                self.rpcConnections[conn][1].write(yarp.Bottle('EXIT'), self.inputBottle)
                self.rpcConnections[conn][1].interrupt()
                time.sleep(1)
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                self.rpcConnections[conn][1].close()
                time.sleep(1)
                self.rpcConnections[conn][4].send_signal(signal.SIGINT)
                self.rpcConnections[conn][4].wait()
                del self.rpcConnections[conn]
                reply.addString(command.get(1).asString() + " model closed.")
            else:
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
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
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
                reply.addString(command.get(1).asString() +" model is not running.")
        return True

    def loadModel(self,reply,command):
        parser = SafeConfigParser()

        if(command.size() != 2):
            reply.addString("Model name required. e.g. load Actions")
        elif(command.get(1).asString() in self.trainingListHandles):
            reply.addString("Cannot load model. Model in training")
        elif(command.get(1).asString() in self.noModelsNames):
            reply.addString("Cannot load model. Model training available but not yet trained.")
        elif(command.get(1).asString() in self.uptodateModelsNames+self.updateModelsNames):
            ret = parser.read(join(self.dataPath,command.get(1).asString(),"config.ini"))
            if(len(ret) > 0): 
                #OLD
                if(parser.has_option('model_options', 'interaction')):
                    interactionFunction = parser.get('model_options', 'interaction').split(',')
                #NEW
                # if(parser.has_option('model_options', 'driver') and parser.has_option('model_options', 'modelNameBase')):
                #     interactionFunction = parser.get('model_options', 'driver').split(',')
                #     modelNameBase = parser.get('model_options', 'modelNameBase')
                    
                    interactionFunction = [s for s in interactionFunction for g in self.functionsList if s == g]
                    if(len(interactionFunction) != 0):
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
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
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                        j = [s for s in self.trainableModels if s[0] == command.get(1).asString()][0]

                        interfacePortName = self.interactionParser.get(j[0],'rpcBase') + ':o'
                        callSignList = self.interactionParser.get(j[0],'callSign').replace(' ', '').split(',')
                        
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
                        # check if the model is already loaded
                        alreadyOpen = False
                        conn = -1
                        for k in range(len(self.rpcConnections)):
                            if self.rpcConnections[k][0] == j[0]:
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
                        #check if the model is already loaded
                        alreadyOpen = False
                        conn = -1
                        for k in range(len(self.rpcConnections)):
                            if(self.rpcConnections[k][0] == j[0]):
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                        # check if the model is already loaded
                        alreadyOpen = False
                        conn = -1
                        for k in range(len(self.rpcConnections)):
                            if self.rpcConnections[k][0] == j[0]:
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                                alreadyOpen = True
                                conn = k

                        print "Loading ", interfacePortName, " with ", callSignList
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
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
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
                        if(alreadyOpen):
                            if(self.verbose):print "Model already open"
                            #check it is functioning correctly
                            correctOperation = True if self.rpcConnections[conn][1].getOutputCount() > 0 else False
                            if(self.verbose):
                                print "correct operation = ", correctOperation
                                print 
                        else:
                            if(self.verbose):print "Model not open"
                        
                        if(alreadyOpen and correctOperation):
                            #send a message to the interaction model to check version of currently loaded model
                            #and compare it with that stored on disk. If model on disk is more recent reload model
                            #interaction model to return "model re-loaded correctly" or "loaded model already up to date"
                            reply.addString(command.get(1).asString() + " model re-loaded correctly")
                        elif(alreadyOpen and not correctOperation):
                            #terminate model by finding process in self.rpcConnections[4]
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
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
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                            alreadyOpen = False
                            rep = yarp.Bottle()
                            cmd = yarp.Bottle()
                            cmd.addString("close")
                            cmd.addString(command.get(1).asString())
                            self.closeModel(rep, cmd)
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
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
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder

                            reply.addString(command.get(1).asString() + " model terminated ")

                        if(not alreadyOpen):
                            interfacePort = yarp.RpcClient()
                            interfacePort.open(interfacePortName)
                            
                            #OLD
                            args = ' '.join([join(self.dataPath,j[0]), join(self.modelPath, j[4]), self.interactionConfFile])
                            cmd = 'ipython ' + join(self.trainingFunctionsPath, interactionFunction[0]+'.py') + ' -- ' + args
                            #NEW
                            #args = ' '.join([join(self.dataPath,j[0]), join(self.modelPath, j[4]), self.interactionConfFile, interactionFunction])
                            #cmd = 'interactionSAMModel.py' + ' -- ' + args

                            if(self.verbose):
                                print
                                print "args = ", args
                                print
                            if(self.persistence):
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
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
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                                command = "bash -c \"" + cmd + "; exec bash\""
                            else:
                                command = "bash -c \"" + cmd + "\""

<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
                            if self.windowed:
=======
                            if(self.windowed):
>>>>>>> New version of samSupervisor with rpc interface and README
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
                            if(self.windowed):
=======
                            if self.windowed:
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
                            if self.windowed:
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                                c = subprocess.Popen(['xterm', '-e', command], shell=False)
                            else:
                                c = subprocess.Popen([cmd], shell=True, stdout=self.devnull, stderr=self.devnull)

                            self.rpcConnections.append([j[0], interfacePort, interfacePortName[:-1], callSignList, c])
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
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
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
                            #pause here

                            noConn = True
                            iters = 0
                            if(self.verbose): print 'connecting ' + self.rpcConnections[-1][2]+'o ' + 'with ' + self.rpcConnections[-1][2]+'i'
                            while(noConn):
                                noConn = yarp.Network.connect(self.rpcConnections[-1][2]+'o',self.rpcConnections[-1][2]+'i')
                                noConn = not noConn
                                time.sleep(1)
                                iters += 1
                                if(iters >= 20):
                                    break

                            if(noConn):
                                reply.addString("Failure to load " + str(interactionFunction) +" model")
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
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
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                                rep = yarp.Bottle()
                                cmd = yarp.Bottle()
                                cmd.addString("close")
                                cmd.addString(self.rpcConnections[-1][0])
                                self.closeModel(rep, cmd)
                            else:
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
                                # then execute an interaction model check to verify correct startup
                                reply.addString(str(interactionFunction[0]) + " model loaded at " +
                                                interfacePortName + " with call signs " + str(callSignList))
                    else:
                        reply.addString('No interaction function found in ' + command.get(1).asString() +
                                        ' model path. Skipping model')
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
                                #then execute an interaction model check to verify correct startup
                                reply.addString(str(interactionFunction) +" model loaded at " + interfacePortName + " with call signs " + str(callSignList))
                    else:
                        reply.addString('No interaction function found in '+command.get(1).asString()+ ' model path. Skipping model')
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                                # then execute an interaction model check to verify correct startup
                                reply.addString(str(interactionFunction[0]) + " model loaded at " +
                                                interfacePortName + " with call signs " + str(callSignList))
                    else:
                        reply.addString('No interaction function found in ' + command.get(1).asString() +
                                        ' model path. Skipping model')
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
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
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
        #n = mod[1] + '.py'
        #trainPath = join(self.trainingFunctionsPath, n) 
        #NEW
        trainPath = 'trainSAMModel.py'
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
        n = mod[1] + '.py'
        trainPath = join(self.trainingFunctionsPath, n) 
        #NEW
        #trainPath = 'trainSAMModel.py'
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
        #n = mod[1] + '.py'
        #trainPath = join(self.trainingFunctionsPath, n) 
        #NEW
        trainPath = 'trainSAMModel.py'
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs

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
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
        # if(mod[0] in self.updateModelsNames):
        #      args = ' '.join([dPath, mPath, mod[1], 'update'])
        # else:
        #      args = ' '.join([dPath, mPath, mod[1], 'new'])
        #NEW
        if(mod[0] in self.updateModelsNames):
           args = ' '.join([dPath, mPath, mod[1], 'update', mod[0]])
        else:
           args = ' '.join([dPath, mPath, mod[1], 'new', mod[0]])
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
        if(mod[0] in self.updateModelsNames):
             args = ' '.join([dPath, mPath, mod[1], 'update'])
        else:
             args = ' '.join([dPath, mPath, mod[1], 'new'])
        #NEW
        #if(mod[0] in self.updateModelsNames):
        #    args = ' '.join([dPath, mPath, mod[1], 'update', mod[0]])
        #else:
        #    args = ' '.join([dPath, mPath, mod[1], 'new', mod[0]])
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
        # if(mod[0] in self.updateModelsNames):
        #      args = ' '.join([dPath, mPath, mod[1], 'update'])
        # else:
        #      args = ' '.join([dPath, mPath, mod[1], 'new'])
        #NEW
        if(mod[0] in self.updateModelsNames):
           args = ' '.join([dPath, mPath, mod[1], 'update', mod[0]])
        else:
           args = ' '.join([dPath, mPath, mod[1], 'new', mod[0]])
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
		
        if(self.verbose): print 'args: ', args

        #OLD
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
        #cmd = 'ipython ' + trainPath + ' -- ' + args
        #NEW
        cmd = trainPath + ' -- ' + args
=======
        cmd = 'ipython ' + trainPath + ' -- ' + args
        #NEW
        #cmd = trainPath + ' -- ' + args
>>>>>>> New version of samSupervisor with rpc interface and README
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
        cmd = 'ipython ' + trainPath + ' -- ' + args
        #NEW
        #cmd = trainPath + ' -- ' + args
=======
        #cmd = 'ipython ' + trainPath + ' -- ' + args
        #NEW
        cmd = trainPath + ' -- ' + args
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
        #cmd = 'ipython ' + trainPath + ' -- ' + args
        #NEW
        cmd = trainPath + ' -- ' + args
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
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

<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
        # OLD
        # n = mod[1] + '.py'
        # trainPath = join(self.trainingFunctionsPath, n) 
        # NEW
        trainPath = 'trainSAMModel.py'
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
        #OLD
        n = mod[1] + '.py'
        trainPath = join(self.trainingFunctionsPath, n) 
        #NEW
        #trainPath = 'trainSAMModel.py'
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
        # OLD
        # n = mod[1] + '.py'
        # trainPath = join(self.trainingFunctionsPath, n) 
        # NEW
        trainPath = 'trainSAMModel.py'
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs

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
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
        #args = ' '.join([dPath, mPath, mod[1], 'new', 'False', 'False', 'True'])
        #NEW
        args = ' '.join([dPath, mPath, mod[1], 'new', mod[0], 'False', 'False', 'True'])
        
        if(self.verbose): print 'args: ', args

        cmd = 'samOptimiser.py ' + trainPath + ' ' + args
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
        if(mod[0] in self.updateModelsNames):
             args = ' '.join([dPath, mPath, mod[1], 'update', 'False', 'False', 'True'])
        else:
             args = ' '.join([dPath, mPath, mod[1], 'new', 'False', 'False', 'True'])
        #NEW
        #if(mod[0] in self.updateModelsNames):
        #    args = ' '.join([dPath, mPath, mod[1], 'update', mod[0]])
        #else:
        #    args = ' '.join([dPath, mPath, mod[1], 'new', mod[0]])
        
        if(self.verbose): print 'args: ', args

        #OLD
        cmd = 'samOptimiser.py ' + trainPath + ' ' + args
        #NEW
        #cmd = trainPath + ' -- ' + args
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
        #args = ' '.join([dPath, mPath, mod[1], 'new', 'False', 'False', 'True'])
        #NEW
        args = ' '.join([dPath, mPath, mod[1], 'new', mod[0], 'False', 'False', 'True'])
        
        if(self.verbose): print 'args: ', args

        cmd = 'samOptimiser.py ' + trainPath + ' ' + args
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
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
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
        # check communication with loaded models
        readyList = []
        for i, v in self.trainingListHandles.iteritems():
            if i != 'Cluster':
                ret = v.poll()
                if ret is not None:
                    if ret == 0:
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
        #check communication with loaded models
        readyList = []
        for i,v in self.trainingListHandles.iteritems():
            if(i != 'Cluster'):
                ret = v.poll()
                if(ret != None):
                    if(ret == 0):
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
        # check communication with loaded models
        readyList = []
        for i, v in self.trainingListHandles.iteritems():
            if i != 'Cluster':
                ret = v.poll()
                if ret is not None:
                    if ret == 0:
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                        readyList += [i]
                        print i, 'terminated successfully'
                        b = yarp.Bottle()
                        self.checkAvailabilities(b)
                    else:
                        readyList += [i]
                        print i, 'terminated with ', self.SIGNALS_TO_NAMES_DICT[abs(ret)]
                else:
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
                    # if(self.verbose): print i, "still training "
=======
                    #if(self.verbose): print i, "still training "
>>>>>>> New version of samSupervisor with rpc interface and README
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
                    #if(self.verbose): print i, "still training "
=======
                    # if(self.verbose): print i, "still training "
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
                    # if(self.verbose): print i, "still training "
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
                    pass
        
        for i in readyList:
            del self.trainingListHandles[i]
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422

    def updateModule(self):
        if self.iter == 10:
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
        

    def updateModule(self):
        if(self.iter == 10):
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======

    def updateModule(self):
        if self.iter == 10:
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======

    def updateModule(self):
        if self.iter == 10:
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
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
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
    rf.setVerbose(True)
    rf.setDefaultContext("samSupervisor")
    rf.setDefaultConfigFile("default.ini")
    rf.configure(sys.argv)
=======
=======
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
    rf.setVerbose(True);
    rf.setDefaultContext("samSupervisor");
    rf.setDefaultConfigFile("default.ini");
    rf.configure(sys.argv);
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
>>>>>>> New version of samSupervisor with rpc interface and README
=======
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
    rf.setVerbose(True)
    rf.setDefaultContext("samSupervisor")
    rf.setDefaultConfigFile("default.ini")
    rf.configure(sys.argv)
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
>>>>>>> updated SAM folder
>>>>>>> updated SAM folder
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs

    mod.runModule(rf)
