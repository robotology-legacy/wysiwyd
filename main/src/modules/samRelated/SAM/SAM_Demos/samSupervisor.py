import matplotlib.pyplot as plt
from SAM.SAM_Core import SAMCore
from SAM.SAM_Drivers import SAMDriver_interaction
import pylab as pb
import sys 
from sys import executable
import subprocess
from subprocess import Popen, PIPE
import pickle
import os
from os import listdir, walk, system
from os.path import isfile, join, isdir
import numpy
import time
import operator
import numpy as np
import datetime

try:
    import yarp
    yarpRunning = True
    print 'Yarp found'
except ImportError:
    print 'WARNING! Yarp was not found! Switching to offline mode'
    yarpRunning = False

#parse configuration file to identify root location for data and models
from ConfigParser import SafeConfigParser

# Check configuration file
parser = SafeConfigParser()

file_candidates = ["config_supervisor.ini"]
section_candidates = ["config_options"]
model_params = ["model_options"]

configData = False

print 'Finding config file'
print '-------------------'
for loc in os.curdir, os.path.expanduser("~"), os.environ.get("WYSIWYD_DIR")+"/share/wysiwyd/contexts/samSupervisor":
    print loc
    try:
        found = parser.read(os.path.join(loc,file_candidates[0]))
        if not found:
            pass
        else:
            pathFound = found
            print os.path.join(loc,file_candidates[0])
            if( parser.has_section(section_candidates[0]) == True ):
                rootPath = parser.get(section_candidates[0], 'root_path')
                interactionConfPath = parser.get(section_candidates[0], 'config_path')
              	interactionConfPath = join(loc,interactionConfPath)
                configData = True
            else:
                print 'config_options not found...'
    except IOError:
        pass

if( configData == True ):
    print "config paths ready"
else:
    print "config paths failed"
    sys.exit()

print '-------------------'
print 'Config file found: ' + pathFound[0]
print rootPath
print interactionConfPath
print '-------------------'

modelPath = rootPath + '/Models'
dataPath = rootPath + '/Data'
trainingFunctionsPath = os.environ.get("WYSIWYD_DIR")+"/bin"
#after finding the root path, go to models folder and compile list of all
#models together with the last time they were modified

onlyfiles = [f for f in listdir(modelPath) if isfile(join(modelPath, f))]

#find number of .pickle files
modelsList = [s.replace(".pickle","") for s in onlyfiles if ".pickle" in s if '~' not in s]
print 'Models available:                ' + ', '.join(modelsList)

#likewise go to data folder and compile list of all folders and last time they were modified
dataList = [f for f in listdir(dataPath) if isdir(join(dataPath, f))]
print "Data folders available:          " + ', '.join(dataList)

#likewise parse training functions folder
functionsList = [f.replace(".py","") for f in listdir(trainingFunctionsPath) if isfile(join(trainingFunctionsPath, f)) if ".py" in f if '~' not in f]
print "Training functions available:    " + ', '.join(functionsList)

#format of training functions is expected to be train_modelName_anythingElseToDistinguish
#therefore data folders must contain .ini file pointing towards the preferred algorithm to be chosen
print '-------------------'
print 'Finding trainable data ...'
print
#exit if no training functions have been found
if(len(functionsList) == 0):
    print "No training functions found. Exiting ..."
    sys.exit()
else:
    trainableModels = []
    #check which data folders are trainable i.e training functions available
    for f in dataList:
        loc = join(dataPath, f)
        print "Checking " + loc + " ..."
        try:
            parser = SafeConfigParser()
            found = parser.read(loc + "/config.ini")
            if not found:
                print "config.ini not found for " + f
                pass
            else:
                if( parser.has_section(model_params[0]) == True ):
                    trainOptions = parser.get(model_params[0], 'train').split(',')
                    #check training function exists 
                    availableFuncs = [s for s in trainOptions for g in functionsList if s == g]
                    if(len(availableFuncs) != 0):
                        print "Training functions for data " + f + " are " + ','.join(trainOptions)
                        print "Corresponding functions available: " + ','.join(availableFuncs)
                        if(len(availableFuncs) > 1):
                            print "The first function will be chosen: " + availableFuncs[0]
                        #find latest modified date of directory and subdirectories thus checking for addition of new data
                        t = []
                        for dirName, dirs, filenames in os.walk(loc):
                            t.append(os.path.getmtime(dirName))
                        lastMod = max(t)
                        print "Data folder last modified: %s" % time.ctime(lastMod)
                        #format of trainableModels is: dataFolder name, correspoding training function, date data ast modified, train boolean
                        trainableModels += [[f, availableFuncs[0], lastMod, True]]
                    else:
                        print "Training functions for data " + f + " not found. Will not train " + f 
                else:
                    print "Training parameters for data " + f + " not found. Will not train " + f + "\nCheck config.ini is formatted correctly"
        except IOError:
            pass
        print 
    print '-------------------'
    print 'Checking corresponding models'
    print
    #compare models and data folders. Assuming model names = folder names
    #check if model exists
    for f in trainableModels:
        t = []
        currModels = []
        for g in modelsList:
            if(f[0] in g and '~' not in g):
                #compare time of model and data
                currModels.append(g)
                g += ".pickle"
                loc = join(modelPath,g)
                t.append(os.path.getmtime(loc))
        if(len(t) > 0):
            lastMod = max(t)
            currModels = currModels[t.index(lastMod)]
            trainableModels[trainableModels.index(f)].append(currModels)
            print f[0] + " Model last modified: %s" % time.ctime(lastMod)
            if(lastMod < f[2]):
                tdiff =  datetime.datetime.fromtimestamp(f[2]).replace(microsecond = 0) - datetime.datetime.fromtimestamp(lastMod).replace(microsecond = 0)
                print f[0] +' Model outdated by ' + str(tdiff) + '. Will be trained'
            else:
                print f[0] +' Model up-to-date'
                f[3] = False
        else:
            trainableModels[trainableModels.index(f)].append('')
            print f[0] + ' Model not found. Training Required'
        print
    print '-------------------'
    print

    #provide option to train now or on close 
    #if train now provide option to change experiment number or leave default
    updateModels = [s for s in trainableModels if s[3] == True if s[4] != '']
    noModels = [s for s in trainableModels if s[3] == True if s[4] == '']
    uptodateModels = [s for s in trainableModels if s[3] == False] 
    
    print str(len(uptodateModels)) + " Models up-to-date."
    print str(len(updateModels)) + " Models require an update."
    print str(len(noModels)) + " new models to train."
    print
    trainNowChoice = ''
    #trainNowChoice = 'y' #DEBUG
    if(len(updateModels) + len(noModels) > 0):
        while(trainNowChoice == '' or not (trainNowChoice == 'y' or trainNowChoice == 'n')):
            trainNowChoice = raw_input("Do you want to train models now? Press Y or N: ")
            trainNowChoice = trainNowChoice.lower()
    print '-------------------'
    if(trainNowChoice =='y'):
        print "Training Models:"
        print
        #start calling training functions for all items in updateModels and uptodateModels
        for mod in updateModels + noModels:
            n = mod[1] + '.py'
            trainPath = join(trainingFunctionsPath, n) 
            print 'Training ' + mod[0] + ' ...'
            print 'Opening ' + trainPath
            print
            dPath = join(dataPath, mod[0])
            if(mod[4] != ''):
                mPath = join(modelPath, mod[4]) + '.pickle'
            else:
                mPath = join(modelPath, mod[4])

            #open separate ipython for training
            #this will allow separate training across different computers in future
            if(mod in updateModels):
                command = 'ipython ' + trainPath + ' -- ' + ' '.join([dPath, mPath, mod[1], 'update'])
            else:
                command = 'ipython ' + trainPath + ' -- ' + ' '.join([dPath, mPath, mod[1], 'new'])
            os.system(command)
            mod[3] = False
        print
        print 'Training finished'
        print '-------------------'
        print
        for j in trainableModels:
            print j
        print
        print 'Updating models status:'
        
        #update trainableModels
        for f in trainableModels:
            t = []
            currModels = []
            for g in modelsList:
                if(f[0] in g and '~' not in g):
                    #compare time of model and data
                    currModels.append(g)
                    g += ".pickle"
                    loc = join(modelPath,g)
                    t.append(os.path.getmtime(loc))
            currModels
            if(len(t) > 0):
                lastMod = max(t)
                currModels = currModels[t.index(lastMod)]
                trainableModels[trainableModels.index(f)][4] = currModels
                print f[0] + " Model last modified: %s" % time.ctime(lastMod)
                if(lastMod < f[2]):
                    tdiff =  datetime.datetime.fromtimestamp(f[2]).replace(microsecond = 0) - datetime.datetime.fromtimestamp(lastMod).replace(microsecond = 0)
                    print f[0] +' Model outdated by ' + str(tdiff) + '. Will be trained'
                else:
                    print f[0] +' Model up-to-date'
                    f[3] = False
            else:
                trainableModels[trainableModels.index(f)].append('')
                print f[0] + ' Model not found. Training Required'
                f[3] = True
            print
        print '-------------------'
        print

    uptodateModels = [s for s in trainableModels if s[3] == False]


    rpcConnections = []
    #check yarpserver
    yarp.Network.init()
    if(yarpRunning):
        for j in uptodateModels:
            #find corresponding interaction function
            parser = SafeConfigParser()
            parser.read(join(dataPath, j[0]) + "/config.ini")
            if(parser.has_option('model_options', 'interaction')):
                interactionFunction = parser.get('model_options', 'interaction').split(',')
                interactionFunction = [s for s in interactionFunction for g in functionsList if s == g]
                if(len(interactionFunction) != 0):
                    parser2 = SafeConfigParser()
                    parser2.read(interactionConfPath)
                    if(parser2.has_section(j[0].lower())): #checking current model allows this memory section
                        #create interface ports
                        interfacePortName = parser2.get(j[0].lower(),'rpcBase') + ':o'
                        callSignList = parser2.get(j[0].lower(),'callSign').replace(' ', '').split(',')
                        interfacePort = yarp.RpcClient()
                        interfacePort.open(interfacePortName)
                        rpcConnections.append([j[0].lower(), interfacePort, interfacePortName[:-1], callSignList])
                        args = ' '.join([join(dataPath,j[0]), join(modelPath, j[4]), interactionConfPath])
                        command = 'ipython ' + join(trainingFunctionsPath, interactionFunction[0]+'.py') + ' -- ' + args
                        command = "bash -c \"" + command + "; exec bash\""
                        #command = "bash -c \"" + command + "\""
                        subprocess.Popen(['gnome-terminal','-e', command])
                else:
                    print 'No interaction function found in model path. Skipping model'
            else:
                print 'No interaction function specified in config.ini. Skipping model'
        
        #connect supervisor to all submodels
        if(len(rpcConnections) != 0):
            for j in rpcConnections:
                noConn = True
                print 'connecting ' + j[2]+'o ' + 'with ' + j[2]+'i'
                while(noConn):
                    noConn = yarp.Network.connect(j[2]+'o',j[2]+'i')
                    noConn = not noConn
                    time.sleep(1)
                print 'connected'

            #create rpc port to communicate with outside
            supervisorPort = yarp.RpcServer()
            supervisorPort.open('/sam/rpc:i')
            inputBottle = yarp.Bottle()
            sendingBottle = yarp.Bottle()
            responseBottle = yarp.Bottle()
            outputBottle = yarp.Bottle()
            print
            print '-------------------'

            while( True ):
                try: 
                    print 'Waiting for input:'
                    supervisorPort.read(inputBottle,True)

                    #determine towards which process inputBottle is directed
                    callSign = inputBottle.get(0).asString()
                    print callSign + ' request received'
                    print
                    replied = False;
                    for j in rpcConnections:
                        if(callSign in j[3]):
                            #sendingBottle.clean()
                            sendingBottle = inputBottle
                            #send input bottle intact
                            j[1].write(sendingBottle,responseBottle)
                            supervisorPort.reply(responseBottle)
                            replied = True

                    if(not replied):
                        supervisorPort.reply(yarp.Bottle('nack'))

                except KeyboardInterrupt:
                    print 'Interrupted'
                    for j in rpcConnections:
                        j[1].write(yarp.Bottle('EXIT'), inputBottle)
                        j[1].close()

                    supervisorPort.close()
                    try:
                        sys.exit(0)
                    except SystemExit:
                        os._exit(0)
        else:
            print 'Intersection of sensory_level_conf and available models = no models'
            print 'Exiting...'
            try:
                sys.exit(0)
            except SystemExit:
                os._exit(0)
    else:
        print 'Error: Yarp not found'

    print 'Finished'
    for j in rpcConnections:
        j[1].close()

    supervisorPort.close()
