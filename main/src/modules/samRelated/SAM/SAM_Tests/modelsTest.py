#!/usr/bin/env python
from os import listdir
from os.path import isfile, join
import subprocess
import yarp
import time
import signal
import random
import glob
from shutil import copyfile
from ConfigParser import SafeConfigParser


def test_modelWithParams(modelName, driverName, datacollectionOnly, randomRecall,
                         probRecall, bufferLen, recency, transmissionDelay):

    # constant variables
    automaticOpen = True
    connectDirect = True
    frameLen = 15

    yrf = yarp.ResourceFinder()
    yrf.setVerbose(True)
    yrf.setDefaultContext("samSupervisor")
    yrf.setDefaultConfigFile("default.ini")
    yrf.configure([])

    rootPath = yrf.find("root_path").asString()
    name = rootPath + '/Models/' + modelName + '__' + driverName + '*.pickle'
    fname = glob.glob(name)
    assert len(fname) > 0, 'model file not found'
    modelFileName = fname[0]

    interactionConfFile = yrf.find("config_path").asString()
    interactionConfFile = yrf.findFile(interactionConfFile)
    # copyfile(interactionConfFile, interactionConfFile+'.bkp')
    parser = SafeConfigParser()
    parser.optionxform = str
    parser.read(interactionConfFile)
    assert modelName in parser.sections(), 'model name not in parser sections'

    items = parser.items(modelName)
    for j in items:
        if j[0] == 'dataIn':
            dataInPort = j[1].split(' ')[0]
        elif j[0] == 'rpcBase':
            rpcInPort = j[1]
        elif j[0] == 'callSign':
           askCommand = [k for k in j[1].split(',') if 'label' in k][0]
        elif j[0] == 'collectionMethod':
            collectionMethod = j[1].split(' ')[0]
            if collectionMethod == 'continuous':
                parser.set(modelName, 'collectionMethod', 'continuous ' + str(bufferLen))
        elif j[0] == 'recency' and collectionMethod == 'continuous':
            parser.set(modelName, 'recency', str(recency))

    parser.write(open(interactionConfFile, 'wb'))


    def checkRecall(rpcPort, askCommand):
        # check correct response
        cmBottle = yarp.Bottle()
        rpBottle = yarp.Bottle()
        cmBottle.addString(askCommand)
        rpcPort.write(cmBottle, rpBottle)
        results.append(rpBottle.toString())
        return rpBottle

    networkFound = yarp.Network.checkNetwork()
    assert networkFound, 'Yarpserver not found'
    processesList = []

    # open data port
    dataPortName = '/modelTest/data:o'
    dataPort = yarp.BufferedPortBottle()
    dataPort.open(dataPortName)

    # open query port
    rpcPortName = '/modelTest/rpc'
    rpcPort = yarp.RpcClient()
    rpcPort.open(rpcPortName)

    # open interactionSAMModel
    if connectDirect:
        if automaticOpen:

            args = ' '.join([rootPath +'/Data/' + modelName, modelFileName.replace('.pickle', ''),
                             interactionConfFile, driverName, 'False'])
            pyFile = 'interactionSAMModel.py'
            interactionCMD = ' '.join([pyFile, args])
            print interactionCMD
            windowedCMD = "bash -c \"" + interactionCMD + "\""
            interactionProcess = subprocess.Popen(['xterm', '-e', windowedCMD], shell=False)

            # wait until model loaded
            time.sleep(7)
        else:
            # check interactionSAMModel present
            pass
        yarp.Network.connect(rpcPortName, rpcInPort+':i')
    else:
        yarp.Network.connect(rpcPortName, '/sam/rpc:i')
        # send load Actions3


    # connect data to /sam/actions/actionData:i
    yarp.Network.connect(dataPortName, dataInPort)
    # yarp.Network.connect(dataPortName, '/reader')

    # check length of data log file
    dataFile = open(join('./noisyActionData/recordedData', 'data.log'), 'r')
    for i, l in enumerate(dataFile):
        pass
    lenDataFile = i + 1
    print lenDataFile
    dataFile.close()

    # send each line with a pause of 0.1s and query model every frameLen data points.
    # check correct response and continued operation os samSupervisor

    processBreak = False
    dataFile = open(join('./noisyActionData/recordedData', 'data.log'), 'r')
    results = []
    for curr in range(lenDataFile):
        line = dataFile.readline()
        lineParts = line.split(' ')
        line = ' '.join(lineParts[2:])
        dataBottle = dataPort.prepare()
        dataBottle.fromString(line)
        dataPort.write()
        time.sleep(transmissionDelay)

        if randomRecall and random.random() < probRecall and not datacollectionOnly:
            rpBottle = checkRecall(rpcPort, askCommand)
            pollResult = interactionProcess.poll()
            print "{0:.2f}".format(curr*100.0/lenDataFile)+'%\t', 'Request Response:', rpBottle.toString(), pollResult

        if curr%frameLen == 0:
            if not datacollectionOnly and not randomRecall:
                rpBottle = checkRecall(rpcPort, askCommand)
                # check reply and status of process
                pollResult = interactionProcess.poll()
                print "{0:.2f}".format(curr*100.0/lenDataFile)+'%\t', 'Request Response:', rpBottle.toString(), pollResult
            else:
                pollResult = interactionProcess.poll()
                print "{0:.2f}".format(curr*100.0/lenDataFile)+'%\t', pollResult

            if pollResult is not None:
                processBreak = True
                break

    dataFile.close()
    try:
        print 'Terminating process'
        interactionProcess.send_signal(signal.SIGINT)
    except:
        pass
    retCode = interactionProcess.wait()
    if processBreak:
        testResult = False
    else:
        testResult = True

    return testResult



