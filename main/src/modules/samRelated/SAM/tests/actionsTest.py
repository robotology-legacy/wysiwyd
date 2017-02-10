#!/usr/bin/env python
from os import listdir
from os.path import isfile, join
import subprocess
import yarp
import time
import signal

automaticOpen = True
connectDirect = True

networkFound = yarp.Network.checkNetwork()
assert networkFound, 'Yarpserver not found'
processesList = []

# open data port
dataPortName = '/actionsTest/data:o'
dataPort = yarp.BufferedPortBottle()
dataPort.open(dataPortName)

# open query port
rpcPortName = '/actionsTest/rpc'
rpcPort = yarp.RpcClient()
rpcPort.open(rpcPortName)

# open interactionSAMModel
if connectDirect:
    if automaticOpen:
        args = ' '.join(['/home/$USER/SAM_Data_Models/Data/Actions3 ',
                         '/home/$USER/SAM_Data_Models/Models/Actions3__SAMDriver_ARWin__mrd__backup',
                         '/home/$USER/.local/share/yarp/contexts/samSupervisor/sensory_level_conf.ini',
                         'SAMDriver_ARWin', 'False'])
        pyFile = 'interactionSAMModel.py'
        interactionCMD = ' '.join([pyFile, args])
        print interactionCMD
        windowedCMD = "bash -c \"" + interactionCMD + "\""
        interactionProcess = subprocess.Popen(['xterm', '-hold', '-e', windowedCMD], shell=False)

        # wait until model loaded
        time.sleep(7)
    else:
        # check interactionSAMModel present
        pass
    yarp.Network.connect(rpcPortName, '/sam/actions/rpc:i')
else:
    yarp.Network.connect(rpcPortName, '/sam/rpc:i')
    # send load Actions3


# connect data to /sam/actions/actionData:i
yarp.Network.connect(dataPortName, '/sam/actions/actionData:i')
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
frameLen = 15
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
    time.sleep(0.05)
    if curr%frameLen == 0:
        # check correct response
        cmBottle = yarp.Bottle()
        rpBottle = yarp.Bottle()
        cmBottle.addString('ask_action_label')
        rpcPort.write(cmBottle, rpBottle)
        results.append(rpBottle.toString())
        # check reply and status of process
        print "{0:.2f}".format(curr*100.0/lenDataFile)+'%\t', rpBottle.toString(), interactionProcess.poll()
        if interactionProcess.poll() is not None:
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
    print 'ERROR'
else:
	print 'SUCCESS'
raw_input("Press Enter to exit...")



