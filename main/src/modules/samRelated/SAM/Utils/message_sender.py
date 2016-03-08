import sys
import time
import os
try:
    import yarp
    yarpRunning = True
except ImportError:
    print 'WARNING! Yarp was not found! Switching to offline mode'
    yarpRunning = False

messageList = []
messageList.append('ask_face_label')
messageList.append('ask_face_instance')
messageList.append('information_PAOR (P1 wanted) (A1 I) (P2 get) (A2 I) (O2 croco)')
messageList.append('567 1454594804.655575 none none ((elbowLeft (-1.256752 0.27183 -0.246973)) (elbowRight (-1.270087 -0.326172 -0.238417)) (handLeft (-1.046666 0.28861 -0.012651)) (handRight (-1.073482 -0.459701 -0.038741)) (head (-1.294263 -0.085272 0.132913)) (shoulderCenter (-1.292125 -0.055619 -0.083768)) (shoulderLeft (-1.290561 0.097068 -0.091069)) (shoulderRight (-1.29369 -0.208306 -0.076466)) (spine (-1.284894 -0.060415 -0.182516))) (partner -1.294263 -0.085272 0.132913 1) (car -0.57236 -0.023297 -0.207144 1) (octopus -1.335235 0.291384 -0.175733 1) 0')
messageList.append('information_PAOR (P1 failed) (A1 I) (P2 grasp) (A2 I) (O2 it)')
messageList.append('information_PAOR (P1 laid) (A1 it) (O1 outofreach)')
messageList.append('information_PAOR (P1 found) (A1 I) (O1 action) (R1 different)')
messageList.append('information_PAOR (P1 could) (A1 I) (P2 ask) (A2 I) (O2 you) (P3 give) (A3 you) (O3 it) (R3 me)')
messageList.append('information_PAOR (P1 would) (A1 you) (P2 give) (A2 you) (O2 it) (R2 me)')
messageList.append('information_PAOR (P1 asked) (A1 I) (O1 you) (P2 give) (A2 you) (O2 it) (R2 me)')
messageList.append('information_PAOR (P1 gave) (A1 you) (O1 it) (R1 me)')
messageList.append('information_PAOR (P1 have) (A1 I) (O1 croco)')

clientName = '/testSender'
#serverName = '/sam/rpc:i'
serverName = '/testServer'
yarp.Network.init()

print 'available messages: \n' + '\n'.join(messageList)

messagePort = yarp.RpcClient()
messagePort.open(clientName)

connected = False
inputBottle = yarp.Bottle()
outputBottle = yarp.Bottle()

try:
	while(True):
		if(messagePort.getOutputCount() == 0):
			print 'Waiting for a connection'
			print str(messagePort.getOutputCount())
			yarp.Network.connect(clientName, serverName)
			time.sleep(1)
		else:
			var = raw_input('choose message to send: ')
			var = int(var)
			if(var < 0 or var > len(messageList)):
				var = 0
			outputBottle = yarp.Bottle(messageList[var])
			print 'sending ' + outputBottle.toString()
			messagePort.write(outputBottle, inputBottle)

			inputList = []
			for i in range(inputBottle.size()):
				inputList.append(inputBottle.get(i).asString())

			print 'received input:'
			print inputList

except KeyboardInterrupt:
	print
	print 'Interrupted'

	messagePort.close()
	try:
		sys.exit(0)
	except SystemExit:
		os._exit(0)

