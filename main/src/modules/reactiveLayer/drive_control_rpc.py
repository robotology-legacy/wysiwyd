#!/usr/local/bin/python

import yarp

#Open network
yarp.Network.init()

# Open the RPC port
toHomeo = yarp.Port()
portName="/manager/toHomeostasis/rpc"+":o"#"/NNsound:i"    
toHomeo.open(portName)

targetRPC = "/homeostasis/rpc"

print yarp.Network.connect(portName,targetRPC)

def trigger_behavior(behavior):
	# Prepare command
	cmd = yarp.Bottle()
	cmd.clear()
	cmd.addString('force')
	cmd.addString(behavior)
	cmd.addString('bottom')
	# Send command
	if not yarp.Network.isConnected(portName,targetRPC):
		print yarp.Network.connect(portName,targetRPC)
		yarp.Time.delay(0.1)
	toHomeo.write(cmd)
	yarp.Time.delay(2.)
	freeze_all()	

def freeze_all():
	# Prepare command
	cmd = yarp.Bottle()
	cmd.clear()
	cmd.addString('freeze')
	cmd.addString('all')
	# Send command
	if not yarp.Network.isConnected(portName,targetRPC):
		print yarp.Network.connect(portName,targetRPC)
		yarp.Time.delay(0.1)
	toHomeo.write(cmd)

def unfreeze_all():
	# Prepare command
	cmd = yarp.Bottle()
	cmd.clear()
	cmd.addString('unfreeze')
	cmd.addString('all')
	# Send command
	if not yarp.Network.isConnected(portName,targetRPC):
		print yarp.Network.connect(portName,targetRPC)
		yarp.Time.delay(0.1)
	toHomeo.write(cmd)

def reset_all():
	# Prepare command
	cmd = yarp.Bottle()
	cmd.clear()
	cmd.addString('par')
	cmd.addString('all')
	cmd.addString('reset')
	# Send command
	if not yarp.Network.isConnected(portName,targetRPC):
		print yarp.Network.connect(portName,targetRPC)
		yarp.Time.delay(0.1)
	toHomeo.write(cmd)

def narrate():
	trigger_behavior("narrate")

def pointing():
	trigger_behavior("pointing")

def tagging():
	trigger_behavior("tagging")

def test():
	trigger_behavior("test")