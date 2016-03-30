#!/usr/local/bin/python

from pyNNLib import *
import yarp

#Open network
yarp.Network.init()

# Open the RPC port
toHomeo = yarp.Port()
portName="/manager/toHomeostasis/rpc"+":o"#"/NNsound:i"    
toHomeo.open(portName)

targetRPC = "/homeostasis/rpc"

print yarp.Network.connect(portName,targetRPC)



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
	return True

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
	return True

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
	return True

def narrate():
	# Prepare command
	cmd = yarp.Bottle()
	cmd.clear()
	cmd.addString('force')
	cmd.addString('narrate')
	cmd.addString('bottom')
	# Send command
	if not yarp.Network.isConnected(portName,targetRPC):
		print yarp.Network.connect(portName,targetRPC)
		yarp.Time.delay(0.1)
	toHomeo.write(cmd)
	return True

def pointing():
	# Prepare command
	cmd = yarp.Bottle()
	cmd.clear()
	cmd.addString('force')
	cmd.addString('pointing')
	cmd.addString('bottom')
	# Send command
	if not yarp.Network.isConnected(portName,targetRPC):
		print yarp.Network.connect(portName,targetRPC)
		yarp.Time.delay(0.1)
	toHomeo.write(cmd)
	return True

def tagging():
	# Prepare command
	cmd = yarp.Bottle()
	cmd.clear()
	cmd.addString('force')
	cmd.addString('tagging')
	cmd.addString('bottom')
	# Send command
	if not yarp.Network.isConnected(portName,targetRPC):
		print yarp.Network.connect(portName,targetRPC)
		yarp.Time.delay(0.1)
	toHomeo.write(cmd)
	return True

def test():
	# Prepare command
	cmd = yarp.Bottle()
	cmd.clear()
	cmd.addString('force')
	cmd.addString('test')
	cmd.addString('bottom')
	# Send command
	if not yarp.Network.isConnected(portName,targetRPC):
		print yarp.Network.connect(portName,targetRPC)
		yarp.Time.delay(0.1)
	toHomeo.write(cmd)
	return True