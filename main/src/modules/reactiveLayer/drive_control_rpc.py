#!/usr/bin/env python

import yarp

#Open network
yarp.Network.init()

# Open the RPC port
toHomeo = yarp.Port()
homeoPortName="/manager/toHomeostasis/rpc"+":o"#"/NNsound:i"    
toHomeo.open(homeoPortName)

toAllo = yarp.Port()
alloPortName="/manager/toAllostasis/rpc"+":o"#"/NNsound:i"    
toAllo.open(alloPortName)

toBM = yarp.Port()
BMPortName="/manager/BehaviorManager/rpc"+":o"#"/NNsound:i"    
toBM.open(BMPortName)

homeoRPC = "/homeostasis/rpc"
alloRPC = "/AllostaticController/rpc"
BMRPC = "/BehaviorManager/rpc"

print yarp.Network.connect(homeoPortName,homeoRPC)
print yarp.Network.connect(alloPortName,alloRPC)
print yarp.Network.connect(BMPortName,BMRPC)

def updateDriveList():
	cmd = yarp.Bottle()
	rply = yarp.Bottle()
	cmd.clear()
	cmd.addString('names')
	toHomeo.write(cmd,rply)
	driveList = rply.toString().strip('()').split(' ')
	print driveList
	return driveList

def updateBehaviorList():
	cmd = yarp.Bottle()
	rply = yarp.Bottle()
	cmd.clear()
	cmd.addString('names')
	toBM.write(cmd,rply)
	behList = rply.toString().strip('()').split(' ')
	print behList
	return beheList

def close_ports():		
	print "Interrupting and closing ports"		
	# interrupting		
	toHomeo.interrupt()		
	toAllo.interrupt()	
	toBM.interrupt()			
	# closing		
	toHomeo.close()		
	toAllo.close()
	toBM.close()		


def trigger_behavior(behavior):
	# Prepare command
	cmd = yarp.Bottle()
	cmd.clear()
	cmd.addString('force')
	cmd.addString(behavior)
	cmd.addString('bottom')
	# Send command
	if not yarp.Network.isConnected(homeoPortName,homeoRPC):
		print yarp.Network.connect(homeoPortName,homeoRPC)
		yarp.Time.delay(0.1)
	toHomeo.write(cmd)
	yarp.Time.delay(2.)
	reset_all()
	freeze_all()	

def freeze_all():
	# Prepare command
	cmd = yarp.Bottle()
	cmd.clear()
	cmd.addString('freeze')
	cmd.addString('all')
	# Send command
	if not yarp.Network.isConnected(homeoPortName,homeoRPC):
		print yarp.Network.connect(homeoPortName,homeoRPC)
		yarp.Time.delay(0.1)
	toHomeo.write(cmd)

def unfreeze_all():
	# Prepare command
	cmd = yarp.Bottle()
	cmd.clear()
	cmd.addString('unfreeze')
	cmd.addString('all')
	# Send command
	if not yarp.Network.isConnected(homeoPortName,homeoRPC):
		print yarp.Network.connect(homeoPortName,homeoRPC)
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
	if not yarp.Network.isConnected(homeoPortName,homeoRPC):
		print yarp.Network.connect(homeoPortName,homeoRPC)
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

def manual_mode():
	# Prepare command
	cmd = yarp.Bottle()
	cmd.clear()
	cmd.addString('manual')
	cmd.addString('on')
	# Send command
	if not yarp.Network.isConnected(alloPortName,alloRPC):
		print yarp.Network.connect(alloPortName,alloRPC)
		yarp.Time.delay(0.1)
	toAllo.write(cmd)

	yarp.Time.delay(0.1)
	freeze_all()
	yarp.Time.delay(0.1)
	reset_all()

def automatic_mode():
	# Prepare command
	cmd = yarp.Bottle()
	cmd.clear()
	cmd.addString('manual')
	cmd.addString('off')
	# Send command
	if not yarp.Network.isConnected(alloPortName,alloRPC):
		print yarp.Network.connect(alloPortName,alloRPC)
		yarp.Time.delay(0.1)
	toAllo.write(cmd)

	yarp.Time.delay(0.1)
	reset_all()
	yarp.Time.delay(0.1)
	unfreeze_all()
	
