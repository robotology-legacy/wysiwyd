#!/usr/bin/env python

import yarp

#Open network
yarp.Network.init()

# Open the RPC port
toHomeo = yarp.Port()
homeoPortName="/driveControl/toHomeostasis/rpc"+":o"
toHomeo.open(homeoPortName)

toAllo = yarp.Port()
alloPortName="/driveControl/toAllostasis/rpc"+":o"
toAllo.open(alloPortName)

homeoRPC = "/homeostasis/rpc"
alloRPC = "/AllostaticController/rpc"
behaviorRPC = "/BehaviorManager/trigger:i"

toBehaviorManager = yarp.Port()
toBehaviorManager.open("/driveControl/to_behaviorManager_rpc")

print yarp.Network.connect(homeoPortName,homeoRPC)
print yarp.Network.connect(alloPortName,alloRPC)
print yarp.Network.connect(toBehaviorManager.getName(), behaviorRPC)

request = yarp.Bottle()
rep = yarp.Bottle()

# Retrieve behavior names from the behaviorManager module
request.clear()
rep.clear()
request.addString("names")
toBehaviorManager.write(request, rep)
behaviors = []
behavior_ports = {}
names = rep.get(0).asList()
for i in range(names.size()):
	name = names.get(i).asString()
	behaviors.append(name)
	behavior_ports[name] = yarp.BufferedPortBottle()
	behavior_ports[name].open("/driveControl/behaviors/" + behaviors[-1] + ":i")
	yarp.Network.connect("/BehaviorManager/" + behaviors[-1] + "/start_stop:o", behavior_ports[name].getName())


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
	if not yarp.Network.isConnected("/BehaviorManager/" + behavior + "/start_stop:o", behavior_ports[behavior].getName()):
		print yarp.Network.connect("/BehaviorManager/" + behavior + "/start_stop:o", behavior_ports[behavior].getName())
		yarp.Time.delay(0.1)	
	toHomeo.write(cmd)
	yarp.Time.delay(0.5)
	reset_all()
	freeze_all()	
	res = behavior_ports[behavior].read(False)
	while res is None and msg != "stop":
		print "while"
		res = behavior_ports[behavior].read(False)
		if res is not None:
			msg = res.get(0).asString()
			if msg == "stop":
				reset_all()
				freeze_all()			
		yarp.Time.delay(0.1)
		

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

def dummy():
	trigger_behavior("dummy")

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
	
