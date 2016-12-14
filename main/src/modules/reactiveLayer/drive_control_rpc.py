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

toPlanner = yarp.Port()
plannerPortName="/manager/toPlanner/rpc:o"
toPlanner.open(plannerPortName)

homeoRPC = "/homeostasis/rpc"
alloRPC = "/AllostaticController/rpc"
BMRPC = "/BehaviorManager/rpc"
plannerRPC = "/planner/rpc"

print yarp.Network.connect(homeoPortName,homeoRPC)
print yarp.Network.connect(alloPortName,alloRPC)
print yarp.Network.connect(BMPortName,BMRPC)
print yarp.Network.connect(plannerPortName,plannerRPC)

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
    return behList

def close_ports():
    print "Interrupting and closing ports"
    # interrupting
    toHomeo.interrupt()
    toAllo.interrupt()
    toBM.interrupt()
    toPlanner.interrupt()
    # closing
    toHomeo.close()
    toAllo.close()
    toBM.close()
    toPlanner.close()


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


def launch_behavior(behavior):
    # Prepare command
    cmd = yarp.Bottle()
    cmd.clear()
    cmd.addString(behavior)
    # Send command
    if not yarp.Network.isConnected(BMPortName,BMRPC):
        print yarp.Network.connect(BMPortName,BMRPC)
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
    cmd_allo = yarp.Bottle()
    cmd_allo.clear()
    cmd_allo.addString('manual')
    cmd_allo.addString('on')
    # Send command to allostatic controller
    if not yarp.Network.isConnected(alloPortName,alloRPC):
        print yarp.Network.connect(alloPortName,alloRPC)
        yarp.Time.delay(0.1)
    toAllo.write(cmd_allo)

    # Send command to planner
    cmd_planner = yarp.Bottle()
    cmd_planner.clear()
    cmd_planner.addString('manual')
    cmd_planner.addString('on')
    if not yarp.Network.isConnected(plannerPortName,plannerRPC):
        print yarp.Network.connect(plannerPortName,plannerRPC)
        yarp.Time.delay(0.1)
    toPlanner.write(cmd_planner)

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

