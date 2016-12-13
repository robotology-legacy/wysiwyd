#!/usr/bin/env python

import readline
import yarp
import time
import sys

print sys.argv

class ReachableArea(yarp.RFModule):
    def get_bounds_from_config(self, config_name, default_min, default_max):
        bounds_b = rf.find(config_name).asList();
        if bounds_b:
            bounds = (bounds_b.get(0).asDouble(), bounds_b.get(1).asDouble())
        else:
            print "Warning: bounds for " + config_name + " set to default value"
            bounds = (default_min, default_max)
        return bounds

    def configure(self, rf):
        moduleName = rf.check("name", yarp.Value("reachableArea")).asString()
        self.setName(moduleName)

        self.rpc_port = yarp.Port()
        self.rpc_port.open("/reachableArea/rpc")
        self.attach(self.rpc_port)

        self.human_area_x_bounds = self.get_bounds_from_config("human_area_x_bounds", -0.7, -0.5)
        self.human_area_y_bounds = self.get_bounds_from_config("human_area_y_bounds", -0.3, -0.5)
        self.shared_area_x_bounds = self.get_bounds_from_config("shared_area_x_bounds", -0.7, -0.5)
        self.shared_area_y_bounds = self.get_bounds_from_config("shared_area_y_bounds", -0.7, -0.5)
        self.robot_area_x_bounds = self.get_bounds_from_config("robot_area_x_bounds", -0.7, -0.5)
        self.robot_area_y_bounds = self.get_bounds_from_config("robot_area_y_bounds", -0.7, -0.5)

        return True

    def respond(self, command, reply):
        reply.clear()
        if command.get(0).asString() == "get_area" and command.size() == 4:
            reply.addString(self.getAreaType( (command.get(1).asDouble(), command.get(2).asDouble(), command.get(3).asDouble()) ))
        else:
            reply.addString("Command is: get_area X Y Z")

        return True

    def interruptModule(self):
        print "Interrupting ports"
        self.rpc_port.interrupt()
        return True

    def close(self):
        print "Closing ports"
        self.rpc_port.close()
        return True

    def getPeriod(self):
        return 1.0

    def updateModule(self):
        time.sleep(0.5)
        return True

    def getAreaType(self, objpos):
        if ((objpos[0]>self.human_area_x_bounds[0]) and (objpos[0]<self.human_area_x_bounds[1]) and
            (objpos[1]>self.human_area_y_bounds[0]) and (objpos[1]<self.human_area_y_bounds[1])):
            return "HumanOnly"
        elif ((objpos[0]>self.shared_area_x_bounds[0]) and (objpos[0]<self.shared_area_x_bounds[1]) and
                   (objpos[1]>self.shared_area_y_bounds[0]) and (objpos[1]<self.shared_area_y_bounds[1])):
            return "Shared"
        elif ((objpos[0]>self.robot_area_x_bounds[0]) and (objpos[0]<self.robot_area_x_bounds[1]) and
                   (objpos[1]>self.robot_area_y_bounds[0]) and (objpos[1]<self.robot_area_y_bounds[1])):
            return "RobotOnly"
        else:
            return "NotReachable"


yarp.Network.init()

rf = yarp.ResourceFinder()
rf.setVerbose(False);
rf.setDefaultContext("iol2opc");
rf.setDefaultConfigFile("config.ini");
rf.configure(sys.argv)

mod = ReachableArea()
mod.runModule(rf)

