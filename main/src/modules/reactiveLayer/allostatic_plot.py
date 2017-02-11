#!/usr/bin/env python

from copy import copy
from time import sleep
import matplotlib.pyplot as plt
from matplotlib import get_backend
from pylab import get_current_fig_manager
from numpy import linspace
from collections import defaultdict
import yarp
import sys

def change_drive_names(drive_names):
    new_names = []
    for name in drive_names:
        if name == "exploration":
            new_names.append("Knowledge acquisition")
        elif name == "demonstration":
            new_names.append("Knowledge expression")
        elif name == "narration":
            new_names.append("History narration")
        elif name == "dummy":
            new_names.append("This is a test")
        else:
            new_names.append(name)
    return new_names

default_geometry = dict(xpos=0.0, ypos=0.0, width=800, height=400)

class AllostaticPlotModule(yarp.RFModule):
    def configure(self, rf):

        for attr in ["xpos", "ypos", "width", "height"]:
            a = rf.find(attr)
            val = a.asInt() if not a.isNull() else default_geometry[attr]
            setattr(self, attr, val)
        # self.xpos = rf.find("xpos").asInt();
        # self.ypos = rf.find("ypos").asInt();
        # self.width = rf.find("width").asInt();
        # self.height = rf.find("height").asInt();
        # time window size (= win_size * 0.1s)
        self.win_size = 600

        self.module_name = "allostatic_plot"
        self.setName(self.module_name)
        self.homeo_rpc = yarp.Port()
        self.homeo_rpc.open("/" + self.module_name +"/to_homeo_rpc")
        yarp.Network.connect(self.homeo_rpc.getName(), "/homeostasis/rpc")
        self.behaviorManager_rpc = yarp.Port()
        self.behaviorManager_rpc.open("/" + self.module_name +"/to_behaviorManager_rpc")
        yarp.Network.connect(self.behaviorManager_rpc.getName(), "/BehaviorManager/trigger:i")

        request = yarp.Bottle()
        rep = yarp.Bottle()

        # Retrieve drive names from the homeostasis module
        request.addString("names")
        self.homeo_rpc.write(request, rep)
        self.drives = []
        names = rep.get(0).asList()
        if not names:
            return False
        for i in range(names.size()):
            self.drives.append(names.get(i).asString())

        # Retrieve behavior names from the behaviorManager module
        request.clear()
        rep.clear()
        request.addString("names")
        self.behaviorManager_rpc.write(request, rep)
        self.behaviors = []
        self.behavior_ports = []
        names = rep.get(0).asList()
        if not names:
            return False
        for i in range(names.size()):
            self.behaviors.append(names.get(i).asString())
            self.behavior_ports.append(yarp.BufferedPortBottle())
            self.behavior_ports[-1].open("/" + self.module_name + "/behaviors/" + self.behaviors[-1] + ":i")
            yarp.Network.connect("/BehaviorManager/" + self.behaviors[-1] + "/start_stop:o", self.behavior_ports[-1].getName())

        # Retrieve homeostasis boundaries
        self.homeo_mins = []
        self.homeo_maxs = []
        for d in self.drives:
            request.clear()
            request.addString("ask") 
            request.addString(d) 
            request.addString("min") 
            rep.clear()
            self.homeo_rpc.write(request, rep)
            self.homeo_mins.append(rep.get(0).asDouble())

            request.clear()
            request.addString("ask") 
            request.addString(d) 
            request.addString("max") 
            rep.clear()
            self.homeo_rpc.write(request, rep)
            self.homeo_maxs.append(rep.get(0).asDouble())    


        # Connect to the homeostasis module to get drive values
        self.drive_value_ports = [yarp.BufferedPortBottle() for _ in self.drives] 
        for i, d in enumerate(self.drives):
            self.drive_value_ports[i].open("/" + self.module_name +"/" + d + ":i") 
            yarp.Network.connect("/homeostasis/" + d + "/max:o", self.drive_value_ports[i].getName())



        # Init matplotlib stuff
        min_val = min(self.homeo_mins)
        max_val = max(self.homeo_maxs)
        center_val = (max_val - min_val) / 2.
        self.y_min = -0.1  # ((min_val - center_val) * 1.25) + center_val
        self.y_max = 1.1  # ((max_val - center_val) * 1.25) + center_val
        self.fig = plt.figure()
        thismanager = get_current_fig_manager()
        thismanager.window.setGeometry(self.xpos, self.ypos, self.width, self.height)
        self.ax = plt.axes(xlim=(0, self.win_size), ylim=(self.y_min, self.y_max))
        self.colors = plt.cm.viridis(linspace(0,1,len(self.drives)+2))
        self.value_lines = [self.ax.plot([], [], linestyle = '-', color=self.colors[i+1], lw=2, label=d) for i,d in enumerate(self.drives)]
        self.homeo_min_lines = [self.ax.plot([], [], linestyle = '--', color=self.colors[i+1], lw=1) for i,d in enumerate(self.drives)]
        self.homeo_max_lines = [self.ax.plot([], [], linestyle = '--', color=self.colors[i+1], lw=1) for i,d in enumerate(self.drives)]
        plt.legend(change_drive_names(self.drives))
        self.ax.set_xlim(0- self.win_size, 0)


        self.drive_values = [[0.] * self.win_size for _ in self.drives]

        self.behaviors_to_plot = defaultdict(list)

        self.t = 0

        return True

    def reconnect_ports(self):
        everything_connected = True
        for name_in, port_out in zip(self.behaviors, self.behavior_ports):
            in_port, out_port = "/BehaviorManager/" + name_in + "/start_stop:o", port_out.getName()
            if not yarp.Network.isConnected(in_port, out_port):
                if not yarp.Network.connect(in_port, out_port):
                    print "Could not connect to /BehaviorManager/" + name_in + "/start_stop:o"
                    everything_connected = False

        if not yarp.Network.isConnected(self.behaviorManager_rpc.getName(), "/BehaviorManager/trigger:i"):
            if not yarp.Network.connect(self.behaviorManager_rpc.getName(), "/BehaviorManager/trigger:i"):
                print "Could not connect to /BehaviorManager/trigger:i"
                everything_connected = False

        for i, d in enumerate(self.drives):
            in_port, out_port = "/homeostasis/" + d + "/max:o", self.drive_value_ports[i].getName()
            if not yarp.Network.isConnected(in_port, out_port):
                if not yarp.Network.connect(in_port, out_port):
                    print "Coult not connect to /homeostasis/" + d + "/max:o", self.drive_value_ports[i].getName()
                    everything_connected = False

        if not yarp.Network.isConnected(self.homeo_rpc.getName(), "/homeostasis/rpc"):
            if not yarp.Network.connect(self.homeo_rpc.getName(), "/homeostasis/rpc"):
                print "Could not connect to /homeostasis/rpc"
                everything_connected = False

        return everything_connected


    def close(self):
        print "Closing ports..."
        self.homeo_rpc.close()
        self.behaviorManager_rpc.close()
        for p in self.behavior_ports:
            p.close()
        for p in self.drive_value_ports:
            p.close()

    def interruptModule(self):
        print "Interrupting ports..."
        self.homeo_rpc.interrupt()
        self.behaviorManager_rpc.interrupt()
        for p in self.behavior_ports:
            p.interrupt()
        for p in self.drive_value_ports:
            p.interrupt()
        return True        

    def getPeriod(self):
        return 0.1

    def one_step(self,t):
        if t % (10 / self.getPeriod()) == 0:
            if not self.reconnect_ports():
                yarp.Time.delay(0.1)
                return

        self.drive_values = [values[1:] + [0.] for values in self.drive_values]
        for i, (port, homeo_max, v_line, min_line, max_line) in enumerate(zip(self.drive_value_ports, self.homeo_maxs, self.value_lines, self.homeo_min_lines, self.homeo_max_lines)):
            res = port.read(False)
            if res is not None:
                self.drive_values[i][-1] = res.get(0).asDouble() + homeo_max
                v_line[0].set_data(range(0- self.win_size, 0), self.drive_values[i])
                min_line[0].set_data((0- self.win_size, 0), (self.homeo_mins[i], self.homeo_mins[i]))
                max_line[0].set_data((0- self.win_size, 0), (self.homeo_maxs[i], self.homeo_maxs[i]))

        for key, plotitem_list in self.behaviors_to_plot.iteritems():
            for plotitem in plotitem_list:
                plotitem[0].set_x(plotitem[0].get_x() - 1)
                plotitem[1].set_x(plotitem[0].get_x() + 5)
                if -(plotitem[0].get_x() + plotitem[0].get_width()) > self.win_size:
                    # the plotted rectangle is outside the x-axis from the left
                    plotitem[1].remove()
                    plotitem_list.pop(0)

        # if len(self.behaviors_to_plot)>0 and -(self.behaviors_to_plot[0][1].get_x()+self.behaviors_to_plot[0][1].get_width()) > self.win_size:
        #     self.behaviors_to_plot[0][2].remove()
        #     self.behaviors_to_plot.pop(0)

        for name, port in zip(self.behaviors, self.behavior_ports):
            res = port.read(False)
            if res is not None:
                msg = res.get(0).asString()
                print name, msg
                if msg == "start":
                    new_rectangle = plt.Rectangle(xy=(0, self.y_min), width=10000, height=(self.y_max-self.y_min)/20.)
                    plt.gca().add_patch(new_rectangle)
                    new_text = plt.text(5, self.y_min+0.025, name, horizontalalignment='left', color="gray")
                    self.behaviors_to_plot[name].append((new_rectangle, new_text))
                    print "Behavior " + name + " starts"
                elif msg == "stop":
                    # print "TEST ", self.behaviors_to_plot
                    self.behaviors_to_plot[name][-1][0].set_width(-self.behaviors_to_plot[name][-1][0].get_x())
                    print "Behavior " + name + " stops"
                print "number of behaviors to plot = ", sum([len(behs) for behs in self.behaviors_to_plot.values()]), self.behaviors_to_plot

        plt.draw()
        plt.pause(0.1)

    def updateModule(self):
        # Don't forget the following:
        self.one_step(self.t)
        self.t += 1
        return True


if __name__ == '__main__':

    plt.ion()
    yarp.Network.init() 
    mod = AllostaticPlotModule()
    rf = yarp.ResourceFinder()
    rf.configure(sys.argv)
#    mod.configure(rf)

    mod.runModule(rf)
