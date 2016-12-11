#!/usr/bin/env python

from copy import copy
from time import sleep
import matplotlib.pyplot as plt

import yarp

def change_drive_names(drive_names):
    new_names = []
    for name in drive_names:
        if name == "exploration":
            new_names.append("Knowledge exploration")
        elif name == "demonstration":
            new_names.append("Knowledge exploitation")
        elif name == "narration":
            new_names.append("History narration")
        elif name == "dummy":
            new_names.append("This is a test")
        else:
            new_names.append(name)
    return new_names

class AllostaticPlotModule(yarp.RFModule):
    def configure(self, rf):

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
        self.ax = plt.axes(xlim=(0, self.win_size), ylim=(self.y_min, self.y_max))
        self.colors = 'g', 'b', 'm', 'r', 'y', 'c', 'k'
        self.value_lines = [self.ax.plot([], [], self.colors[i] + '-', lw=2, label=d) for i,d in enumerate(self.drives)]
        self.homeo_min_lines = [self.ax.plot([], [], self.colors[i] + '--', lw=1) for i,d in enumerate(self.drives)]
        self.homeo_max_lines = [self.ax.plot([], [], self.colors[i] + '--', lw=1) for i,d in enumerate(self.drives)]
        plt.legend(change_drive_names(self.drives))
        self.ax.set_xlim(0- self.win_size, 0)


        self.drive_values = [[0.] * self.win_size for _ in self.drives]

        self.behaviors_to_plot = []  # plt.Rectangle(xy=(0,0), width=0, height=0)
        self.text_to_plot = {}

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

        for plotitem in self.behaviors_to_plot:
            plotitem[1].set_x(plotitem[1].get_x()-1)
            plotitem[2].set_x(plotitem[1].get_x()+5)

        for name, port in zip(self.behaviors, self.behavior_ports):
            res = port.read(False)
            if res is not None:
                msg = res.get(0).asString()
                if msg == "start":
                    new_rectangle = plt.Rectangle(xy=(0, self.y_min), width=10000, height=(self.y_max-self.y_min)/20.)
                    plt.gca().add_patch(new_rectangle)
                    new_text = plt.text(5, self.y_min+5, name, horizontalalignment='left', color="black")
                    self.behaviors_to_plot.append((name, new_rectangle, new_text))
                    print "Behavior " + name + " starts"
                elif msg == "stop":
                    plot_idx = None
                    for idx, plotitem in enumerate(self.behaviors_to_plot):
                        if plotitem[0] == name:
                            plot_idx = idx
                    self.behaviors_to_plot[idx][1].set_width(-self.behaviors_to_plot[idx][1].get_x())
                    print "Behavior " + name + " stops"

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
#    mod.configure(rf)

    mod.runModule(rf)
