#!/usr/bin/python

from copy import copy
from time import sleep
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import yarp





class AllostaticPlotModule(yarp.RFModule):
    def configure(self, rf):
        self.module_name = "allostatic_plot"
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

        # time window size (= win_size * 0.1s)
        self.win_size = 200

        # Init matplotlib stuff
        min_val = min(self.homeo_mins)
        max_val = max(self.homeo_maxs)
        center_val = (max_val - min_val) / 2.
        self.y_min = -0.1  # ((min_val - center_val) * 1.25) + center_val
        self.y_max = 1.1  # ((max_val - center_val) * 1.25) + center_val
        self.fig = plt.figure()
        self.ax = plt.axes(xlim=(0, self.win_size), ylim=(self.y_min, self.y_max))
        self.colors = 'g', 'b', 'w', 'm', 'r', 'y', 'c', 'k'
        self.value_lines = [self.ax.plot([], [], self.colors[i] + '-', lw=2, label=d) for i,d in enumerate(self.drives)]
        self.homeo_min_lines = [self.ax.plot([], [], self.colors[i] + '--', lw=1) for i,d in enumerate(self.drives)]
        self.homeo_max_lines = [self.ax.plot([], [], self.colors[i] + '--', lw=1) for i,d in enumerate(self.drives)]
        plt.legend(self.drives)


        self.drive_values = [[0.] * self.win_size for _ in self.drives]

        self.behaviors_to_plot = {}  # plt.Rectangle(xy=(0,0), width=0, height=0)
        self.text_to_plot = {}

        self.has_started = {}
        for name in self.behaviors:
            self.has_started[name] = False
        self.t = 0
        return True

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
        self.drive_values = [values[1:] + [0.] for values in self.drive_values]
        for i, (port, homeo_max, v_line, min_line, max_line) in enumerate(zip(self.drive_value_ports, self.homeo_maxs, self.value_lines, self.homeo_min_lines, self.homeo_max_lines)):
            res = port.read()
            if res is not None:
                self.drive_values[i][-1] = res.get(0).asDouble() + homeo_max
            else:
                self.drive_values[i][-1] = res
            v_line[0].set_data(range(t- self.win_size, t), self.drive_values[i])
            min_line[0].set_data((t- self.win_size, t), (self.homeo_mins[i], self.homeo_mins[i]))
            max_line[0].set_data((t- self.win_size, t), (self.homeo_maxs[i], self.homeo_maxs[i]))
            self.ax.set_xlim(t- self.win_size, t)
        for name, port in zip(self.behaviors, self.behavior_ports):
            res = port.read(False)
            if res is not None:
                msg = res.get(0).asString()
                if msg == "start":
                    #behaviors_to_plot.append([t, -1, plt.Rectangle(xy=(t,y_min), width=10, height=(y_max-y_min)/20.)])
                    self.behaviors_to_plot[name] = plt.Rectangle(xy=(t, self.y_min), width=10000, height=(self.y_max-self.y_min)/20.)
                    plt.gca().add_patch(self.behaviors_to_plot[name])
                    self.text_to_plot[name] = plt.text(max(t, self.ax.get_xlim()[0]), self.y_min, name, horizontalalignment='left', color="white")
                    self.has_started[name] = True
                    print "Behavior " + name + " starts"
                elif msg == "stop" and self.has_started[name]:
                    self.behaviors_to_plot[name].set_width(t - self.behaviors_to_plot[name].get_x())
                    #plt.text(behaviors_to_plot.get_x() + behaviors_to_plot.get_width(), 0., name, horizontalalignment='right', verticalalignment='center', transform=ax.transAxes)
                    # text_to_plot[name].set_transform(ax.transLimits)
                    self.text_to_plot[name].set_x(self.behaviors_to_plot[name].get_x() + self.behaviors_to_plot[name].get_width())
                    self.text_to_plot[name].set_horizontalalignment("right")
                        #behaviors_to_plot[-1][1] = copy(t)
                    print "Behavior " + name + " stops"
        
    def animate(self):
        self.anim = animation.FuncAnimation(self.fig, self.one_step, fargs=None, init_func=None,
                                        frames=None, interval=100)
        plt.show()

    def updateModule(self):
        # Don't forget the following:
        return True


if __name__ == '__main__':
    # try: 
        # yarp port stuff
    yarp.Network.init() 
    mod = AllostaticPlotModule()
    rf = yarp.ResourceFinder()
    mod.configure(rf)

    
    mod.animate()
    # mod.runModule(rf)


    #     # Plot update function, called each 0.1s
    #     def animate(t, drive_values_as_list):
    #         global has_started, behaviors_to_plot
    #         drive_values_as_list[0] = [values[1:] + [0.] for values in drive_values_as_list[0]]
    #         for i, (port, homeo_max, v_line, min_line, max_line) in enumerate(zip(drive_value_ports, homeo_maxs, value_lines, homeo_min_lines, homeo_max_lines)):
    #             res = port.read()
    #             if res is not None:
    #                 drive_values_as_list[0][i][-1] = res.get(0).asDouble() + homeo_max
    #             else:
    #                 drive_values_as_list[0][i][-1] = res
    #             v_line[0].set_data(range(t- win_size, t), drive_values_as_list[0][i])
    #             min_line[0].set_data((t- win_size, t), (homeo_mins[i], homeo_mins[i]))
    #             max_line[0].set_data((t- win_size, t), (homeo_maxs[i], homeo_maxs[i]))
    #             ax.set_xlim(t- win_size, t)
    #         for name, port in zip(behaviors, behavior_ports):
    #             res = port.read(False)
    #             if res is not None:
    #                 msg = res.get(0).asString()
    #                 if msg == "start":
    #                     #behaviors_to_plot.append([t, -1, plt.Rectangle(xy=(t,y_min), width=10, height=(y_max-y_min)/20.)])
    #                     behaviors_to_plot[name] = plt.Rectangle(xy=(t,y_min), width=10000, height=(y_max-y_min)/20.)
    #                     plt.gca().add_patch(behaviors_to_plot[name])
    #                     text_to_plot[name] = plt.text(max(t, ax.get_xlim()[0]), y_min, name, horizontalalignment='left', color="white")
    #                     has_started = True
    #                     print "Behavior " + name + " starts"
    #                 elif msg == "stop" and has_started:
    #                     behaviors_to_plot[name].set_width(t - behaviors_to_plot[name].get_x())
    #                     #plt.text(behaviors_to_plot.get_x() + behaviors_to_plot.get_width(), 0., name, horizontalalignment='right', verticalalignment='center', transform=ax.transAxes)
    #                     # text_to_plot[name].set_transform(ax.transLimits)
    #                     text_to_plot[name].set_x(behaviors_to_plot[name].get_x() + behaviors_to_plot[name].get_width())
    #                     text_to_plot[name].set_horizontalalignment("right")
    #                         #behaviors_to_plot[-1][1] = copy(t)
    #                     print "Behavior " + name + " stops"
    #         # for i, (start, end, rect) in enumerate(behaviors_to_plot):
    #         #     if end == -1:
    #         #         rect.set_width(rect.get_x() + t - start)
    #         #     else:
    #         #         print rect.get_x() + end - start
    #         #         rect.set_width(rect.get_x() + end - start)
    #         #     plt.gca().add_patch(rect)


    #         return value_lines

    #     # Matplotlib animator (call the above funtion in a loop)

    #     anim = animation.FuncAnimation(fig, animate, fargs=([drive_values],), init_func=None,
    #                                    frames=None, interval=100)

    #     plt.show()
    # except KeyboardInterrupt: