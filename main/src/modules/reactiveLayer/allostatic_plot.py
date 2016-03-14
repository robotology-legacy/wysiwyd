#!/usr/bin/python

from copy import copy
from time import sleep
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import yarp


module_name = "allostatic_plot"


if __name__ == '__main__':
    try: 
        # yarp port stuff
        yarp.Network.init() 
        homeo_rpc = yarp.Port()
        homeo_rpc.open("/" + module_name +"/to_homeo_rpc")
        yarp.Network.connect(homeo_rpc.getName(), "/homeostasis/rpc")
        behaviorManager_rpc = yarp.Port()
        behaviorManager_rpc.open("/" + module_name +"/to_behaviorManager_rpc")
        yarp.Network.connect(behaviorManager_rpc.getName(), "/BehaviorManager/trigger:i")

        request = yarp.Bottle()
        rep = yarp.Bottle()

        # Retrieve drive names from the homeostasis module
        request.addString("names")
        homeo_rpc.write(request, rep)
        drives = []
        names = rep.get(0).asList()
        for i in range(names.size()):
            drives.append(names.get(i).asString())

        # Retrieve behavior names from the behaviorManager module
        request.clear()
        rep.clear()
        request.addString("names")
        behaviorManager_rpc.write(request, rep)
        behaviors = []
        behavior_ports = []
        names = rep.get(0).asList()
        for i in range(names.size()):
            behaviors.append(names.get(i).asString())
            behavior_ports.append(yarp.BufferedPortBottle())
            behavior_ports[-1].open("/" + module_name + "/behaviors/" + behaviors[-1] + ":i")
            yarp.Network.connect("/BehaviorManager/" + behaviors[-1] + "/start_stop:o", behavior_ports[-1].getName())

        # Retrieve homeostasis boundaries
        homeo_mins = []
        homeo_maxs = []
        for d in drives:
            request.clear()
            request.addString("ask") 
            request.addString(d) 
            request.addString("min") 
            rep.clear()
            homeo_rpc.write(request, rep)
            homeo_mins.append(rep.get(0).asDouble())

            request.clear()
            request.addString("ask") 
            request.addString(d) 
            request.addString("max") 
            rep.clear()
            homeo_rpc.write(request, rep)
            homeo_maxs.append(rep.get(0).asDouble())    


        # Connect to the homeostasis module to get drive values
        drive_value_ports = [yarp.BufferedPortBottle() for _ in drives] 
        for i, d in enumerate(drives):
            drive_value_ports[i].open("/" + module_name +"/" + d + ":i") 
            yarp.Network.connect("/homeostasis/" + d + "/max:o", drive_value_ports[i].getName())

        # time window size (= win_size * 0.1s)
        win_size = 200

        # Init matplotlib stuff
        min_val = min(homeo_mins)
        max_val = max(homeo_maxs)
        center_val = (max_val - min_val) / 2.
        y_min = -0.1  # ((min_val - center_val) * 1.25) + center_val
        y_max = 1.1  # ((max_val - center_val) * 1.25) + center_val
        fig = plt.figure()
        ax = plt.axes(xlim=(0, win_size), ylim=(y_min, y_max))
        colors = 'g', 'b', 'w', 'k', 'r', 'y', 'c', 'm'
        value_lines = [ax.plot([], [], colors[i] + '-', lw=2, label=d) for i,d in enumerate(drives)]
        homeo_min_lines = [ax.plot([], [], colors[i] + '--', lw=1) for i,d in enumerate(drives)]
        homeo_max_lines = [ax.plot([], [], colors[i] + '--', lw=1) for i,d in enumerate(drives)]
        plt.legend(drives)


        drive_values = [[0.] * win_size for _ in drives]

        behaviors_to_plot = {}  # plt.Rectangle(xy=(0,0), width=0, height=0)
        text_to_plot = {}

        has_started = False

        # Plot update function, called each 0.1s
        def animate(t, drive_values_as_list):
            global has_started, behaviors_to_plot
            drive_values_as_list[0] = [values[1:] + [0.] for values in drive_values_as_list[0]]
            for i, (port, homeo_max, v_line, min_line, max_line) in enumerate(zip(drive_value_ports, homeo_maxs, value_lines, homeo_min_lines, homeo_max_lines)):
                res = port.read()
                if res is not None:
                    drive_values_as_list[0][i][-1] = res.get(0).asDouble() + homeo_max
                else:
                    drive_values_as_list[0][i][-1] = res
                v_line[0].set_data(range(t- win_size, t), drive_values_as_list[0][i])
                min_line[0].set_data((t- win_size, t), (homeo_mins[i], homeo_mins[i]))
                max_line[0].set_data((t- win_size, t), (homeo_maxs[i], homeo_maxs[i]))
                ax.set_xlim(t- win_size, t)
            for name, port in zip(behaviors, behavior_ports):
                res = port.read(False)
                if res is not None:
                    msg = res.get(0).asString()
                    if msg == "start":
                        #behaviors_to_plot.append([t, -1, plt.Rectangle(xy=(t,y_min), width=10, height=(y_max-y_min)/20.)])
                        behaviors_to_plot[name] = plt.Rectangle(xy=(t,y_min), width=10000, height=(y_max-y_min)/20.)
                        plt.gca().add_patch(behaviors_to_plot[name])
                        text_to_plot[name] = plt.text(max(t, ax.get_xlim()[0]), y_min, name, horizontalalignment='left', color="white")
                        has_started = True
                        print "Behavior " + name + " starts"
                    elif msg == "stop" and has_started:
                        behaviors_to_plot[name].set_width(t - behaviors_to_plot[name].get_x())
                        #plt.text(behaviors_to_plot.get_x() + behaviors_to_plot.get_width(), 0., name, horizontalalignment='right', verticalalignment='center', transform=ax.transAxes)
                        # text_to_plot[name].set_transform(ax.transLimits)
                        text_to_plot[name].set_x(behaviors_to_plot[name].get_x() + behaviors_to_plot[name].get_width())
                        text_to_plot[name].set_horizontalalignment("right")
                            #behaviors_to_plot[-1][1] = copy(t)
                        print "Behavior " + name + " stops"
            # for i, (start, end, rect) in enumerate(behaviors_to_plot):
            #     if end == -1:
            #         rect.set_width(rect.get_x() + t - start)
            #     else:
            #         print rect.get_x() + end - start
            #         rect.set_width(rect.get_x() + end - start)
            #     plt.gca().add_patch(rect)


            return value_lines

        # Matplotlib animator (call the above funtion in a loop)

        anim = animation.FuncAnimation(fig, animate, fargs=([drive_values],), init_func=None,
                                       frames=None, interval=100)

        plt.show()
    except KeyboardInterrupt:
        print "Closing ports..."
        homeo_rpc.close()
        behaviorManager_rpc.close()
        for p in behavior_ports:
            p.close()
        for p in drive_value_ports:
            p.close()