from time import sleep
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import yarp


module_name = "allostatic_plot"


if __name__ == '__main__':

    yarp.Network.init() #Initialize network. This is mandatory

    homeo_rpc = yarp.Port()
    homeo_rpc.open("/" + module_name +"/to_homeo_rpc")
    yarp.Network.connect(homeo_rpc.getName(), "/homeostasis/rpc")
    request = yarp.Bottle()
    rep = yarp.Bottle()

    request.addString("names")
    homeo_rpc.write(request, rep)
    drives = []
    names = rep.get(0).asList()
    for i in range(names.size()):
        drives.append(names.get(i).asString())

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


    drive_value_ports = [yarp.BufferedPortBottle() for _ in drives] # Create a buff.Port
    for i, d in enumerate(drives):
        drive_value_ports[i].open("/" + module_name +"/" + d + ":i") # Open port (name)
        print yarp.Network.connect("/homeostasis/" + d + "/max:o", drive_value_ports[i].getName())

    win_size = 200

    fig = plt.figure()
    ax = plt.axes(xlim=(0, win_size), ylim=(0, 1))
    colors = 'g', 'b', 'w', 'k', 'r', 'y', 'c', 'm'
    value_lines = [ax.plot([], [], colors[i] + '-', lw=2) for i in range(len(drives))]


    drive_values = [[0.] * win_size for _ in drives]
    # drive_value_history = [drive_values for _ in range(win_size)]

    # def init():
    #     for line in value_lines:
    #         line.set_data([], [])
    #     return value_lines

    def animate(t, drive_values_as_list):
        # print drive_values_as_list[0]
        drive_values_as_list[0] = [values[1:] + [0.] for values in drive_values_as_list[0]]
        # drive_value_history[:-1] = drive_value_history[1:]
        for i, (port, homeo_max, line) in enumerate(zip(drive_value_ports, homeo_maxs, value_lines)):
            res = port.read()
            if res is not None:
                drive_values_as_list[0][i][-1] = res.get(0).asDouble() + homeo_max
                # drive_value_history[-1][i] = res.get(0).asDouble() + homeo_max
                # drive_values.append(res.get(0).asDouble() + homeo_max)
            else:
                drive_values_as_list[0][i][-1] = res
                # drive_values.append(res)
            # print line
            line[0].set_data(range(win_size), drive_values_as_list[0][i])
        return value_lines

    # call the animator.  blit=True means only re-draw the parts that have changed.
    anim = animation.FuncAnimation(fig, animate, fargs=([drive_values],), init_func=None,
                               frames=None, interval=100)

    plt.show()

    # x = range(200)
    # drive_value_history = []
    # while True:
    #     drive_values = []
    #     for port, homeo_max in zip(drive_value_ports, homeo_maxs):
    #         res = port.read()
    #         if res is not None:
    #             drive_values.append(res.get(0).asDouble() + homeo_max)
    #         else:
    #             drive_values.append(res)
    #     drive_value_history.append(drive_values)
    #     plt.plot(x, drive_value_history[-200:])
    #     plt.draw()
    #     yarp.Time.delay(0.1)

