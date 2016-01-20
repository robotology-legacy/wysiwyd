from time import sleep
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import yarp


module_name = "allostatic_plot"


if __name__ == '__main__':

    # yarp port stuff
    yarp.Network.init() 
    homeo_rpc = yarp.Port()
    homeo_rpc.open("/" + module_name +"/to_homeo_rpc")
    yarp.Network.connect(homeo_rpc.getName(), "/homeostasis/rpc")
    request = yarp.Bottle()
    rep = yarp.Bottle()

    # Retrieve drive names from the homeostasis module
    request.addString("names")
    homeo_rpc.write(request, rep)
    drives = []
    names = rep.get(0).asList()
    for i in range(names.size()):
        drives.append(names.get(i).asString())

    # Retrieve honeostasis boundaries
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
        print yarp.Network.connect("/homeostasis/" + d + "/max:o", drive_value_ports[i].getName())

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

    # Plot update function, called each 0.1s
    def animate(t, drive_values_as_list):
        drive_values_as_list[0] = [values[1:] + [0.] for values in drive_values_as_list[0]]
        for i, (port, homeo_max, v_line, min_line, max_line) in enumerate(zip(drive_value_ports, homeo_maxs, value_lines, homeo_min_lines, homeo_max_lines)):
            res = port.read()
            if res is not None:
                drive_values_as_list[0][i][-1] = res.get(0).asDouble() + homeo_max
            else:
                drive_values_as_list[0][i][-1] = res
            v_line[0].set_data(range(win_size), drive_values_as_list[0][i])
            min_line[0].set_data((0, win_size), (homeo_mins[i], homeo_mins[i]))
            print homeo_maxs[i]
            max_line[0].set_data((0, win_size), (homeo_maxs[i], homeo_maxs[i]))
        return value_lines

    # Matplotlib animator (call the above funtion in a loop)
    anim = animation.FuncAnimation(fig, animate, fargs=([drive_values],), init_func=None,
                                   frames=None, interval=100)

    plt.show()
