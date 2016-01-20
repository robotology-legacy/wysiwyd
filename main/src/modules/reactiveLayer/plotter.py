from time import sleep
import matplotlib.pyplot as plt
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

    x = range(200)
    drive_value_history = []
    while True:
        drive_values = []
        for port, homeo_max in zip(drive_value_ports, homeo_maxs):
            res = port.read()
            if res is not None:
                drive_values.append(res.get(0).asDouble() + homeo_max)
            else:
                drive_values.append(res)
        drive_value_history.append(drive_values)
        plt.plot(x, drive_value_history[-200:])
        plt.draw()
        yarp.Time.delay(0.1)

