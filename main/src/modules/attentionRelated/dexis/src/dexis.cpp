

//   Include declaration file
#include "dexis.h"
//   Include math library
#include <cmath>
#include <cstdlib>
#include <vector>


bool babbler::configure(yarp::os::ResourceFinder &rf)
{
    // create an output port
    string moduleName = "dexis";
    string port2output = "/dexis";

    if (!portOutput.open(port2output.c_str())) {
        cout << getName() << ": Unable to open port " << port2output << endl;
        cout << "The microphone might be turned on" << endl;
    }

    cout << moduleName << ": finding configuration files..." << endl;

    rpc.open(("/" + moduleName + "/rpc").c_str());
    attach(rpc);


    return true;
}




bool babbler::close() {
    portOutput.close();
    rpc.close();
    return true;
}


/* Respond function */
bool babbler::respond(const yarp::os::Bottle& bCommand, yarp::os::Bottle& bReply)
{

    std::string helpMessage = std::string(getName().c_str()) +
        " commands are: \n" +
        "help \n" +
        "F + (int)Frequency\n" +
        "RF \n" +
        "quit \n";

    bReply.clear();
    std::string keyWord = bCommand.get(0).asString().c_str();

    if (keyWord == "quit") {
        bReply.addString("quitting");
        return false;
    }
    else if (keyWord == "help") {
        cout << helpMessage;
        bReply.addString("ok");
    }
    else if (keyWord == "F") {
        if (bCommand.size() == 2)
        {
            if (bCommand.get(1).isInt())
            {
                
                bReply.addInt(0);
            }
        }
    }
    else if (keyWord == "RF") {    
                bReply.addInt(0);
    }

    return true;
}



/* Called periodically every getPeriod() seconds */
bool babbler::updateModule() {


    return true;
}
