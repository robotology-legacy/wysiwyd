#include "recognitionOrder.h"

using namespace std;
using namespace yarp::os;

void RecognitionOrder::configure() {
    // Todo: set the value beow from a config file (but we are not in a module here)
    external_port_name = "/sam/rpc:i";
    from_sensation_port_name = "None";
    //this could be a port that sends a command 
    //describing what to classify e.g action/face/emotion...

    homeoPort = "/homeostasis/rpc";

    port_to_homeo_name = "/"+behaviorName+"/toHomeo:o";
    port_to_homeo.open(port_to_homeo_name);

    manual = true;
}

void RecognitionOrder::run(const Bottle &args) {
    yInfo() << "recognitionOrder::run";

    if (!Network::isConnected(port_to_homeo_name,homeoPort)){
        if (!Network::connect(port_to_homeo_name,homeoPort)){
            yWarning()<<"Port to Homeostasis not available. Could not freeze the drives...";
        }
    }
    if (Network::isConnected(port_to_homeo_name,homeoPort)){
        yInfo()<<"freezing drives";
        Bottle cmd;
        Bottle rply;
        cmd.addString("freeze");
        cmd.addString("all");

        port_to_homeo.write(cmd, rply);
    }

    yDebug() << "send rpc to SAM";
    //Bottle *order = sensation_port_in.read();
    
    Bottle cmd;
    Bottle rply;
    cmd.clear();
    cmd.addString("ask_action_label");
    yInfo() << "Recognising...";
    
    string toSay;

    rpc_out_port.write(cmd, rply);

    if(rply.get(0).asString() == "ack")
    {
        toSay = rply.get(1).asString();
    }
    else
    {
        toSay = "Sorry. I did not recognise that action";
    }
    iCub->lookAtPartner();
    iCub->say(toSay);
    iCub->home();
    yInfo() << "Recognising ends";

    if (!manual && Network::isConnected(port_to_homeo_name, homeoPort)){
        yInfo()<<"unfreezing drives";
        Bottle cmd;
        Bottle rply;
        cmd.addString("unfreeze");
        cmd.addString("all");
        port_to_homeo.write(cmd, rply);
    }
}
