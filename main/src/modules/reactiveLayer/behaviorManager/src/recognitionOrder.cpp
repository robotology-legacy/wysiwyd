#include "recognitionOrder.h"

void recognitionOrder::configure() {
    // Todo: set the value beow from a config file (but we are not in a module here)
    external_port_name = "/SAM/rpc";
    from_sensation_port_name = "None";
    //this could be a port that sends a command 
    //describing what to classify e.g action/face/emotion...
}

void recognitionOrder::run(Bottle args/*=Bottle()*/) {
    yInfo() << "recognitionOrder::run";
    yDebug() << "send rpc to SAM";
    //Bottle *order = sensation_port_in.read();
    Bottle order;
    order.addString("action");
    string id = order.get(0).asString();
    string toSay;
    
    Bottle cmd;
    Bottle rply;
    cmd.clear();
    cmd.addString("ask_"+id+"_label");
    yInfo() << "Recognising...";
    
    rpc_out_port.write(cmd, rply);

    if(rply.get(0).asString() == "ack")
    {
        toSay = "You just did a " + rply.get(1).asString() + " action";
    }
    else
    {
        toSay = "Sorry. I did not recognise that action";
    }
    iCub->say(toSay);
    yInfo() << "Recognising ends";
}
