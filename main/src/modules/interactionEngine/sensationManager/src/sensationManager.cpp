#include "sensationManager.h"


bool SensationManager::close()
{
    return true;
}


bool SensationManager::configure(yarp::os::ResourceFinder &rf)
{
    moduleName = rf.check("name",Value("SensationManager")).asString();
    setName(moduleName.c_str());
    cout<<moduleName<<": finding configuration files..."<<endl;
    period = rf.check("period",Value(0.1)).asDouble();

    Bottle grp = rf.findGroup("SENSATIONS");
    Bottle sensationList = *grp.find("sensations").asList();  
    for (int i = 0; i < sensationList.size(); i++)
    {
        string sensation_name = sensationList.get(i).asString();
        // behavior_names.push_back(behavior_name);
        if (sensation_name == "opcSensation") {
            sensations.push_back(new UnknownObjects());
        } else if (sensation_name == "test") {
            sensations.push_back(new Test());
        }
            // other behaviors here
        else{
            yDebug() << "Sensation " + sensation_name + " not implemented";
            return false;
        }
        sensations.back()->configure();
    }

    // for(std::vector<Behavior*>::iterator it = behaviors.begin(); it != behaviors.end(); ++it) {
    // }

    yInfo("Init done");

    return true;
}


bool SensationManager::updateModule()
{
    for(std::vector<Sensation*>::size_type i = 0; i != sensations.size(); i++) {
        sensations[i]->publish();
    }
    return true;


}


// bool BehaviorManager::respond(const Bottle& cmd, Bottle& reply)
// {
//     cout << cmd.get(0).asString() << endl;
//     if (cmd.get(0).asString() == "help" )
//     {   string help = "\n";
//         help += " ['behavior_name']  : Triggers corresponding behavior \n";
//         reply.addString(help);
//     }else
//     {
//         for(std::vector<Behavior*>::size_type i = 0; i != behaviors.size(); i++) {
//             if (cmd.get(0).asString() == behaviors[i]->name) {
    //                 behavior_to_trigger = i;
//             }
//         }

//         reply.addString("ack");
//     }
//     // strange:
//     reply.addString("nack");

//     return true;
// }
