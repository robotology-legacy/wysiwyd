#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Rand.h>
#include <map>
#include <wrdac/clients/clients.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace wysiwyd::wrdac;

enum OutCZ {UNDER, OVER};

struct DriveOutCZ {
    string name;
    OutCZ level;

};

enum DriveUpdateMode {SENSATION_ON, SENSATION_OFF};

class AllostaticDrive
{
public: 
    string name;
    bool active;
    Port *behaviorUnderPort;
    Port *behaviorOverPort;
    Port *homeoPort;
    BufferedPort<Bottle> *inputSensationPort;
    Bottle behaviorUnderCmd;
    Bottle behaviorOverCmd;
    Bottle sensationOnCmd, sensationOffCmd, beforeTriggerCmd, afterTriggerCmd;
    

    bool close_ports() {
        if (behaviorUnderPort) {
            behaviorUnderPort->interrupt();
            behaviorUnderPort->close();
        }
        if (behaviorOverPort) {
            behaviorOverPort->interrupt();
            behaviorOverPort->close();
        }
        homeoPort->interrupt();
        homeoPort->close();
        inputSensationPort->interrupt();
        inputSensationPort->close();
        return true;
    }

    Bottle update(DriveUpdateMode mode)
    {
        Bottle cmds;
        switch (mode) {
            case SENSATION_ON:
                cmds = sensationOnCmd;
                break;
            case SENSATION_OFF:
                cmds = sensationOffCmd;
                break;
            default:
                yDebug() << "Update mode not implemented";
                yDebug() << to_string(mode);
                break;
        }
        Bottle rplies;
        rplies.clear();
        for (int i=0; i<cmds.size(); i++){
            Bottle rply;
            rply.clear();
            Bottle cmd = *cmds.get(i).asList();        
            homeoPort->write(cmd,rply);
            rplies.addList() = rply;
        }
        return rplies;
    }

    void triggerBehavior(OutCZ mode)
    {

        Bottle cmd, rply, rplies;
        // before trigger command
        if ( ! beforeTriggerCmd.isNull()) {
            cmd.clear();
            rply.clear();
            rplies.clear();
            for (int i=0; i<beforeTriggerCmd.size(); i++){
                rply.clear();
                Bottle cmd = *beforeTriggerCmd.get(i).asList();   
                yDebug() << cmd.toString();     
                homeoPort->write(cmd,rply);
                rplies.addList() = rply;
            }        
        }

        Port* port = NULL;
        switch (mode) {
            case UNDER:
                cmd = behaviorUnderCmd;
                port = behaviorUnderPort;
                break;
            case OVER:
                cmd = behaviorOverCmd;
                port = behaviorOverPort;
                break;   
            default:
                yDebug() << "Trigger mode not implemented";
                yDebug() << to_string(mode);
                break;
        }
        
        yInfo() << "Drive " + name + " to be triggered via " << port->getName();
        port->write(cmd, rply);
        
        // after trigger command
        if ( ! afterTriggerCmd.isNull()) {
            cmd.clear();
            rply.clear();
            rplies.clear();
            for (int i=0; i<afterTriggerCmd.size(); i++){
                rply.clear();
                Bottle cmd = *afterTriggerCmd.get(i).asList();   
                yDebug() << cmd.toString();     
                homeoPort->write(cmd,rply);
                rplies.addList() = rply;
            yDebug() << "triggerBehavior completed.";

            }        
        }

        // record event in ABM
        // string moduleName = rf.check("name",Value("AllostaticController")).asString();
        // setName(moduleName.c_str());
 
        
    }
};

class AllostaticController: public RFModule
{
private:

    Bottle drivesList;
    
    Port to_homeo_rpc;
    string moduleName;
    string homeo_name;

    double period;

    map<string, AllostaticDrive> allostaticDrives;

    yarp::os::ContactStyle style;

    vector<double> drivePriorities;
    double priority_sum;

    vector< yarp::os::BufferedPort<Bottle>* > outputM_ports;
    vector< yarp::os::BufferedPort<Bottle>* > outputm_ports;

    //Configuration
    void configureAllostatic(yarp::os::ResourceFinder &rf);

    int openPorts(string driveName);
public:
    bool configure(yarp::os::ResourceFinder &rf);
    ICubClient  *iCub;
    bool interruptModule()
    {
        return true;
    }

    bool close();

    double getPeriod()
    {
        return period;
    }

    bool updateModule();

    //Update the drives accordingly to the stimuli
    bool updateAllostatic();

    bool Normalize(vector<double>& vec);

    // Choose a drive out of CZ, according to drive priorities
    DriveOutCZ chooseDrive();
};
