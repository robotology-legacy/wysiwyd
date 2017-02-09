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
    bool active, manualMode;
    Port *behaviorUnderPort;
    Port *behaviorOverPort;
    Port *homeoPort;
    BufferedPort<Bottle> *inputSensationPort;
    Bottle behaviorUnderCmd;
    Bottle behaviorOverCmd;
    Bottle sensationOnCmd, sensationOffCmd, beforeTriggerCmd, afterTriggerCmd;
    
    AllostaticDrive() {
        manualMode = true;
        behaviorUnderPort = nullptr;
        behaviorOverPort = nullptr;
        homeoPort = nullptr;
    }

    bool interrupt_ports() {
        if (behaviorUnderPort!=nullptr) {
            behaviorUnderPort->interrupt();
        }
        if (behaviorOverPort!=nullptr) {
            behaviorOverPort->interrupt();
        }
        inputSensationPort->interrupt();
        return true;
    }

    bool close_ports() {
        if (behaviorUnderPort!=nullptr) {
            behaviorUnderPort->interrupt();
            behaviorUnderPort->close();
            delete behaviorUnderPort;
            behaviorUnderPort=nullptr;
        }
        if (behaviorOverPort!=nullptr) {
            behaviorOverPort->interrupt();
            behaviorOverPort->close();
            delete behaviorOverPort;
            behaviorOverPort=nullptr;
        }
        inputSensationPort->interrupt();
        inputSensationPort->close();
        delete inputSensationPort;
        inputSensationPort=nullptr;
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
            if ( ! manualMode) {
                homeoPort->write(cmd,rply);
            }
            rplies.addList() = rply;
        }
        return rplies;
    }

    void triggerBehavior(OutCZ mode)
    {
        Bottle cmd, rply, rplies;
        // before trigger command
        if ( ! beforeTriggerCmd.isNull() && ! manualMode) {
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
        if ( ! afterTriggerCmd.isNull() && ! manualMode) {
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
    ICubClient  *iCub;

    Bottle drivesList;
    
    Port to_homeo_rpc, rpc_in_port, to_behavior_rpc;
    string moduleName;
    string homeo_name;

    double period;

    map<string, AllostaticDrive> allostaticDrives;


    vector<double> drivePriorities;
    double priority_sum;

    vector< yarp::os::BufferedPort<Bottle>* > outputM_ports;
    vector< yarp::os::BufferedPort<Bottle>* > outputm_ports;


    //Configuration
    void configureAllostatic(yarp::os::ResourceFinder &rf);

    bool openPorts(string driveName);
public:
    bool configure(yarp::os::ResourceFinder &rf);
    bool interruptModule();

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

    bool respond(const Bottle& cmd, Bottle& reply);
};
