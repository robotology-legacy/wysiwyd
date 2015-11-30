#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Rand.h>
#include <map>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

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
    Bottle sensationOnCmd, sensationOffCmd, triggerCmd;

    bool close_ports() {
        if (behaviorUnderPort)
            behaviorUnderPort->close();
        if (behaviorOverPort)
            behaviorOverPort->close();
        homeoPort->close();
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
                yDebug() << "mode not implemented";
                break;
        }
        Bottle rplies;
        rplies.clear();
        for (int i=0; i<cmds.size(); i++){
            Bottle rply;
            rply.clear();
            // drivesSensationOn.push_back(*cmds.get(i).asList())
            Bottle cmd = *cmds.get(i).asList();        
            // yDebug() << cmd.toString();
            homeoPort->write(cmd,rply);
            rplies.addList() = rply;
        }
        return rplies;
    }

    void triggerBehavior(OutCZ mode)
    {
        Bottle cmd, rply;
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
                yDebug() << "mode not implemented";
                break;
        }
        
        yInfo() << "Drive " + name + " to be triggered via " << port->getName();
        port->write(cmd, rply);
        if ( ! triggerCmd.isNull()) {
            Bottle rplies;
            rplies.clear();
            for (int i=0; i<triggerCmd.size(); i++){
                rply.clear();
                Bottle cmd = *triggerCmd.get(i).asList();        
                homeoPort->write(cmd,rply);
                rplies.addList() = rply;
            }        
        }
    }
};

class AllostaticController: public RFModule
{
private:

    Bottle drivesList;
    
    Port to_homeo_rpc;
    Port ears_port;
    string moduleName;
    string homeo_name;

    double period;
    double last_time;

	map<string, AllostaticDrive> allostaticDrives;


    vector<double> drivePriorities;
    double priority_sum;

    vector< yarp::os::BufferedPort<Bottle>* > outputM_ports;
    vector< yarp::os::BufferedPort<Bottle>* > outputm_ports;

    //Configuration
	void configureAllostatic(yarp::os::ResourceFinder &rf);

    int openPorts(string driveName);
public:
    bool configure(yarp::os::ResourceFinder &rf);

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
