#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/SVD.h>
#include <yarp/math/Rand.h>
#include "wrdac/clients/icubClient.h"
#include <map>
// #include "internalVariablesDecay.h"

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
    // homeoPort should be buffered as well
    Port *homeoPort;
    BufferedPort<Bottle> *inputSensationPort;
    Bottle behaviorUnderCmd;
    Bottle behaviorOverCmd;
    Bottle sensationOnCmd, sensationOffCmd, triggerCmd;

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
        Port* port;
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
        // Bottle toSend;
        // toSend.clear();
        // toSend.addString(name);
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
    // int n_drives;
    Bottle drive_names;


    double period;
    double last_time;
	// InternalVariablesDecay* decayThread;

	//Drive triggers
	bool physicalInteraction;
	bool someonePresent;
    bool confusion;
    bool learning;
    bool finding;
    bool pointing;
	//Reflexes
	map<string, AllostaticDrive> allostaticDrives;


    vector<double> drivePriorities;
    vector<string> temporalDrivesList;
    vector<string> searchList;
    vector<string> pointList;
    double priority_sum;

    Port rpc;

    vector< yarp::os::Port* > rpc_ports;
    vector< yarp::os::BufferedPort<Bottle>* > outputM_ports;
    vector< yarp::os::BufferedPort<Bottle>* > outputm_ports;
    vector< yarp::os::BufferedPort<Bottle>* > inputSensationPorts;

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

    //Check for unknown tags in the opc
    bool handleTagging();

    //Check for objects to point to
    bool handlePointing();

	//Check for newcomers and salute them if required
	bool handleSalutation(bool& someoneIsPresent);

    //Retrieve and treat the tactile information input
    bool handleTactile();

    //Retrieve and treat the gesture information input
    bool handleGesture();

    //Handle a search command: look for object in opc or ask for it
    bool handleSearch();
    bool handlePoint();

	//Update the drives accordingly to the stimuli
	bool updateAllostatic();

	//Express emotions
	bool updateEmotions();

	//RPC & scenarios
	bool respond(const Bottle& cmd, Bottle& reply);

    bool Normalize(vector<double>& vec);

    bool createTemporalDrive(string name, double prior);

    // Choose a drive out of CZ, according to drive priorities
    DriveOutCZ chooseDrive();
};
