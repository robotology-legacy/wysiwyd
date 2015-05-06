#include "PerspectiveTaking.h"

using namespace std;
using namespace yarp::os;
using namespace wysiwyd::wrdac;

bool perspectiveTaking::openHandlerPort() {
    string handlerPortName = "/" + getName() + "/rpc";

    if (!handlerPort.open(handlerPortName.c_str())) {
        yError() << getName() << ": Unable to open port " << handlerPortName;
        return false;
    }

    // attach to rpc port
    attach(handlerPort);

    return true;
}

bool perspectiveTaking::attach(RpcServer &source) {
    return this->yarp().attachAsServer(source);
}

void perspectiveTaking::connectToRFH(const string &rfhName) {
    string rfhLocal = "/"+getName()+"/rfh:o";
    rfh.open(rfhLocal.c_str());
    string rfhRemote = "/"+rfhName+"/rpc";

    yInfo() << "Connect " << rfhLocal << " with " << rfhRemote;
    while (!Network::connect(rfhLocal.c_str(),rfhRemote.c_str())) {
        yInfo() << "Waiting for connection to RFH...";
        Time::delay(1.0);
    }
}

void perspectiveTaking::connectToSFM(const string &sfmName) {
    string sfmLocal = "/"+getName()+"/sfm:o";
    sfm.open(sfmLocal.c_str());
    string sfmRemote = "/"+sfmName+"/rpc";

    yInfo() << "Connect " << sfmLocal << " with " << sfmRemote;
    while (!Network::connect(sfmLocal.c_str(),sfmRemote.c_str())) {
        yInfo() << "Waiting for connection to SFM...";
        Time::delay(1.0);
    }
}

void perspectiveTaking::connectToABM(const string &abmName) {
    string abmLocal = "/"+getName()+"/abm:o";
    abm.open(abmLocal.c_str());
    string abmRemote = "/"+abmName+"/rpc";

    int trial=0;
    while (!Network::connect(abmLocal.c_str(),abmRemote.c_str()) && trial<3) {
        yInfo() << "Waiting for connection to ABM...";
        trial++;
        Time::delay(0.2);
    }

    if(Network::isConnected(abmLocal.c_str(),abmRemote.c_str())) {
        isConnectedToABM = true;
    } else {
        isConnectedToABM = false;
    }
}

void perspectiveTaking::connectToAgentDetector(const string &agentDetectorName) {
    string agentDetectorLocal = "/"+getName()+"/agentdetector:o";
    agentdetector.open(agentDetectorLocal.c_str());
    string agentDetectorRemote = "/"+agentDetectorName+"/rpc";

    int trial=0;
    while (!Network::connect(agentDetectorLocal.c_str(),agentDetectorRemote.c_str()) && trial<3) {
        yInfo() << "Waiting for connection to Agent Detector...";
        trial++;
        Time::delay(0.2);
    }

    if(Network::isConnected(agentDetectorLocal.c_str(),agentDetectorRemote.c_str())) {
        lookDown = 0.5;
        isConnectedToAgentDetector = true;
    } else {
        isConnectedToAgentDetector = false;
    }
}

void perspectiveTaking::connectToHeadPoseEstimator(const string &headPoseEstimatorName) {
    string headPoseEstimatorLocal = "/"+getName()+"/headpose:o";
    headPoseEstimator.open(headPoseEstimatorLocal.c_str());
    string headPoseEstimatorRemote = "/"+headPoseEstimatorName+"/rpc";

    int trial=0;
    while (!Network::connect(headPoseEstimatorLocal.c_str(),headPoseEstimatorRemote.c_str()) && trial<3) {
        yInfo() << "Waiting for connection to Head Pose Estimator...";
        trial++;
        Time::delay(0.5);
    }

    if(Network::isConnected(headPoseEstimatorLocal.c_str(),headPoseEstimatorRemote.c_str())) {
        isConnectedToHeadPoseEstimator = true;
    } else {
        isConnectedToHeadPoseEstimator = false;
    }
}

void perspectiveTaking::connectToKinectServer(int verbosity) {
    string clientName = getName()+"/kinect";

    Property options;
    options.put("carrier","tcp");
    options.put("remote","kinectServer");
    options.put("local",clientName.c_str());
    options.put("verbosity",verbosity);

    while (!client.open(options)) {
        yInfo() << "Waiting for connection to KinectServer...";
        Time::delay(1.0);
    }
}

void perspectiveTaking::connectToOPC(const string &opcName) {
    opc = new OPCClient(getName());
    int trial=0;
    while (!opc->connect(opcName) && trial < 3) {
        yInfo() << "Waiting for connection to OPC...";
        trial++;
        Time::delay(0.2);
    }
    if(opc->isConnected()) {
        isConnectedToOPC = true;
    } else {
        isConnectedToOPC = false;
    }
}
