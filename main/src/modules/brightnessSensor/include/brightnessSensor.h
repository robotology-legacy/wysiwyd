//#include "automaticCalibrationThread.h"

#include "wrdac/clients/icubClient.h"
using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;

class brightnessSensor : public RFModule {
private:

    //ICubClient *iCub;
    double      period;
    double      pBrightness;
    string      grammarToString(string sPath);
    //AutomaticCalibrationThread* calibrationThread;
    Port        rpc;
    int boolLeftCam;
    int boolRightCam;
    //string      nameMainGrammar;

    yarp::os::BufferedPort<ImageOf<PixelRgb> > Port2camRight;           // a port to get the input from camera
    yarp::os::BufferedPort<ImageOf<PixelRgb> > Port2camLeft;            // a port to get the input from camera
    yarp::os::BufferedPort<yarp::os::Bottle> Port2outBrightness;      // a port for publishing camera's brightness

    string Port2outBrightnessName;
    string Port2camLeftName;
    string Port2camRightName;

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

    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);
};
