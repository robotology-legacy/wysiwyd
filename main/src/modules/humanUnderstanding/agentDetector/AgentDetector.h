#include <yarp/os/RFModule.h>
#include <yarp/math/Math.h>
#include <kinectWrapper/kinectWrapper_client.h>
#include <wrdac/helpers.h>
#include <time.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace kinectWrapper;
using namespace wysiwyd::wrdac;

class AgentDetector: public RFModule
{
protected:
    //Kinect Related
    KinectWrapperClient client;
    ImageOf<PixelRgb> rgb;
    ImageOf<PixelMono16> depth;
    ImageOf<PixelFloat> depthToDisplay;
    ImageOf<PixelBgr> playersImage;
    ImageOf<PixelBgr> skeletonImage;
    IplImage* depthTmp;
    IplImage* rgbTmp;
    
    yarp::os::BufferedPort<Bottle> outputSkeletonPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> > depthPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > playersPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > skeletonPort;
        
    bool showImages;
    bool handleMultiplePlayers;
    Player joint;
    deque<Player> joints;
    Matrix players;

    //OPC/RFH related
    OPCClient* opc;
    Port   rfh;
    Port   rpc;
    bool isMounted;
    bool isCalibrated;
    Matrix kinect2icub;
    Agent* icub;
    Matrix icub2ir;
    int pointsCount;
    static bool clicked;
    static float clickX, clickY;

    //Agent Identity related
    bool useFaceRecognition;
    Port faceRecognizerModule;
    BufferedPort<Bottle> faceRecognizerModuleResults;
    Agent* partner;
    map<int, Agent*> identities;
    string currentTrainingFace;
	map<string, Vector> skeletonPatterns;
    double dSince;

    unsigned long dTimingLastApparition;        // time struct of the last appartition of an agent
    double dThresholdDisparition;           // timing maximal of non-reconnaissance of a agent, after thath we consider the agent as absent

public:

    bool configure(ResourceFinder &rf);

    bool checkCalibration();

    bool close();

    bool respond(const Bottle& cmd, Bottle& reply);

    double getPeriod();

    bool updateModule();

    void setIdentity(Player p, string name);

    string getIdentity(Player p);

    Vector transform2IR(Vector v);

	Vector getSkeletonPattern(Player p);

    static void click_callback( int event, int x, int y, int flags, void* param ){
	    //IplImage* image = (IplImage*) param;

	    switch( event ){

		    case CV_EVENT_LBUTTONDOWN:
                cout<<"Got a click."<<endl;
                AgentDetector::clickX = (float)x;
                AgentDetector::clickY = (float)y;
                AgentDetector::clicked = true;
			    break;

            default: break;
	    }
    }
};