#include "cvz/core/all.h"
#include "cvz/gui/all.h"
#include <yarp/os/all.h>

using namespace std;
using namespace yarp::os;

class CvzFiberModule :public RFModule
{
	cvz::core::CvzFiber* fiber;

public:
	bool configure(yarp::os::ResourceFinder &rf)
	{
		fiber = new cvz::core::CvzFiber();
		yarp::os::Property prop; prop.fromConfigFile(rf.findFile("from"));
        attachTerminal();
		return fiber->configure(prop);
	}

    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
    {
        std::string key = command.get(0).asString();
        reply.addString("ACK");
        
        if (key == "saveRF")
        {
            IplImage* img;
            Bottle *bTarget = command.get(1).asList();
            if (bTarget)
            {
                if (bTarget->size() == 1)
                {
                    //Saving a layer
                    int layer = bTarget->get(0).asInt();
                    img = fiber->layers[layer].getReceptiveField();
                }
                else if (bTarget->size() == 3)
                {
                    //Saving a layer
                    int layer = bTarget->get(0).asInt();
                    int x = bTarget->get(1).asInt();
                    int y = bTarget->get(2).asInt();
                    img = fiber->layers[layer].getReceptiveField(x,y);
                }
                else
                {
                    reply.addString("saveRF wrong format. Expected saveRF (layer) or saveRF (layer x y)");
                    return true;
                }

                std::stringstream fileName;
                fileName << "layer_" << bTarget->toString() << ".jpg";
                cout << "Saving receptive fields of " << fileName.str() << " --> " << cvSaveImage(fileName.str().c_str(), img) << endl;
                cvReleaseImage(&img);
            }
            else
            {
                reply.addString("saveRF wrong format. Expected saveRF (layer) or saveRF (layer x y)");
                return true;
            }
        }


        if (key == "saveRRF")
        {
            IplImage* img;
            Bottle *bTarget = command.get(1).asList();
            if (bTarget)
            {
                if (bTarget->size() == 1)
                {
                    //Saving a layer
                    int layer = bTarget->get(0).asInt();
                    img = fiber->getRecursiveRFLayer(layer);
                }
                else
                {
                    reply.addString("saveRF wrong format. Expected saveRF (layer) or saveRF (layer x y)");
                    return true;
                }

                std::stringstream fileName;
                fileName << "layer_" << bTarget->toString() << "_recursive.jpg";
                cout << "Saving recursively receptive fields of " << fileName.str() << " --> " << cvSaveImage(fileName.str().c_str(), img) << endl;
                cvReleaseImage(&img);
            }
            else
            {
                reply.addString("saveRF wrong format. Expected saveRRF (layer)");
                return true;
            }
        }
        return true;
    }

    double getPeriod()
    {
        return 0.01;
    }

	bool updateModule()
	{
		fiber->cycle();
		return true;
	}

	bool close()
	{
		delete fiber;
		return true;
	}
};

int main(int argc, char * argv[])
{
	Network yarp;
	if (!Network::checkNetwork())
	{
		cout << "yarp network is not available!" << endl;
		return 0;
	}

	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultContext("cvz");
	rf.setDefaultConfigFile("defaultFiber.ini"); //overridden by --from parameter
	rf.configure(argc, argv);

	CvzFiberModule cvzFiber;
	if (cvzFiber.configure(rf))
		cvzFiber.runModule();
	else
		cout << "Unable to configure the cvz fiber module." << endl;

	return 0;
}