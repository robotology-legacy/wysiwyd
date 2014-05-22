#include "cvz/core/all.h"
#include "cvz/gui/all.h"
#include <cv.h>
#include <cvaux.h>
#include "highgui.h"
#include <yarp/os/all.h>

using namespace std;
using namespace yarp::os;

#define V1_RETINA_W	10
#define V1_RETINA_H	10
#define V1_RETINA_L	1
#define V1_RETINA_LEARNING 0.005
#define V1_RETINA_SIGMA_FACTOR 1.5

#define V1_FOVEA_W	10
#define V1_FOVEA_H	10
#define V1_FOVEA_L	1
#define V1_FOVEA_LEARNING 0.01
#define V1_FOVEA_SIGMA_FACTOR 1.5

#define V2_W	30
#define V2_H	30
#define V2_L	1
#define V2_LEARNING 0.005
#define V2_SIGMA_FACTOR 1.5

void replace_all(std::string & in, const std::string & plain, const std::string & tok)
{
	std::string::size_type n = 0;
	const std::string::size_type l = plain.length();
	while (1){
		n = in.find(plain, n);
		if (n != std::string::npos){
			in.replace(n, l, tok);
		}
		else{
			break;
		}
	}
}

class StackRpcWrapper : public BufferedPort<Bottle>, public yarp::os::RateThread
{
	cvz::core::CvzStack* stack;
	cvz::core::CvzMMCM* v2;
	//cvz::core::CvzMMCM* v1;
	cvz::core::CvzMMCM* v1Fovea;
	cvz::core::CvzMMCM* v1Retina;
	cvz::core::CvzMMCM* gaze;
	vector< vector< cvz::core::CvzMMCM* > > retina;
	vector< vector< cvz::core::CvzMMCM* > > fovea;
	int autoIncrementCnt;
private:
	void saveRetinaLikeRF(const vector< vector< cvz::core::CvzMMCM* > > &retinaUsed)
	{
		for (int i = 0; i < retinaUsed.size(); i++)
		{
			for (int j = 0; j < retinaUsed[0].size(); j++)
			{
				//First we probe the size of one RF. In theory only one bottom up modality
				yarp::sig::ImageOf<yarp::sig::PixelRgb> probe = retinaUsed[i][j]->getReceptiveFieldRepresentation(0, 0, 0, retinaUsed[i][j]->modalitiesBottomUp.begin()->second);
				int totalW = probe.width() * retinaUsed[i][j]->W() * retinaUsed[i][j]->L(); //We display the potential layers next to each other
				int totalH = probe.height() * retinaUsed[i][j]->H();
				IplImage* fullImg = cvCreateImage(cvSize(totalW, totalH), 8, 3);

				for (map<string, cvz::core::IModality* >::iterator mod = retinaUsed[i][j]->modalitiesBottomUp.begin(); mod != retinaUsed[i][j]->modalitiesBottomUp.end(); mod++)
				{
					yarp::sig::ImageOf<yarp::sig::PixelRgb> img;
					for (int x = 0; x < retinaUsed[i][j]->W(); x++)
					{
						for (int y = 0; y < retinaUsed[i][j]->H(); y++)
						{
							for (int z = 0; z < retinaUsed[i][j]->L(); z++)
							{
								yarp::sig::ImageOf<yarp::sig::PixelRgb> thumbnail = retinaUsed[i][j]->getReceptiveFieldRepresentation(x, y, z, mod->second);
								cvSetImageROI(fullImg, cvRect(x*probe.width() + z*probe.width(), y*probe.height(), probe.width(), probe.height()));
								cvCopyImage(thumbnail.getIplImage(), fullImg);
								cvResetImageROI(fullImg);
							}
						}
					}
					std::stringstream fileName;
					fileName<< retinaUsed[i][j]->getName() << mod->first <<"__"<<autoIncrementCnt<<".jpg";
					std::string finalName = fileName.str();
					replace_all(finalName, "/", "_");
					finalName = "rfImages/" + finalName;
					cout << "Saving receptive fields of " << finalName << " --> " << cvSaveImage(finalName.c_str(), fullImg) << endl;;
				}
			}
		}
	}

	IplImage* getV1RFSingleNeuron(cvz::core::CvzMMCM* lgnUsed, const vector< vector< cvz::core::CvzMMCM* > > &retinaUsed, int v1x, int v1y, int v1z)
	{
		//Get the size of one retina "pixel"
		yarp::sig::ImageOf<yarp::sig::PixelRgb> probe = retinaUsed[0][0]->getReceptiveFieldRepresentation(0, 0, 0, retinaUsed[0][0]->modalitiesBottomUp.begin()->second);
		int retinaPixelW = probe.width() * retinaUsed.size(); //We display the potential layers next to each other
		int retinaPixelH = probe.height() * retinaUsed[0].size();

		//Prepare the image of a single V1 neuron RF
		IplImage* fullRFSingleNeuron = cvCreateImage(cvSize(retinaPixelW, retinaPixelH), 8, 3);

		//Go through all the bottom up modalities ( we assume only retinas at this level)
		for (int x = 0; x < retinaUsed.size(); x++)
		{
			for (int y = 0; y < retinaUsed[0].size(); y++)
			{
				std::stringstream ssV1Name;
				ssV1Name << "/" << lgnUsed->getName() << "/in_" << x << "_" << y;
				vector<double> rfv1 = lgnUsed->getReceptiveFieldWeights(v1x, v1y, v1z, lgnUsed->modalitiesBottomUp[ssV1Name.str()]);


				std::stringstream upModalityName;
				upModalityName << "/" << retinaUsed[x][y]->getName() << "/v1";
				int bmuX, bmuY, bmuZ;
				retinaUsed[x][y]->getBestMatchingUnit(rfv1, retinaUsed[x][y]->modalitiesTopDown[upModalityName.str()], bmuX, bmuY, bmuZ);

				yarp::sig::ImageOf<yarp::sig::PixelRgb> px = retinaUsed[x][y]->getReceptiveFieldRepresentation(bmuX, bmuY, bmuZ, retinaUsed[x][y]->modalitiesBottomUp.begin()->second);
				cvSetImageROI(fullRFSingleNeuron, cvRect(x*probe.width(), y*probe.height(), probe.width(), probe.height()));
				cvCopyImage(px.getIplImage(), fullRFSingleNeuron);
				cvResetImageROI(fullRFSingleNeuron);
			}
		}
		return fullRFSingleNeuron;
	}

	void saveV1RF(cvz::core::CvzMMCM* lgnUsed, const vector< vector< cvz::core::CvzMMCM* > > &retinaUsed)
	{
		//Get the size of one retina "pixel"
		yarp::sig::ImageOf<yarp::sig::PixelRgb> probe = retinaUsed[0][0]->getReceptiveFieldRepresentation(0, 0, 0, retinaUsed[0][0]->modalitiesBottomUp.begin()->second);
		int retinaPixelW = probe.width() * retinaUsed.size(); //We display the potential layers next to each other
		int retinaPixelH = probe.height() * retinaUsed[0].size();

		//Prepare the full image
		int border = 5;
		int totalW = (retinaPixelW + border) * lgnUsed->W() * lgnUsed->L();
		int totalH = (retinaPixelH + border) * lgnUsed->H();

		IplImage* fullRF = cvCreateImage(cvSize(totalW, totalH), 8, 3);

		//Go through all the neurons of lgnUsed
		for (int v1x = 0; v1x < lgnUsed->W(); v1x++)
		{
			for (int v1y = 0; v1y < lgnUsed->H(); v1y++)
			{
				for (int v1z = 0; v1z < lgnUsed->L(); v1z++)
				{
					IplImage*fullRFSingleNeuron = getV1RFSingleNeuron(lgnUsed, retinaUsed, v1x, v1y, v1z);
					cvSetImageROI(fullRF, cvRect((v1x + v1z)*(border + fullRFSingleNeuron->width), v1y*(border+fullRFSingleNeuron->height), fullRFSingleNeuron->width, fullRFSingleNeuron->height));
					cvCopyImage(fullRFSingleNeuron, fullRF);
					cvResetImageROI(fullRF);
					cvReleaseImage(&fullRFSingleNeuron);
				}
			}
		}

		std::stringstream fileName;
		fileName<< "rfImages/" << lgnUsed->getName() <<"__"<<autoIncrementCnt<<".jpg";
		cout << "Saving receptive fields of " << fileName.str() << " --> " << cvSaveImage(fileName.str().c_str(), fullRF) <<endl;
		cvReleaseImage(&fullRF);
	}

	void saveV2RF()
	{
		//1 - Do the retinajob

		//Get the size of one retina "pixel"
		yarp::sig::ImageOf<yarp::sig::PixelRgb> probe = retina[0][0]->getReceptiveFieldRepresentation(0, 0, 0, retina[0][0]->modalitiesBottomUp.begin()->second);
		int retinaPixelW = probe.width() * retina.size(); //We display the potential layers next to each other
		int retinaPixelH = probe.height() * retina[0].size();

		//Prepare the full image
		int border = 5;
		int totalW = (retinaPixelW + border) * v2->W() * v2->L();
		int totalH = (retinaPixelH + border) * v2->H();

		IplImage* fullRF = cvCreateImage(cvSize(totalW, totalH), 8, 3);

		//Go through all the neurons of lgnUsed
		for (int v2x = 0; v2x < v2->W(); v2x++)
		{
			for (int v2y = 0; v2y < v2->H(); v2y++)
			{
				for (int v2z = 0; v2z < v2->L(); v2z++)
				{
					//stack.connectModalities("/gaze/v1", "/v2/gaze");
					//stack.connectModalities("/v1Retina/out", "/v2/v1Retina");
					//stack.connectModalities("/v1Fovea/out", "/v2/v1Fovea");

					vector<double> rfv2 = v2->getReceptiveFieldWeights(v2x, v2y, v2z, v2->modalitiesBottomUp["/v2/v1Retina"]);
					int bmuX, bmuY, bmuZ;
					v1Retina->getBestMatchingUnit(rfv2, v1Retina->modalitiesTopDown["/v1Retina/out"], bmuX, bmuY, bmuZ);
					IplImage* px = getV1RFSingleNeuron(v1Retina, retina, bmuX, bmuY, bmuZ);;

					cvSetImageROI(fullRF, cvRect((v2x + v2z)*(border + px->width), v2y*(border + px->height), px->width, px->height));
					cvCopyImage(px, fullRF);
					cvResetImageROI(fullRF);
					cvReleaseImage(&px);
				}
			}
		}

		std::stringstream fileName;
		fileName << "rfImages/" << v2->getName() << "_retina__" << autoIncrementCnt << ".jpg";
		cout << "Saving receptive fields of " << fileName.str() << " --> " << cvSaveImage(fileName.str().c_str(), fullRF) << endl;
		cvReleaseImage(&fullRF);

		//2 - Do the foveajob

		//Get the size of one retina "pixel"
		probe = fovea[0][0]->getReceptiveFieldRepresentation(0, 0, 0, fovea[0][0]->modalitiesBottomUp.begin()->second);
		int foveaPixelW = probe.width() * fovea.size(); //We display the potential layers next to each other
		int foveaPixelH = probe.height() * fovea[0].size();

		//Prepare the full image
		totalW = (foveaPixelW + border) * v2->W() * v2->L();
		totalH = (foveaPixelH + border) * v2->H();

		fullRF = cvCreateImage(cvSize(totalW, totalH), 8, 3);

		//Go through all the neurons of lgnUsed
		for (int v2x = 0; v2x < v2->W(); v2x++)
		{
			for (int v2y = 0; v2y < v2->H(); v2y++)
			{
				for (int v2z = 0; v2z < v2->L(); v2z++)
				{
					//stack.connectModalities("/gaze/v1", "/v2/gaze");
					//stack.connectModalities("/v1Retina/out", "/v2/v1Retina");
					//stack.connectModalities("/v1Fovea/out", "/v2/v1Fovea");

					vector<double> rfv2 = v2->getReceptiveFieldWeights(v2x, v2y, v2z, v2->modalitiesBottomUp["/v2/v1Fovea"]);
					int bmuX, bmuY, bmuZ;
					v1Fovea->getBestMatchingUnit(rfv2, v1Fovea->modalitiesTopDown["/v1Fovea/out"], bmuX, bmuY, bmuZ);
					IplImage* px = getV1RFSingleNeuron(v1Fovea, fovea, bmuX, bmuY, bmuZ);;

					cvSetImageROI(fullRF, cvRect((v2x + v2z)*(border + px->width), v2y*(border + px->height), px->width, px->height));
					cvCopyImage(px, fullRF);
					cvResetImageROI(fullRF);
					cvReleaseImage(&px);
				}
			}
		}
		std::stringstream fileName2;
		fileName2 << "rfImages/" << v2->getName() << "_fovea__" << autoIncrementCnt << ".jpg";
		cout << "Saving receptive fields of " << fileName2.str() << " --> " << cvSaveImage(fileName2.str().c_str(), fullRF) << endl;
		cvReleaseImage(&fullRF);
	}

public:

	void run()
	{
		saveRetinaLikeRF(retina);
		saveRetinaLikeRF(fovea);
		saveV1RF(v1Retina, retina);
		saveV1RF(v1Fovea, fovea);
		saveV2RF();
		autoIncrementCnt++;
	}

	StackRpcWrapper(cvz::core::CvzStack* _stack, int retinaW, int retinaH, int foveaW, int foveaH) :yarp::os::RateThread(15*60*1000)
	{
		autoIncrementCnt = 0;
		stack = _stack;

		//Parse the stack
		retina.resize(retinaW);
		for (int x = 0; x < retinaW; x++)
		{
			retina[x].resize(retinaH);
			for (int y = 0; y < retinaH; y++)
			{

				std::stringstream ssName;
				ssName << "retina/" << x << "_" << y;
				retina[x][y] = (cvz::core::CvzMMCM*) stack->nodes[ssName.str()]->cvz;
			}
		}

		fovea.resize(foveaW);
		for (int x = 0; x < foveaW; x++)
		{
			fovea[x].resize(foveaH);
			for (int y = 0; y < foveaH; y++)
			{

				std::stringstream ssName;
				ssName << "fovea/" << x << "_" << y;
				fovea[x][y] = (cvz::core::CvzMMCM*) stack->nodes[ssName.str()]->cvz;
			}
		}

		v1Retina = (cvz::core::CvzMMCM*) stack->nodes["v1Retina"]->cvz;
		v1Fovea = (cvz::core::CvzMMCM*) stack->nodes["v1Fovea"]->cvz;
		//v1 = (cvz::core::CvzMMCM*) stack->nodes["v1"]->cvz;
		v2 = (cvz::core::CvzMMCM*) stack->nodes["v2"]->cvz;
		gaze = (cvz::core::CvzMMCM*) stack->nodes["gaze"]->cvz;
	}

	virtual void onRead(Bottle& b)
	{
		if (stack == NULL)
			return;

		cout << "StackWrapper received : " << b.toString() << endl;

		std::string keyWord = b.get(0).asString();
		if (keyWord == "pause")
		{
			cout << "Pausing the stack." << endl;
			stack->pause();
			return;
		}

		if (keyWord == "start")
		{
			cout << "Starting the stack." << endl;
			stack->start();
			return;
		}

		if (keyWord == "resume")
		{
			cout << "Resuming the stack." << endl;
			stack->resume();
			return;
		}

		if (keyWord == "set")
		{
			std::string keyword2 = b.get(1).asString();
			std::string area = b.get(2).asString();
			double newValue = b.get(3).asDouble();
			if (area == "retina")
			{
				for (int i = 0; i < retina.size(); i++)
				{
					for (int j = 0; j < retina[i].size(); j++)
					{
						if (keyword2 == "learning")
							retina[i][j]->setLearningRate(newValue);
						if (keyword2 == "sigma")
							retina[i][j]->setSigma(newValue);
					}
				}
				return;
			}
			
			if (area == "fovea")
			{
				for (int i = 0; i < fovea.size(); i++)
				{
					for (int j = 0; j < fovea[i].size(); j++)
					{
						if (keyword2 == "learning")
							fovea[i][j]->setLearningRate(newValue);
						if (keyword2 == "sigma")
							fovea[i][j]->setSigma(newValue);
					}
				}
				return;
			}
			
			if (stack->nodes.find(area) == stack->nodes.end())
			{
				cout << "Asked map doesn't exist" << endl;
				return;
			}
			cvz::core::CvzMMCM* targetMap = (cvz::core::CvzMMCM*) stack->nodes[area]->cvz;
		}

		if (keyWord == "save")
		{
			cout << "Pausing the stack." << endl;
			stack->pause();

			cout << "Saving the stack weights." << endl;
			for (int i = 0; i < retina.size(); i++)
			{
				for (int j = 0; j < retina[i].size(); j++)
				{
					retina[i][j]->saveWeightsToFile(retina[i][j]->getName());
				}
			}			
			for (int i = 0; i < fovea.size(); i++)
			{
				for (int j = 0; j < fovea[i].size(); j++)
				{
					fovea[i][j]->saveWeightsToFile(fovea[i][j]->getName());
				}
			}
			v1Fovea->saveWeightsToFile("v1Fovea");
			v1Retina->saveWeightsToFile("v1Retina");
			v2->saveWeightsToFile("v1Fovea");

			cout << "Resuming the stack." << endl;
			stack->resume();

		}

		if (keyWord == "plotRF")
		{
			cout << "About to plot receptive fields..." << endl;

			cout << "Pausing the stack." << endl;
			stack->pause();

			//Proceed v2
			saveRetinaLikeRF(retina);
			saveRetinaLikeRF(fovea);
			saveV1RF(v1Retina, retina);
			saveV1RF(v1Fovea, fovea);
			saveV2RF();
			cout << "Resuming the stack." << endl;
			stack->resume();

			return;
		}
	}
};

void configureV1Retina(cvz::core::CvzStack* stack, int retinaX, int retinaY)
{
	stringstream configV1Retina;

	//V1 Retina
	configV1Retina
		<< "type" << '\t' << cvz::core::TYPE_MMCM << endl
		<< "name" << '\t' << "v1Retina" << endl
		<< "width" << '\t' << V1_RETINA_W << endl
		<< "height" << '\t' << V1_RETINA_H << endl
		<< "layers" << '\t' << V1_RETINA_L << endl
		<< "sigmaFactor" << '\t' << V1_RETINA_SIGMA_FACTOR << endl
		<< "learningRate" << '\t' << V1_RETINA_LEARNING << endl << endl;

	int modalitiesCount = 0;
	for (int x = 0; x < retinaX; x++)
	{
		for (int y = 0; y < retinaY; y++)
		{
			configV1Retina
				<< "[modality_" << modalitiesCount << "]" << endl
				<< "name" << '\t' << "in_" << x << "_" << y << endl
				<< "type" << '\t' << "yarpVector" << endl
				<< "size" << '\t' << 3 << endl << endl;
			modalitiesCount++;
		}
	}

	configV1Retina
		<< "[modality_" << modalitiesCount << "]" << endl
		<< "name" << '\t' << "out" << endl
		<< "type" << '\t' << "yarpVector" << endl
		<< "isTopDown" << '\t' << "yarpVector" << endl
		<< "size" << '\t' << 3 << endl << endl;

	Property propV1;
	propV1.fromConfig(configV1Retina.str().c_str());
	stack->addCvzFromProperty(propV1, "v1Retina");

	//Instantiate the retina maps
	for (int x = 0; x < retinaX; x++)
	{
		for (int y = 0; y < retinaY; y++)
		{
			std::stringstream ssName;
			ssName << "retina/" << x << "_" << y;
			stack->addCvzFromConfigFile("retinaCell.ini", ssName.str().c_str());

			//Connect camera input
			std::string inputModName = "/";
			inputModName += ssName.str().c_str();
			inputModName += "/retina";
			std::stringstream inputName;
			inputName << "/imageSplitter/split/" << x << "_" << y << ":o";
			stack->connectExternalInput(inputName.str(), inputModName);

			//Connect topdown
			std::string upModalityName = "/";
			upModalityName += ssName.str().c_str();
			upModalityName += "/v1";
			std::stringstream ssV1Name;
			ssV1Name << "/v1Retina/in_" << x << "_" << y;
			std::string downModalityName = ssV1Name.str();
			stack->connectModalities(upModalityName, downModalityName);
		}
	}
}

/*******************************************************************************/
void configureV1Fovea(cvz::core::CvzStack* stack, int foveaX, int foveaY)
{
	stringstream configV1Retina;

	//V1 Retina
	configV1Retina
		<< "type" << '\t' << cvz::core::TYPE_MMCM << endl
		<< "name" << '\t' << "v1Fovea" << endl
		<< "width" << '\t' << V1_FOVEA_W << endl
		<< "height" << '\t' << V1_FOVEA_H << endl
		<< "layers" << '\t' << V1_FOVEA_L << endl
		<< "sigmaFactor" << '\t' << V1_FOVEA_SIGMA_FACTOR << endl
		<< "learningRate" << '\t' << V1_FOVEA_LEARNING << endl << endl;

	int modalitiesCount = 0;
	for (int x = 0; x < foveaX; x++)
	{
		for (int y = 0; y < foveaY; y++)
		{
			configV1Retina
				<< "[modality_" << modalitiesCount << "]" << endl
				<< "name" << '\t' << "in_" << x << "_" << y << endl
				<< "type" << '\t' << "yarpVector" << endl
				<< "size" << '\t' << 3 << endl << endl;
			modalitiesCount++;
		}
	}

	configV1Retina
		<< "[modality_" << modalitiesCount << "]" << endl
		<< "name" << '\t' << "out"<< endl
		<< "type" << '\t' << "yarpVector" << endl
		<< "isTopDown" << '\t' << "yarpVector" << endl
		<< "size" << '\t' << 3 << endl << endl;

	Property propV1;
	propV1.fromConfig(configV1Retina.str().c_str());
	stack->addCvzFromProperty(propV1, "v1Fovea");

	//Instantiate the retina maps
	for (int x = 0; x < foveaX; x++)
	{
		for (int y = 0; y < foveaY; y++)
		{
			std::stringstream ssName;
			ssName << "fovea/" << x << "_" << y;
			stack->addCvzFromConfigFile("foveaCell.ini", ssName.str().c_str());

			//Connect camera input
			std::string inputModName = "/";
			inputModName += ssName.str().c_str();
			inputModName += "/retina";
			std::stringstream inputName;
			inputName << "/imageSplitter/fovea/split/" << x << "_" << y << ":o";
			stack->connectExternalInput(inputName.str(), inputModName);

			//Connect topdown
			std::string upModalityName = "/";
			upModalityName += ssName.str().c_str();
			upModalityName += "/v1";
			std::stringstream ssV1Name;
			ssV1Name << "/v1Fovea/in_" << x << "_" << y;
			std::string downModalityName = ssV1Name.str();
			stack->connectModalities(upModalityName, downModalityName);
		}
	}
}

int main(int argc, char * argv[])
{
	Network yarp;
	bool b1, b2;
	cout << "Server name is : " << yarp.getNameServerName() << endl;

	if (!Network::checkNetwork())
	{
		cout << "yarp network is not available!" << endl;
		return 0;
	}

	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultContext("visualSystem");
	rf.setDefaultConfigFile("visualSystem.ini"); //overridden by --from parameter
	rf.configure(argc, argv);

	//Get the parameters of the model
	cvz::core::CvzStack stack;
	int retinaX = rf.check("retinaW",Value(3)).asInt();
	int retinaY = rf.check("retinaH", Value(3)).asInt();
	int foveaX = rf.check("foveaW", Value(3)).asInt();
	int foveaY = rf.check("foveaH", Value(3)).asInt();

	/*************************************************/
	//Add V1
	configureV1Retina(&stack, retinaX, retinaY);
	configureV1Fovea(&stack, foveaX, foveaY);

	/*************************************************/
	//Add v2
	stringstream configV2;
	configV2
		<< "type" << '\t' << cvz::core::TYPE_MMCM << endl
		<< "name" << '\t' << "v2" << endl
		<< "width" << '\t' << V2_W << endl
		<< "height" << '\t' << V2_H << endl
		<< "layers" << '\t' << V2_L << endl
		<< "sigmaFactor" << '\t' << V2_SIGMA_FACTOR << endl
		<< "learningRate" << '\t' << V2_LEARNING << endl << endl;

	//Add the proprioception
	configV2
		<< "[modality_0]" << endl
		<< "name" << '\t' << "gaze" << endl
		<< "type" << '\t' << "yarpVector" << endl
		<< "size" << '\t' << 3 << endl << endl;

	//Add the v1Retina
	configV2
		<< "[modality_1]" << endl
		<< "name" << '\t' << "v1Retina" << endl
		<< "type" << '\t' << "yarpVector" << endl
		<< "size" << '\t' << 3 << endl << endl;

	configV2
		<< "[modality_2]" << endl
		<< "name" << '\t' << "v1Fovea" << endl
		<< "type" << '\t' << "yarpVector" << endl
		<< "size" << '\t' << 3 << endl << endl;

	Property propV2;
	propV2.fromConfig(configV2.str().c_str());
	stack.addCvzFromProperty(propV2, "v2");

	//Add the head proprioception
	stack.addCvzFromConfigFile(std::string("icub_head.ini"), "gaze");

	//Connect the head to v2 -- I know this is non plausible
	stack.connectModalities("/gaze/v1", "/v2/gaze");
	stack.connectModalities("/v1Retina/out", "/v2/v1Retina");
	stack.connectModalities("/v1Fovea/out", "/v2/v1Fovea");

	//Make sure the graph is completed and all the connections are established
	stack.connectModalities();

	stack.configure(rf);
	
	StackRpcWrapper* rpc = new StackRpcWrapper(&stack, retinaX, retinaY, foveaX, foveaY);
	rpc->open("/visualSystem/rpc");
	rpc->useCallback();
	rpc->start();

	stack.runModule();
	rpc->stop();
	return 0;
}