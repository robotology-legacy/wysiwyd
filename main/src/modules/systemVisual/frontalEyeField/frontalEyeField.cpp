// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2014 WYSIWYD Consortium
 * Authors: Stéphane Lallée
 * email:   stephane.lallee@gmail.com
 * website: http://efaa.upf.edu/ 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * $EFAA_ROOT/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "frontalEyeField.h"


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/************************************************************************/
bool FrontalEyeField::configure(yarp::os::ResourceFinder &rf) {

    //Get conf parameters
    moduleName = rf.check("name",Value("frontalEyeField")).asString();
    setName(moduleName.c_str());
    tau = rf.check("tau", Value(2.0)).asDouble();
    retinaW = rf.check("retinaW", Value(3)).asInt();
    retinaH = rf.check("retinaH", Value(3)).asInt();
    string nameSourcePrefix = rf.check("nameSplitterPrefix", Value("/v1Retina/in_")).asString();
    string nameSourceSuffix = rf.check("nameSplitterSuffix", Value("/error:o")).asString();
    cameraUsed = (rf.check("camera", Value("left")).asString() == "right");

    //string nameSourcePrefix = rf.check("nameSplitterPrefix", Value("/retina/")).asString();
    //string nameSourceSuffix = rf.check("nameSplitterSuffix", Value("/retina/error:o")).asString();


    //----------------------------//
    //Configure the egocentric error cvz
    stringstream configEgocentricError;
    configEgocentricError
        << "type" << '\t' << cvz::core::TYPE_MMCM << endl
        << "name" << '\t' << "egocentricError" << endl
        << "width" << '\t' << 10 << endl
        << "height" << '\t' << 10 << endl
        << "layers" << '\t' << 5 << endl
        << "sigmaFactor" << '\t' << 0.75 << endl
        << "learningRate" << '\t' << 0.01 << endl << endl;

    //Add the proprioception
    configEgocentricError
        << "[modality_0]" << endl
        << "name" << '\t' << "fixationPoint" << endl
        << "type" << '\t' << "yarpVector" << endl
        << "size" << '\t' << 3 << endl
        << "minBounds" << '\t' << "(-2.0 -1.0 -1.0)" << endl
        << "maxBounds" << '\t' << "(-1.0 1.0 1.0)" << endl
        << "autoconnect" << '\t' << "/iKinGazeCtrl/x:o" << endl << endl;

    //Add the v1Retina
    configEgocentricError
        << "[modality_1]" << endl
        << "name" << '\t' << "v2Error" << endl
        << "type" << '\t' << "yarpVector" << endl
        << "size" << '\t' << 1 << endl
        //<< "isBlocking" << endl
        << "autoconnect" << '\t' << "/v2/v1Retina/error:o"<<endl;

    Property prop;
    prop.fromConfig(configEgocentricError.str().c_str());
    mmcmErrorPrediction = new cvz::core::ThreadedCvz(prop, 100);
    mmcmErrorPrediction->start();
    //cvz::core::CvzBuilder::allocate(&mmcmErrorPrediction, cvz::core::TYPE_MMCM);
    //mmcmErrorPrediction->configure(prop);

    //----------------------------//
    //Open the gaze controller
    gazePortName = "/";
    gazePortName += getName() + "/gaze";
    Property option;
    option.put("device", "gazecontrollerclient");
    option.put("remote", "/iKinGazeCtrl");
    option.put("local", gazePortName.c_str());

    igaze = NULL;
    if (clientGazeCtrl.open(option)) {
        clientGazeCtrl.view(igaze);
    }
    else
    {
        cout << "Invalid gaze polydriver" << endl;
        return false;
    }

    igaze->storeContext(&store_context_id);

    double neckTrajTime = rf.check("neckTrajTime",
        Value(0.75)).asDouble();
    igaze->setNeckTrajTime(neckTrajTime);
    igaze->bindNeckPitch(-15.0, 10.0);
    igaze->bindNeckRoll(-10.0, 10.0);
    igaze->bindNeckYaw(-20.0, 20.0);
    igaze->blockEyes(0.0);

    //Open the retina input ports
    retinaInput.resize(retinaW);
    errorMap.resize(retinaW);
    for (int x = 0; x < retinaW; x++)
    {
        retinaInput[x].resize(retinaH);
        errorMap[x].resize(retinaH);
        for (int y = 0; y < retinaH; y++)
        {
            stringstream ss;
            ss << "/" << moduleName << "/error/" << x << "_" << y << ":i";
            retinaInput[x][y] = new BufferedPort<Bottle>();
            retinaInput[x][y]->open(ss.str().c_str());
        }
    }
    //Wait for the connection
    connectErrorInput(nameSourcePrefix, nameSourceSuffix);
    timeNextSaccade = Time::now() + tau;
    return true;
}

/************************************************************************/
void FrontalEyeField::connectErrorInput(std::string splitterPrefix, std::string splitterSuffix)
{
    for (int x = 0; x < retinaW; x++)
    {
        for (int y = 0; y < retinaH; y++)
        {
            stringstream ssSource;
            ssSource << splitterPrefix << x << "_" << y << splitterSuffix;
            int attempt = 0;
            while (attempt < 5 && !Network::connect(ssSource.str().c_str(), retinaInput[x][y]->getName().c_str()))
            {
                std::cout << "Attempt to automated connection failed for : " << std::endl
                    << "\t" << ssSource.str().c_str() << std::endl
                    << "\t" << retinaInput[x][y]->getName().c_str() << std::endl;
                Time::delay(0.5);
                //attempt++;
            }

        }
    }
}
/************************************************************************/
bool FrontalEyeField::interruptModule() {
    for (int x = 0; x < retinaW; x++)
    {
        retinaInput[x].resize(retinaH);
        for (int y = 0; y < retinaH; y++)
        {
            retinaInput[x][y]->interrupt();
        }
    }
    return true;
}

/************************************************************************/
bool FrontalEyeField::close() {

    igaze->restoreContext(store_context_id);
    if (clientGazeCtrl.isValid())
        clientGazeCtrl.close();

    for (int x = 0; x < retinaW; x++)
    {
        retinaInput[x].resize(retinaH);
        for (int y = 0; y < retinaH; y++)
        {
            retinaInput[x][y]->close();
        }
    }
    return true;
}


/***************************************************************************/
bool FrontalEyeField::updateModule() 
{
    //Update the error map activity, mean error and maximum error region
    int xMax = 0;
    int yMax = 0;
    double meanError = 0.0;
    //std::cout << "Receiving error:" << endl;
    for (int x = 0; x < retinaW; x++)
    {
        for (int y = 0; y < retinaH; y++)
        {
            Bottle* bError = retinaInput[x][y]->read(true);
            if (bError)
            {
                errorMap[x][y].update(bError->get(0).asDouble());
                meanError += errorMap[x][y].x;
                if (errorMap[x][y].x>errorMap[xMax][yMax].x)
                {
                    xMax = x;
                    yMax = y;
                }
            }
        }
        //std::cout << endl;
    }
    meanError /= (retinaW*retinaH);
    //cout << "Mean error : " << meanError << "\t Max Error : " << errorMap[xMax][yMax].x<<" in \t" << xMax << " " << yMax << endl;

    double SACCADE_TRESHOLD = 0.0001;
    //Bottom up saccades, triggered from high retina level error
    if (Time::now()>timeNextSaccade && errorMap[xMax][yMax].x>SACCADE_TRESHOLD/*meanError>SACCADE_TRESHOLD && errorMap[xMax][yMax].x > meanError + 0.1*meanError*/)
    {
        timeNextSaccade = Time::now() + tau;
        //cout << "Triggering a saccade "
        //Reset the whole error map
        resetErrorMap();

        //Send the saccade to x/y
        int caseW = (int)(320.0 / retinaW);
        int caseH = (int)(240.0 / retinaH);
        Vector px(2);
        px[0] = xMax * caseW + caseW/2.0;
        px[1] = yMax * caseH + caseH/2.0;
        std::cout << "Bottom up saccade to : " << px.toString() << endl;
        igaze->lookAtMonoPixel(cameraUsed, px);
    }
    else if (Time::now()>timeNextSaccade)
    {

        timeNextSaccade = Time::now() + tau;
        //Top down saccade, triggered from high prediction of error at v2 level
        mmcmErrorPrediction->suspend();
        mmcmErrorPrediction->cvz->modalitiesInfluence[mmcmErrorPrediction->cvz->modalitiesBottomUp["/egocentricError/v2Error"]] = 1.0;
        mmcmErrorPrediction->cvz->modalitiesInfluence[mmcmErrorPrediction->cvz->modalitiesBottomUp["/egocentricError/fixationPoint"]] = 0.0;
        //Predict the position in space that is associated with the highest error
        vector<double> highError(1, 1.0);
        mmcmErrorPrediction->cvz->modalitiesBottomUp["/egocentricError/v2Error"]->SetValueReal(highError);
        mmcmErrorPrediction->cvz->ComputePrediction();
        vector<double> worsePredictedPosition = mmcmErrorPrediction->cvz->modalitiesBottomUp["/egocentricError/fixationPoint"]->GetValuePrediction();
        worsePredictedPosition = mmcmErrorPrediction->cvz->modalitiesBottomUp["/egocentricError/fixationPoint"]->unscale(worsePredictedPosition);
        //Make a saccade toward this point
        Vector fxPt(3); 
        fxPt[0] = worsePredictedPosition[0];
        fxPt[1] = worsePredictedPosition[1];
        fxPt[2] = worsePredictedPosition[2];
        igaze->lookAtFixationPoint(fxPt);

        mmcmErrorPrediction->cvz->modalitiesInfluence[mmcmErrorPrediction->cvz->modalitiesBottomUp["/egocentricError/v2Error"]] = 1.0;
        mmcmErrorPrediction->cvz->modalitiesInfluence[mmcmErrorPrediction->cvz->modalitiesBottomUp["/egocentricError/fixationPoint"]] = 1.0;
        mmcmErrorPrediction->resume();
        cout << "Top down saccade to " << fxPt.toString(3,3)<< endl;
    }
    return true;
}


/************************************************************************/
void FrontalEyeField::resetErrorMap()
{
    for (int x = 0; x < retinaW; x++)
    {
        for (int y = 0; y < retinaH; y++)
        {
            errorMap[x][y].x = 0.0;
        }
    }
}

/************************************************************************/
double FrontalEyeField::getPeriod() {
    return 0.1;
}

