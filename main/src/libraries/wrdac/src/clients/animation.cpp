#include "wrdac/clients/animation.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace wysiwyd::wrdac;

Animator::Animator(string _moduleName, string _robotName, int _period):RateThread(_period)
{    
    Time::turboBoost();

    this->moduleName = _moduleName;
    this->robotName = _robotName;

    //Define the parts we want to use
    workingLimbs.push_back("head");
    workingLimbs.push_back("left_arm");
    workingLimbs.push_back("right_arm");
    workingLimbs.push_back("torso"); 

    //Create/load the animation libraries
    for(list<string>::iterator it = workingLimbs.begin();it!=workingLimbs.end();it++)
    {
        AnimationLibrary newLib;
        limbsAnimations[*it] = newLib;
    }

    //Run the interfaces
    interfacesRunning = startInterfaces();

    //Initialise other stuff
    currentlyRecorded = NULL;
    currentlyReplayed = NULL;
}
        
bool Animator::startInterfaces()
{
    for(list<string>::iterator limb = workingLimbs.begin(); limb!=workingLimbs.end(); limb++)
    {
        Property options;
        //options.put("robot", "icub");
        options.put("device", "remote_controlboard");
        options.put("remote", ("/"+robotName+"/"+*limb).c_str());
        options.put("local", ("/"+moduleName+"/"+*limb).c_str());

        drivers[*limb]= new PolyDriver(options);
        if (!drivers[*limb]->isValid()) {
            cout<<"Device not available."<<endl;
            return false;
        }

        drivers[*limb]->view(encs[*limb]);
        drivers[*limb]->view(ctrls[*limb]);

        if (encs[*limb] == NULL || ctrls[*limb] == NULL)
        {
            cout<<"Problem while acquiring the views of the left arm"<<endl;
            return false;
        }
    }        
    cout<<"**********************************************************************"<<endl
        <<"Animator running..."<<endl;
            
    return true;
}

bool Animator::startRecording(std::string part, std::string animationName)
{
    //Check if the part exists first
    if (encs.find(part) == encs.end())
        return false;
    int size;
    encs[part]->getAxes(&size);
    limbsAnimations[part][animationName] = new Animation(part,size);
    currentlyRecorded = limbsAnimations[part][animationName];
    return true;
}

bool Animator::startReplaying(std::string part, std::string animationName)
{
    if (limbsAnimations.find(part) == limbsAnimations.end() || limbsAnimations[part].find(animationName) == limbsAnimations[part].end())
        return false;

    currentlyReplayed = limbsAnimations[part][animationName];

    return true;
}

void Animator::stopRecording()
{    
    Animation *workingCopy = currentlyRecorded;
    currentlyRecorded=NULL;    

    //Reset time to 0
    for(list<pair<Vector, double> >::reverse_iterator f = workingCopy->frames.rbegin(); f!= workingCopy->frames.rend(); f++)
    {
        f->second = f->second - workingCopy->frames.front().second;
    }

}
void Animator::stopReplaying(){currentlyReplayed=NULL;startReplayingTime=-1;}

bool Animator::threadInit()
{
    for(list<string>::iterator limb = workingLimbs.begin(); limb!=workingLimbs.end(); limb++)
    {
        if(encs[*limb] == NULL)
            return false;
        if(ctrls[*limb] == NULL)
            return false;
    }
    return true;
}

void Animator::run()
{
    if (interfacesRunning)
    {
        double t = Time::now();
        std::map<std::string, yarp::sig::Vector> newFrames;

        //Capture the current frame
        for(list<string>::iterator limb = workingLimbs.begin(); limb!=workingLimbs.end(); limb++)
        {
            encs[*limb]->getEncoders(newFrames[*limb].data());
        }

        //Record if we are recording
        if (currentlyRecorded!=NULL)
        {
            currentlyRecorded->frames.push_back(pair<Vector,double>(newFrames[currentlyRecorded->partUsed],t));
        }

        //Replay if we are replaying
        if (currentlyReplayed!=NULL)
        {
            if (startReplayingTime == -1)
                startReplayingTime = t;
            //Calculate elapsed time
            double eT = t-startReplayingTime;
            //Find the next frame to be played (we skip frames in case of slow execution)
            list<pair<Vector, double> >::iterator bestChoice = currentlyReplayed->frames.begin();
            for(list<pair<Vector, double> >::iterator f = currentlyReplayed->frames.begin(); f!= currentlyReplayed->frames.end(); f++)
            {
                if (f->second<=eT)
                    bestChoice = f;
                else
                    break;
            }
            playFrame(currentlyReplayed, bestChoice);

            //If last frame
            if (bestChoice->second == currentlyReplayed->frames.back().second)
                stopReplaying();
        }
    }
    else
        cout<<"Interfaces not started. Skipping animator loop."<<endl;
}

bool Animator::playFrame(Animation* anim, list<pair<Vector, double> >::iterator frameIterator)
{
    if (interfacesRunning)
    {
        bool overall = true;
        for(size_t j=0;j<anim->jntsControlled.size();j++)
        {
            if (anim->jntsControlled[j])
                overall &= ctrls[anim->partUsed]->positionMove(j,frameIterator->first[j]);
        }
        return overall;
    }
    else
    {
        cout<<"Interfaces not started. Frame cannot be played."<<endl;
        return false;
    }
}



