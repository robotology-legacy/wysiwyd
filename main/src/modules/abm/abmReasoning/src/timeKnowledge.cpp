#include <timeKnowledge.h>


using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;


/*
* Transform a shared plan in a timeKnowledge
* input format : 
* <string name> <string timeArg1> <string timeArg2>
*/
void timeKnowledge::fromBottle(Bottle bInput)
{
    if (bInput.size() !=3)
    {
        std::cout << "Error in timeKnowledge : fromBottle. Wrong number of inputs (need 3 inputs : <string name> <string timeArg1> <string timeArg2>)" << endl;
        return;
    }
    if ( !(bInput.get(0).isString() && bInput.get(1).isString() && bInput.get(2).isString()) )
    {
        std::cout << "Error in timeKnowledge : fromBottle. Wrong format of inputs (need 3 inputs : <string name> <string timeArg1> <string timeArg2>)" << endl;
        return;
    }

    sTemporal = bInput.get(0).asString();
    struct tm   tmTimeArg1 = abmReasoningFunction::string2Time(bInput.get(1).asString().c_str()),
        tmTimeArg2 = abmReasoningFunction::string2Time(bInput.get(2).asString().c_str()) ;

    timeArg1.push_back(tmTimeArg1);
    timeArg2.push_back(tmTimeArg2);
}

/**
* Add a new entity to the timeKnowledge
* input format : 
* <string name> <string timeArg1> <string timeArg2>
*/
void timeKnowledge::addKnowledge(Bottle bInput)
{
    if (bInput.size() !=3)
    {
        std::cout << "Error in addKnowledge : fromBottle. Wrong number of inputs (need 3 inputs : <string name> <string timeArg1> <string timeArg2>)" << endl;
        return;
    }
    if ( !(bInput.get(0).isString() && bInput.get(1).isString() && bInput.get(2).isString()) )
    {
        std::cout << "Error in addKnowledge : fromBottle. Wrong format of inputs (need 3 inputs : <string name> <string timeArg1> <string timeArg2>)" << endl;
        return;
    }

    struct tm   tmTimeArg1 = abmReasoningFunction::string2Time(bInput.get(1).asString().c_str()),
        tmTimeArg2 = abmReasoningFunction::string2Time(bInput.get(2).asString().c_str()) ;

    timeArg1.push_back(tmTimeArg1);
    timeArg2.push_back(tmTimeArg2);
}


struct tm timeKnowledge::timeDiff(struct tm tm1, struct tm tm2)
{
    //  struct tm diffTime;
    time_t myTime;
    time(&myTime);                  // get unix time
    tm *diffTime = localtime(&myTime);      // conversion in local time
    return *diffTime;
}

/**
* Return the percentage of action where T1 is inferior to T2
*/
double timeKnowledge::T1inferiorT2percent()
{
    if (timeArg1.size() == timeArg2.size())
    {
        iSize = timeArg1.size();
    }
    else
    {
        std::cout << "Error in timeKnowledge::T1inferiorT2percent() | wrong size of timeKnowledge" << endl;
        return 0.5;
    }

    int inferior = 0,
        superior = 0;

    for (int i = 0; i < iSize; i++)
    {
        if (abmReasoningFunction::timeDiff(timeArg1[i], timeArg2[i]))
        {
            inferior++;
        }
        else
        {
            superior++;
        }
    }

    return ((inferior*1.)/(inferior*1.+superior*1.));
}