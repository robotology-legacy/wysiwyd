#include <abmReasoningFunction.h>

using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;


class contextualKnowledge
{   
public:

    string                              sName;
    string                              sArgument;
    string                              sDependance;
    vector< pair <bool , bool> >        vObjectPresent;
    pair<double , double >              PercentPresence;                        //  percent of presence before and after action
    map<string , vector < pair<bool, bool> > >  mIntersectLocation;             // map : key = location, vector is at location or not before and after. 
    map<string ,  pair<double, double > >       mPercentIntersectLocation;      // map : key = location, vector is at percent of presence or not before and after.
    map<string, int>                    mAgentRelated;                          // map : key = name of the agent, number of time the agent did the action
    map<string, double>                 mPercentAgentRelated;                   // idem that previous in percent

    map<string , vector <pair <bool, bool> > >  mObjectFromTo;                  // map : key = obejct, vector is if an objectwas at the same location that the object of focus before and after
    map<string , pair <double, double> >    mPercentObjectFromTo;               // map : key = obejct, idem, with percent

    void                            updatePresence();
    void                            updateIntersect();
    void                            updateAgentRelated();

    void                            checkConditions();
    void                            checkConsequences();

    void                            presenceConsequence();
    void                            presenceConditions();
};

