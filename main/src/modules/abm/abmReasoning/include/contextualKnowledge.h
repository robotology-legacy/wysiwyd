#ifndef _CONTEXTUALKNOWLEDGE_H_
#define _CONTEXTUALKNOWLEDGE_H_

#include <abmReasoningFunction.h>

class contextualKnowledge
{
public:

    std::string                              sName;
    std::string                              sArgument;
    std::string                              sDependance;
    int                                        iOccurence;
    std::vector< std::pair <bool, bool> >        vObjectPresent;
    std::pair<double, double >              PercentPresence;                        //  percent of presence before and after action
    std::map<std::string, std::vector < std::pair<bool, bool> > >  mIntersectLocation;             // map : key = location, std::vector is at location or not before and after. 
    std::map<std::string, std::pair<double, double > >       mPercentIntersectLocation;      // map : key = location, std::vector is at percent of presence or not before and after.
    std::map<std::string, int>                    mAgentRelated;                          // map : key = name of the agent, number of time the agent did the action
    std::map<std::string, double>                 mPercentAgentRelated;                   // idem that previous in percent

//    std::map<std::string, std::vector < std::pair<bool, bool> > >  mIntersectLocation;             // map : key = location, std::vector is at location or not before and after. 
//    std::map<std::string, std::pair<double, double > >       mPercentIntersectLocation;      // map : key = location, std::vector is at percent of presence or not before and after.

    std::vector<std::pair<std::string, int> >              mRelationBefore;
    std::vector<std::pair<std::string, int> >              mRelationAfter;

    std::map<std::string, std::vector <std::pair <bool, bool> > >  mObjectFromTo;                  // map : key = obejct, std::vector is if an objectwas at the same location that the object of focus before and after
    std::map<std::string, std::pair <double, double> >    mPercentObjectFromTo;               // map : key = obejct, idem, with percent

    void                            updatePresence();
    void                            updateIntersect();
    void                            updateAgentRelated();

    void                            checkConditions();
    void                            checkConsequences();

    void                            presenceConsequence();
    void                            presenceConditions();
};

#endif