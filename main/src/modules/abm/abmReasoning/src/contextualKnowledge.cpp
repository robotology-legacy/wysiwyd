#include <contextualKnowledge.h>


using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;


void contextualKnowledge::checkConditions()
{
    presenceConditions();
}


void contextualKnowledge::checkConsequences()
{
    presenceConsequence();
}


void contextualKnowledge::presenceConditions()
{

}


void contextualKnowledge::presenceConsequence()
{

}


void contextualKnowledge::updatePresence()
{

    int Before = 0,
        After = 0;

    for (vector<pair<bool, bool> >::iterator it_p = vObjectPresent.begin(); it_p != vObjectPresent.end(); it_p++)
    {
        if (it_p->first)
            Before++;
        if (it_p->second)
            After++;
    }

    PercentPresence.first = Before / (1. * vObjectPresent.size());
    PercentPresence.second = After / (1. * vObjectPresent.size());

    for (map<string, vector<pair<bool, bool> > >::iterator itMAP = mIntersectLocation.begin(); itMAP != mIntersectLocation.end(); itMAP++)
    {
        Before = 0;
        After = 0;

        for (vector<pair<bool, bool> >::iterator it_p = itMAP->second.begin(); it_p != itMAP->second.end(); it_p++)
        {
            if (it_p->first)
                Before++;
            if (it_p->second)
                After++;
        }

        mPercentIntersectLocation[itMAP->first].first = Before / (1. * itMAP->second.size());
        mPercentIntersectLocation[itMAP->first].second = After / (1. * itMAP->second.size());
    }

    for (map<string, vector<pair<bool, bool> > >::iterator itMAP = mObjectFromTo.begin(); itMAP != mObjectFromTo.end(); itMAP++)
    {
        Before = 0;
        After = 0;

        for (vector<pair<bool, bool> >::iterator it_p = itMAP->second.begin(); it_p != itMAP->second.end(); it_p++)
        {
            if (it_p->first)
                Before++;
            if (it_p->second)
                After++;
        }

        mPercentObjectFromTo[itMAP->first].first = Before / (1. * itMAP->second.size());
        mPercentObjectFromTo[itMAP->first].second = After / (1. * itMAP->second.size());
    }
}


void contextualKnowledge::updateIntersect()
{

    int Before = 0,
        After = 0;

    for (vector<pair<bool, bool> >::iterator it_p = vObjectPresent.begin(); it_p != vObjectPresent.end(); it_p++)
    {
        if (it_p->first)
            Before++;
        if (it_p->second)
            After++;
    }

    PercentPresence.first = Before / (1. * vObjectPresent.size());
    PercentPresence.second = After / (1. * vObjectPresent.size());
}


void contextualKnowledge::updateAgentRelated()
{

    int iTotal = 0;
    mPercentAgentRelated.clear();

    for (map<string, int>::iterator itMap = mAgentRelated.begin(); itMap != mAgentRelated.end(); itMap++)
    {
        iTotal += itMap->second;
    }

    for (map<string, int>::iterator itMap = mAgentRelated.begin(); itMap != mAgentRelated.end(); itMap++)
    {
        mPercentAgentRelated[itMap->first] = itMap->second / (iTotal*1.);
    }
}