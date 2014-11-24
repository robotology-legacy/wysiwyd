#ifndef _ADJKNOWLEDGE_H_
#define _ADJKNOWLEDGE_H_

#include <abmReasoningFunction.h>

class advKnowledge
{
public:

    advKnowledge();

    std::string      sLabel;
    std::string      sTag;

    bool fTimingInfluence;
    bool fAbsolutInfluence;
    bool fDeltaInfluence;
    bool fFromInfluence;

    // TIMING

    std::vector<double>      vdGnlTiming;             // global timing of any action with this adjective
    std::vector<double>      vdNoGnlTiming;           // global timing of any action without this adjective

    std::map<std::string, std::pair< std::vector<double>, std::vector<double> > >     mActionTiming;     // map with timing of adj + act , adj + no_act ; key is action name;


    // SPATIAL

    //absolut

    std::vector<std::pair<double, double> >       vdGnlXY;          // absolute final location of any action with this adjective
    std::vector<std::pair<double, double> >       vdNoGnlXY;        // absolute final location of any action without this adjective

    std::map<std::string, std::vector< std::pair<double, double > > >     mActionAbsolut;     // map with absolute final location of adj + act ; key is action name;

    // DELTA

    std::vector<std::pair<double, double> >       vdGnlDelta;          // Relative displacement of any action with this adjective
    std::vector<std::pair<double, double> >       vdNoGnlDelta;        // Relative displacement of any action without this adjective

    std::map<std::string, std::vector<std::pair<double, double> >  >     mActionDelta;     // map with Relative displacement of adj + act ; key is action name;


    // FUNCTIONS

    void    determineInfluence();
    void    determineTimingInfluence();
    void    determineSpatialInfluence();

    void    addInteraction(yarp::os::Bottle bInput);
    void    addOtherInteraction(yarp::os::Bottle bInput);


    std::pair<double, double>    coordRelative(double Xo, double Yo, double Xh, double Yh);      // return the relatve coordinates of an object from an other agent


    std::pair <int, double>      distFromMove(std::pair<double, double> XY, std::pair<double, double> MOVE);
    std::vector<double>          determineAbsolut();


};



#endif