#ifndef _ADJKNOWLEDGE_H_
#define _ADJKNOWLEDGE_H_

#include <abmReasoningFunction.h>

class adjKnowledge
{
public:

    adjKnowledge();

    std::string      sLabel;

    bool fTimingInfluence;
    bool fAbsolutInfluence;
    bool fDeltaInfluence;
    bool fFromInfluence;

    double bothtails,
        lefttail,
        righttail;

    // TIMING     TO BE COMPARE WITH OTHER ADJ

    std::vector<double>      vdGnlTiming;             // global timing of any action with this adjective

    std::map<std::string, std::vector<double> >     mActionTiming;     // map with timing of adj + act  ; key is action name;


    // SPATIAL

    //absolut

    std::vector<std::pair<double, double> >       vdGnlXY;          // absolute final location of any action with this adjective

    std::map<std::string, std::vector< std::pair<double, double > > >     mActionAbsolut;     // map with absolute final location of adj + act ; key is action name;

    // DELTA

    std::vector<std::pair<double, double> >       vdGnlDelta;          // Relative displacement of any action with this adjective

    std::map<std::string, std::vector<std::pair<double, double> >  >     mActionDelta;     // map with Relative displacement of adj + act ; key is action name;



    // FUNCTIONS

    void    determineInfluence();
    void    determineTimingInfluence();
    void    determineSpatialInfluence();

    void    addInteraction(yarp::os::Bottle bInput);
    void    addOtherInteraction(yarp::os::Bottle bInput);

    yarp::os::Bottle getEffect(std::string sArgument, bool bPrint = false);

    std::pair<double, double>    coordRelative(double Xo, double Yo, double Xh, double Yh);      // return the relatve coordinates of an object from an other agent


    std::pair <int, double>      distFromMove(std::pair<double, double> XY, std::pair<double, double> MOVE);
    std::vector<double>          determineAbsolut();


};



#endif