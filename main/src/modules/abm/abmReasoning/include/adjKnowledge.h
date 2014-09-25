#include <abmReasoningFunction.h>

using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;


class adjKnowledge
{
public:

    adjKnowledge();

    string      sLabel;
    string      sTag;

    bool fTimingInfluence;
    bool fAbsolutInfluence;
    bool fDeltaInfluence;
    bool fFromInfluence;

    // TIMING

    vector<double>      vdGnlTiming;             // global timing of any action with this adjective
    vector<double>      vdNoGnlTiming;           // global timing of any action without this adjective

    map<string, pair< vector<double>, vector<double> > >     mActionTiming;     // map with timing of adj + act , adj + no_act ; key is action name;


    // SPATIAL

    //absolut

    vector<pair<double, double> >       vdGnlXY;          // absolute final location of any action with this adjective
    vector<pair<double, double> >       vdNoGnlXY;        // absolute final location of any action without this adjective

    map<string, vector< pair<double, double > > >     mActionAbsolut;     // map with absolute final location of adj + act ; key is action name;

    // DELTA

    vector<pair<double, double> >       vdGnlDelta;          // Relative displacement of any action with this adjective
    vector<pair<double, double> >       vdNoGnlDelta;        // Relative displacement of any action without this adjective

    map<string, vector<pair<double, double> >  >     mActionDelta;     // map with Relative displacement of adj + act ; key is action name;



    // FUNCTIONS

    void    determineInfluence();
    void    determineTimingInfluence();
    void    determineSpatialInfluence();

    void    addInteraction(Bottle bInput);
    void    addOtherInteraction(Bottle bInput);

    

    pair<double, double>    coordRelative(double Xo, double Yo, double Xh, double Yh);      // return the relatve coordinates of an object from an other agent


    pair <int, double>      distFromMove(pair<double, double> XY, pair<double, double> MOVE);
    vector<double>          determineAbsolut();


};