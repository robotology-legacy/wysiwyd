#include <abmReasoningFunction.h>

using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;



class timeKnowledge
{   
public:

    string                      sTemporal;
    pair<double, double>        coordFromString(string);
    void                        fromBottle(Bottle bInput);
    void                        addKnowledge(Bottle bInput);
    string                      sArgument;
    int                         iSize;

    struct tm                   timeDiff(struct tm tm1, struct tm tm2);

    double                      T1inferiorT2percent();      // return the percentage of timeArg1 inferior to timeArg2.

    vector<struct tm>           timeArg1;
    vector<struct tm>           timeArg2;
};

