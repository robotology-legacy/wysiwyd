#ifndef _TIMEKNOWLEDGE_H_
#define _TIMEKNOWLEDGE_H_

#include <abmReasoningFunction.h>


class timeKnowledge
{
public:

    std::string                      sTemporal;
    std::pair<double, double>        coordFromString(std::string);
    void                        fromBottle(yarp::os::Bottle bInput);
    void                        addKnowledge(yarp::os::Bottle bInput);
    std::string                      sArgument;
    int                         iSize;

    struct tm                   timeDiff(struct tm tm1, struct tm tm2);

    double                      T1inferiorT2percent();      // return the percentage of timeArg1 inferior to timeArg2.

    std::vector<struct tm>           timeArg1;
    std::vector<struct tm>           timeArg2;
};

#endif