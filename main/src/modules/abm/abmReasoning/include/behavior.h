#ifndef _BEHAVIOR_H_
#define _BEHAVIOR_H_

#include <abmReasoningFunction.h>

class behavior
{
public:
    std::string                                          sName;
    std::string                                          sArgument;
    std::vector < std::vector< std::pair <std::string, double> > >      vEffect;
    yarp::os::Bottle                      getConsequence();
    yarp::os::Bottle                      getConsequence(int last);

};

#endif