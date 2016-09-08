#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <map>


#include "behavior.h"

class Greeting: public Behavior
{
public:
    void configure();
    void run(Bottle args=Bottle());

};

