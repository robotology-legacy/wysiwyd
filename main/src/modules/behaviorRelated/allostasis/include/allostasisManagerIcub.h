#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/SVD.h>
#include <map>
#include <vector>
#include "../homeostasis/homeostasisManagerIcub.h"


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


#include "homeostasisManager.h"






class allostaticModule: public homeostaticModule
{
private:

    

public:

	bool updateAllostatic();

	//RPC & scenarios
	bool respond(const Bottle& cmd, Bottle& reply);
};
