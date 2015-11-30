#ifndef SENSATION
#define SENSATION

#include <string>
#include <yarp/os/all.h>

using namespace std;
using namespace yarp::os;

class Sensation : public BufferedPort<Bottle> 
{
public:

    virtual void configure() = 0;
    virtual void publish() = 0;

// protected:
//     string manager_name;
    
};

#endif