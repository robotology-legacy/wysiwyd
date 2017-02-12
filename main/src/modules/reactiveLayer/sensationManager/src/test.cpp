#include <algorithm>    // std::random_shuffle
#include "test.h"

void TestSensation::configure(yarp::os::ResourceFinder &rf)
{
    on = false;
    in.open("/Sensation/test/in");
    out.open("/Sensation/test/out");
    cout<<"Configuration done."<<endl;

}

void TestSensation::publish()
{

    Bottle *res = in.read(false);
    if (res != NULL) {
        on = (res->get(0).asString() == "ON");
        cout << "TEST is "<< on << endl;
    }

    yarp::os::Bottle &tosend = out.prepare();
    tosend.clear();
    tosend.addInt(int(on));
    out.write();

}

