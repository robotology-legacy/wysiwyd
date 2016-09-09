#include "speech.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

void Speech::configure() {
    ;
}

void Speech::run(Bottle args) {
    yInfo() << "Speech::run";
    // Bottle *objects = sensation.get(1).asList();
    // Might want to include gestures like looking at partner and waving a hand

    iCub->say(args.get(0).asString());

}

