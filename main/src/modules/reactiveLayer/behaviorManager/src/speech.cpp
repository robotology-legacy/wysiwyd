#include "speech.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

void Speech::configure() {
    ;
}

void Speech::run(const Bottle &args) {
    yInfo() << "Speech::run";
    // Bottle *objects = sensation.get(1).asList();
    // Might want to include gestures like looking at partner and waving a hand

    // expects a bottle with the sentence to say in args e.g. args == ("speech example")
    iCub->say(args.get(0).asList()->get(0).asString());

}
