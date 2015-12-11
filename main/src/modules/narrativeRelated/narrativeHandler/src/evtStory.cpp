#include <evtStory.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;
using namespace std;

evtStory::evtStory(int _instance, Bottle bActivity, Bottle bArguments, Bottle _bRelations){

    instance = _instance;
    bRelations = _bRelations;

    activity_type = (bActivity.get(0).asList())->get(1).toString();
    activity_name = (bActivity.get(0).asList())->get(0).toString();

    if (activity_type == ""){
        yWarning() << " in narrativeHandler::evtStory::evtStory no activity_type.";
    }
    if (activity_name == ""){
        yWarning() << " in narrativeHandler::evtStory::evtStory no activity_type.";
    }

    begin = (bActivity.get(0).asList())->get(2).toString() == "t";

    for (int kk = 0; kk < bArguments.size(); kk++){
        Bottle bTemp = *bArguments.get(kk).asList();

        if (bTemp.get(1).toString() == "predicate"
            || bTemp.get(1).toString() == "action"
            || bTemp.get(1).toString() == "action2"
            || bTemp.get(1).toString() == "action3"
            || bTemp.get(1).toString() == "verb1"
            || bTemp.get(1).toString() == "verb2"
            || bTemp.get(1).toString() == "verb3") predicate = bTemp.get(0).toString();
        else if (bTemp.get(1).toString() == "agent"
            || bTemp.get(1).toString() == "agent1"
            || bTemp.get(1).toString() == "agent2")     agent = bTemp.get(0).toString();
        else if (bTemp.get(1).toString() == "object"
            || bTemp.get(1).toString() == "object1"
            || bTemp.get(1).toString() == "object2")    object = bTemp.get(0).toString();
        else if (bTemp.get(1).toString() == "recipient"
            || bTemp.get(1).toString() == "spatial"
            || bTemp.get(1).toString() == "spatial1"
            || bTemp.get(1).toString() == "spatial2") recipient = bTemp.get(0).toString();
        else {
            pair<string, string> ptemp(bTemp.get(1).toString(), bTemp.get(0).toString());
            vArgument.push_back(ptemp);
        }

    }

    if (activity_type == "action"){
        if (predicate == "none" || predicate == ""){
            predicate = activity_name;
        }
    }
}

