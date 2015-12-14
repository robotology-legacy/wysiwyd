#include <evtStory.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;
using namespace std;

vector<string> evtStory::initialize(int _instance, Bottle bActivity, Bottle bArguments, Bottle _bRelations){
    instance = _instance;
    bRelations = _bRelations;
    vector<string>   vOCW;

    vector<string> vPredicate{ "predicate", "action", "action1", "action2", "action3", "verb", "verb1", "verb2", "verb3" };
    vector<string> vAgent{ "agent", "agent1", "agent2", "agent3" };
    vector<string> vObject{ "object", "object1", "object2", "object2" };
    vector<string> vRecipient{ "recipient", "spatial", "spatial1", "spatial2", "spatial3" };

    vector<string>  vNoPAOR{ "subsystem", "provider", "vector", "sentence" };

    activity_type = (bActivity.get(0).asList())->get(1).toString();
    activity_name = (bActivity.get(0).asList())->get(0).toString();

    if (activity_type == ""){
        yWarning() << " in narrativeHandler::evtStory::evtStory no activity_type.";
    }
    if (activity_name == ""){
        yWarning() << " in narrativeHandler::evtStory::evtStory no activity_type.";
    }

    begin = (bActivity.get(0).asList())->get(2).toString() == "t";

    bool bComeFromProvider = false;

    for (int kk = 0; kk < bArguments.size(); kk++){
        if (bArguments.get(kk).isList()) {
            Bottle bTemp = *bArguments.get(kk).asList();

            if (isIn(vPredicate, bTemp.get(1).toString())) predicate = bTemp.get(0).toString();
            else if (isIn(vAgent, bTemp.get(1).toString())) agent = bTemp.get(0).toString();
            else if (isIn(vObject, bTemp.get(1).toString()))    object = bTemp.get(0).toString();
            else if (isIn(vRecipient, bTemp.get(1).toString())) recipient = bTemp.get(0).toString();
            else {
                pair<string, string> ptemp(bTemp.get(1).toString(), bTemp.get(0).toString());
                vArgument.push_back(ptemp);
            }

            if (!isIn(vNoPAOR, bTemp.get(1).toString())) vOCW.push_back(bTemp.get(0).asString());
        }
    }

    if (activity_type == "action"){
        if (predicate == "none" || predicate == ""){
            predicate = activity_name;
            vOCW.push_back(predicate);
        }
    }

    return vOCW;

}



bool evtStory::isIn(vector<string> vec, string str){
    bool out = false;

    for (auto itS = vec.begin(); itS != vec.end(); itS++){
        out |= (*itS == str);
    }
    return out;
}