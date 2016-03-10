#include <evtStory.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;
using namespace std;


bool evtStory::isIn(vector<string> vec, string str){
    bool out = false;

    for (auto itS = vec.begin(); itS != vec.end(); itS++){
        out |= (*itS == str);
    }
    return out;
}



void evtStory::removeUnderscoreString(string &input){
    for (std::string::iterator it = input.begin(); it != input.end(); ++it) {
        if (*it == '_') {
            *it = ' ';
        }
    }
}


void evtStory::addUnderscoreString(string &input){
    for (std::string::iterator it = input.begin(); it != input.end(); ++it) {
        if (*it == ' ') {
            *it = '_';
        }
    }
}


void evtStory::addUnderscore(){
    addUnderscoreString(activity_name);
    addUnderscoreString(activity_type);
    addUnderscoreString(predicate);
    addUnderscoreString(agent);
    addUnderscoreString(object);
    addUnderscoreString(recipient);
}

void evtStory::removeUnderscore(){
    removeUnderscoreString(activity_name);
    removeUnderscoreString(activity_type);
    removeUnderscoreString(predicate);
    removeUnderscoreString(agent);
    removeUnderscoreString(object);
    removeUnderscoreString(recipient);
}