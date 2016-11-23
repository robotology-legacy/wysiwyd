#include <evtStory.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;
using namespace std;


string evtStory::toString(){
    ostringstream os;
    os << "*************************************" << endl;
    os << "\tinstance:      " << instance << endl;
    os << "\tbegin:         " << begin << endl;
    os << "\tactivityName:  " << activity_name << endl;
    os << "\tactivity_type: " << activity_type << endl;
    os << "\tpredicate:     " << predicate << endl;
    os << "\tagent:         " << agent << endl;
    os << "\tobject:        " << object << endl;
    os << "\trecipient:     " << recipient << endl;
    os << "\tisNarration:   " << isNarration << endl;
    os << "\tbRelations:    " << bRelations.toString() << endl;
    os << "\tvArgument:     " << endl;
    for (auto ar : vArgument){
        os << "\t\t" << ar.first << " - " << ar.second << endl;
    }
    os << "\tvFocus:        " << endl;
    for (auto ar : mFocus){
        os << "\t\t" << ar.first << " - " << ar.second << endl;
    }
    os << "*************************************" << endl;
    return os.str();
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


void removeUnderscoreString(std::string &input){
    for (std::string::iterator it = input.begin(); it != input.end(); ++it) {
        if (*it == '_') {
            *it = ' ';
        }
    }
    for (std::string::iterator it = input.begin(); it != input.end(); ++it) {
        if (*it == '*') {
            *it = ',';
        }
    }
    for (std::string::iterator it = input.begin(); it != input.end(); ++it) {
        if (*it == '"') {
            *it = ' ';
        }
    }
}


void addUnderscoreString(std::string &input){
    for (std::string::iterator it = input.begin(); it != input.end(); ++it) {
        if (*it == ' ') {
            *it = '_';
        }
    }
    for (std::string::iterator it = input.begin(); it != input.end(); ++it) {
        if (*it == ',') {
            *it = '*';
        }
    }
}



bool isIn(vector<string> vec, string str){
    bool out = false;

    for (auto itS = vec.begin(); itS != vec.end(); itS++){
        out |= (*itS == str);
    }
    return out;
}
