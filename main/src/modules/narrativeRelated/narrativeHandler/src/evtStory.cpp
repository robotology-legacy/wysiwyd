#include <evtStory.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;
using namespace std;


void evtStory::print(){
    cout << "*************************************" << endl;
    cout << "\tinstance:      " << instance << endl;
    cout << "\tbegin:         " << begin<< endl;
    cout << "\tactivityName:  " << activity_name << endl;
    cout << "\tactivity_type: " << activity_type << endl;
    cout << "\tpredicate:     " << predicate << endl;
    cout << "\tagent:         " << agent << endl;
    cout << "\tobject:        " << object << endl;
    cout << "\trecipient:     " << recipient << endl;
    cout << "\tisNarration:   " << isNarration << endl;
    cout << "\tbRelations:    " << bRelations.toString() << endl;
    cout << "\tvArgument:     " << endl;
    for (auto ar : vArgument){
        cout << "\t\t" << ar.first << " - " << ar.second << endl;
    }
    cout << "\tvFocus:        " << endl;
    for (auto ar : mFocus){
        cout << "\t\t" << ar.first << " - " << ar.second << endl;
    }
    cout << "*************************************" << endl;

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