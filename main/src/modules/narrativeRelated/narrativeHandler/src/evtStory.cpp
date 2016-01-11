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