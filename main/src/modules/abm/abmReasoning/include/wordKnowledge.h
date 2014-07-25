#include <abmReasoningFunction.h>

using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;



class wordKnowledge
{
public:

    vector<pair<string, int>>	listWordObjects;
    vector<pair<string, int>>	listWordContexts;
    vector<pair<string, int>>	listWordWord;

    vector<vector<int>>     matObject2Context;
    vector<vector<int>>     matWord2Context;

    matrix3D_nonCubic       matObject2Word;
    matrix3D_nonCubic       matWord2Object;


    void					simulateData();

    pair<string, double>	getObjectFromWord(string sWord, vector<string> vContext);
    pair<string, double>	getWordFromObject(string sObject, vector<string> vContext);

    Bottle                  addInstance(string sObjectIdOPC, string sWord, vector<string> vContext);
};



