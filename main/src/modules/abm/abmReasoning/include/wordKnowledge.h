#include <abmReasoningFunction.h>

using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;


class wordContext
{
    int iPresence;
    string sLabel;
};

class wordObject
{
    int iOPCid;
    int iPresence;
    string sLabel;
};

class wordWord
{
    string sLabel;
    int iPresence;
};


class wordKnowledge
{
    vector<wordObject>      listWordObjects;
    vector<wordContext>     listWordContexts;
    vector<wordWord>        listWordWord;

    vector<vector<int>>     matObject2Context;
    vector<vector<int>>     matWord2Context;

    matrix3D_nonCubic     matObject2Word;
    matrix3D_nonCubic     matWord2Object;

    void    addInstance(pair<string,int> pObjectIdOPC, string sWord, vector<string> vContext);
};



