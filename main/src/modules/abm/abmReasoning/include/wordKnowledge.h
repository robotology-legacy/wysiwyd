#include <abmReasoningFunction.h>

using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;


class wordContext
{
public:
	int iPresence;
	string sLabel;
};

class wordObject
{
public:
	int iPresence;
	string sLabel;
};

class wordWord
{
public:
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

    matrix3D_nonCubic       matObject2Word;
    matrix3D_nonCubic       matWord2Object;

	void					getObjectFromWord(string sWord, vector<string> vContext);
	void					getWordFromObject(string sObject, vector<string> vContext);

    Bottle                  addInstance(string sObjectIdOPC, string sWord, vector<string> vContext);
};



