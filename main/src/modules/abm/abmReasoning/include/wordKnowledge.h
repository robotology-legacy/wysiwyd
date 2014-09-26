#include <abmReasoningFunction.h>

class wordKnowledge
{
public:

    std::vector<std::pair<std::string, int>>   listWordObjects;
    std::vector<std::pair<std::string, int>>   listWordContexts;
    std::vector<std::pair<std::string, int>>   listWordWord;

    std::vector<std::vector<int>>     matObject2Context;
    std::vector<std::vector<int>>     matWord2Context;

    matrix3D_nonCubic       matObject2Word;
    matrix3D_nonCubic       matWord2Object;


    void                    simulateData();

    std::pair<std::string, double>    getObjectFromWord(std::string sWord, std::vector<std::string> vContext);
    std::pair<std::string, double>    getWordFromObject(std::string sObject, std::vector<std::string> vContext);

    yarp::os::Bottle                  addInstance(std::string sObjectIdOPC, std::string sWord, std::vector<std::string> vContext);
    yarp::os::Bottle                  askWordKnowledge(std::string sQuestion, std::string sWhat, std::vector<std::string> vContext);
};



