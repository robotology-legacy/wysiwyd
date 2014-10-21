#ifndef _GRAMMARKNOWLEDGE_H_
#define _GRAMMARKNOWLEDGE_H_

#include <pronom.h>



class grammarKnowledge
{ 
public:

    grammarKnowledge();

    std::vector<pronom> listPronom;
    std::vector<std::string> listAddressee;
    std::vector<std::string> listAgent;
    std::vector<std::string> listSpeaker;
    std::vector<std::string> listLabel;

    void updateLabel(); // add all label to each pronom (all encountered names)
    void addLabel(std::string sLabel); // add an agent if not existing yet

    void testModel(int iInstances, int condition);

    void testSp(int iInstances, std::vector<std::string> vsLabelToAdd, int condition);
    void testAd(int iInstances, std::vector<std::string> vsLabelToAdd, int condition);
    void testAg(int iInstances, std::vector<std::string> vsLabelToAdd, int condition);
    void testSu(int iInstances, std::vector<std::string> vsLabelToAdd, int condition);


    bool addInteraction(yarp::os::Bottle bInput);

    std::pair<std::string, double> findSubject(std::string sSpeaker, std::string sAddressee, std::string sAgent);
    std::pair<std::string, double> findAgent(std::string sSpeaker, std::string sAddressee, std::string sSubject);
    std::pair<std::string, double> findAddressee(std::string sSpeaker, std::string sAgent, std::string sSubject);
    std::pair<std::string, double> findSpeaker(std::string sAddressee, std::string sAgent, std::string sSubject); 

    void simulateLearning(int Case,int iNbRep);
    void simulateInstance(std::string Sp, std::string Ad, std::string Ag, std::string Su, int iNbRep);

    void clear();
};


#endif