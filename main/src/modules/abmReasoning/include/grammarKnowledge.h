#include <efaa/pronom.h>

using namespace yarp::os;
using namespace efaa::helpers;
using namespace std;


class grammarKnowledge
{	
public:

	grammarKnowledge();

	vector<pronom>	listPronom;
	vector<string>	listAddressee;
	vector<string>	listAgent;
	vector<string>	listSpeaker;
	vector<string>	listLabel;

	void			updateLabel();		// add all label to each pronom (all encountered names)
	void			addLabel(string sLabel);		// add an agent if not existing yet

	void			testModel(int iInstances, int condition);

	void			testSp(int iInstances, vector<string> vsLabelToAdd, int condition);
	void			testAd(int iInstances, vector<string> vsLabelToAdd, int condition);
	void			testAg(int iInstances, vector<string> vsLabelToAdd, int condition);
	void			testSu(int iInstances, vector<string> vsLabelToAdd, int condition);


	bool			addInteraction(Bottle bInput);
	
	pair<string, double>		findSubject(string sSpeaker, string sAddressee, string sAgent);
	pair<string, double>		findSubject2(string sSpeaker, string sAddressee, string sAgent);
	pair<string, double>		findAgent(string sSpeaker, string sAddressee, string sSubject);
	pair<string, double>		findAddressee(string sSpeaker, string sAgent, string sSubject);
	pair<string, double>		findSpeaker(string sAddressee, string sAgent, string sSubject);	

	void			simulateLearning(int Case,int iNbRep);
	void			simulateInstance(string Sp, string Ad, string Ag, string Su, int iNbRep);

	void			clear();
};

