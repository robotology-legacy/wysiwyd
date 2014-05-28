#include <abmReasoningFunction.h>

using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;



class spatialKnowledge
{
public:
	pair<double, double>	coordRelative(double Xo, double Yo, double Xh, double Yh);		// return the relatve coordinates of an object from an other agent
	bool					fromBottle(Bottle bInput);

	bool	isRelative;
	bool	isAbsolut;

	string					sName;
	string					sArgument;
	string					sDependance;

	int						iSize;
	int						iNbParam;
	int						iInfluence;
	
	vector<double>			vX;
	vector<double>			vY;
	vector<double>			vFromX;
	vector<double>			vFromY;
	vector<double>			vDX;
	vector<double>			vDY;

	void						determineInfluence();
	pair <int, double>		distFromMove(pair<double, double> XY, pair<double, double> MOVE);
	vector<double>			determineAbsolut();
	void					updateDataFinalDepart();

};