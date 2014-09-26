#include <abmReasoningFunction.h>


class spatialKnowledge
{
public:
    std::pair<double, double>    coordRelative(double Xo, double Yo, double Xh, double Yh);      // return the relatve coordinates of an object from an other agent
    bool                    fromBottle(yarp::os::Bottle bInput);

    bool    isRelative;
    bool    isAbsolut;

    std::string                  sName;
    std::string                  sArgument;
    std::string                  sDependance;

    int                     iSize;
    int                     iNbParam;
    int                     iInfluence;

    std::vector<double>          vX;
    std::vector<double>          vY;
    std::vector<double>          vFromX;
    std::vector<double>          vFromY;
    std::vector<double>          vDX;
    std::vector<double>          vDY;

    void                        determineInfluence();
    std::pair <int, double>      distFromMove(std::pair<double, double> XY, std::pair<double, double> MOVE);
    std::vector<double>          determineAbsolut();
    void                    updateDataFinalDepart();

};