#include <advKnowledge.h>


using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;


advKnowledge::advKnowledge()
{
    fTimingInfluence = false;
    fAbsolutInfluence = false;
    fDeltaInfluence = false;
    fFromInfluence = false;
}


/**
Add an interaction in the advKnowledge
bInput format: 
(action sAction_name)
(timing timing)
(coordinate X Y dX dY)
*/
void    advKnowledge::addInteraction(Bottle bInput)
{

    string sAction = bInput.check("action", Value("none")).asString();
    vdGnlTiming.push_back(bInput.check("timing", Value(0.)).asDouble());

    Bottle bTemp = *bInput.get(2).asList();
    pair<double, double>    pXY(bTemp.get(1).asDouble(), bTemp.get(2).asDouble());
    pair<double, double>    pDXY(bTemp.get(3).asDouble(), bTemp.get(4).asDouble());

    mActionTiming[sAction].first.push_back(bInput.check("timing", Value(0.)).asDouble());

    vdGnlXY.push_back(pXY);
    vdGnlDelta.push_back(pDXY);

    mActionAbsolut[sAction].push_back(pXY);
    mActionDelta[sAction].push_back(pDXY);
}


/**
Add an other interaction in the advKnowledge (event adj is not this one, this is used for a comparaison)
bInput format: 
(action sAction_name)
(timing timing)
(coordinate X Y dX dY)
*/
void    advKnowledge::addOtherInteraction(Bottle bInput)
{
    string sAction = bInput.check("action", Value("none")).asString();
    vdNoGnlTiming.push_back(bInput.check("timing", Value(0.)).asDouble());
    mActionTiming[sAction].second.push_back(bInput.check("timing", Value(0.)).asDouble());
}


/**
* Return the distance of a move from a cluster (skInput)
* First determines the influence of the cluster
* @param skInput is the spatialKnoledge to search the distance
* @param XY is the final state of the move
* @param MOVE is the delta of the move done
*/
pair <int, double> advKnowledge::distFromMove(pair<double, double> XY, pair<double, double> MOVE)
{
    //// return the influence (Abs or Delta) and the Mahalanobis distance
    //int Influence =0;
    //double Dist=0;
    //if (isAbsolut)
    //{
    //    Dist = abmReasoningFunction::getMahalaDist(vX, vY, XY);
    //    Influence = 1;
    //}
    //else if (isRelative)
    //{
    //    Dist = abmReasoningFunction::getMahalaDist(vDX,vDY, MOVE);
    //    Influence = 2;
    //}

    pair <int, double> pOutput;
    //    pOutput.first = Influence;
    //    pOutput.second = Dist;

    return pOutput;
}


/*
*   Return the relative coordinates of an object of coordinates Xo Yo on the table, for an agent in Xh Yh, looking to the center of the table
*
*/
pair <double, double> advKnowledge::coordRelative(double Xo, double Yo, double Xh, double Yh)
{
    pair<double, double>  pReturn;

    double X,   // X relative Coordinate
        Y,      // Y relative coordinate
        R,      // Distance from agent to object
        A,      // temporaty for atan2 calculation
        B,      // temporaty for atan2 calculation
        Pi = atan(1.)*4,
        Theta;  // angle of the Human relative to the table

    R = sqrt ((Xh-Xo)*(Xh-Xo) + (Yh-Yo)*(Yh-Yo));

    A = Xh - abmReasoningFunction::X_center;
    B = Yh - abmReasoningFunction::Y_center;

    Theta = atan2( B , A ) + Pi/2.;

    X = (Xo - Xh)*cos(Theta) + (Yo-Yh)*sin(Theta);
    Y = -(Xo - Xh)*sin(Theta) + (Yo - Yh)*cos(Theta);

    pReturn.first = X;
    pReturn.second = Y;

    return pReturn;
}


/**
Will try to determine if the adjective is related to space, or time.
First, tries time, if not, try if spatial.
*/
void advKnowledge::determineInfluence()
{
    fTimingInfluence = false;
    fAbsolutInfluence = false;
    fDeltaInfluence = false;
    fFromInfluence = false;

    if (vdGnlTiming.size() >= abmReasoningFunction::THRESHOLD_DETERMINE_INFLUENCE)
    {
        determineTimingInfluence();
        if (!fTimingInfluence)
        {
            determineSpatialInfluence();
        }
    }
}


/** Determine if the adjective influences the timing.
First check the adjtive globaly'
then check for each action if there is a link.

update the parameter: fTimingInfluence
*/
void advKnowledge::determineTimingInfluence()
{
    double bothtails,
        lefttail,
        righttail;

    abmReasoningFunction::studentttest2(vdGnlTiming, vdNoGnlTiming, &bothtails, &lefttail, &righttail);

    cout << "bothtails: " << bothtails << endl;
    //    cout << "lefttail : " << lefttail << endl;
    //    cout << "righttail: " << righttail << endl;


    // if from a general point of view, the adjective influence the timing
    if (bothtails < abmReasoningFunction::THRESHOLD_PVALUE_INFLUENCE_TIMING)
    {
        fTimingInfluence = true;
        return ;
    }
    

    // else check for each association action/adjective:
    for (map<string, pair< vector<double>, vector<double> > >::iterator itMap = mActionTiming.begin() ; itMap != mActionTiming.end() ; itMap++)
    {
        abmReasoningFunction::studentttest2(itMap->second.first, itMap->second.second, &bothtails, &lefttail, &righttail);
        if (bothtails < abmReasoningFunction::THRESHOLD_PVALUE_INFLUENCE_TIMING)
        {
            cout << sLabel << " influences timing when correlated to the action : " << itMap->first << endl;
            fTimingInfluence = true;
        }
    }
}


/**
Determine if the adjective influence in term of spatial relations
*/
void advKnowledge::determineSpatialInfluence()
{

    // 1 For all actions:

    // B : Final point
    // D : move (delta)

    // Get the covariance matrix for abs and delta.
    vector<double>      covMatrixB = abmReasoningFunction::getCovMatrix(vdGnlXY),
        covMatrixD = abmReasoningFunction::getCovMatrix(vdGnlDelta);


    //double sigmaD;
    //double sumdX = 0, sumdY =0;
    //for (unsigned int i = 0 ; i < vDX.size() ; i++)
    //{
    //    sumdX += vDX[i];
    //    sumdY += vDY[i];
    //}

    //sigmaD = sqrt(sumdX*sumdX + sumdY*sumdY);

    double deter_CovB = (covMatrixB[0] * covMatrixB[3])-(covMatrixB[1] * covMatrixB[2]);
    double deter_CovD = (covMatrixD[0] * covMatrixD[3])-(covMatrixD[1] * covMatrixD[2]);

    fDeltaInfluence = (abmReasoningFunction::THRESHOLD_IS_DISPERSION > deter_CovD);

    fAbsolutInfluence =  (abmReasoningFunction::THRESHOLD_IS_DISPERSION > deter_CovB);

    fFromInfluence = fAbsolutInfluence && fDeltaInfluence;

    if (fDeltaInfluence || fAbsolutInfluence)
        return ;

    // 2 if no influence, check action by action

    for (map<string, vector< pair<double, double > > >::iterator itMap = mActionAbsolut.begin() ; itMap != mActionAbsolut.end() ; itMap ++)
    {
        covMatrixB = abmReasoningFunction::getCovMatrix(itMap->second);
        covMatrixD = abmReasoningFunction::getCovMatrix(mActionDelta[itMap->first]);

        deter_CovB = (covMatrixB[0] * covMatrixB[3])-(covMatrixB[1] * covMatrixB[2]);
        deter_CovD = (covMatrixD[0] * covMatrixD[3])-(covMatrixD[1] * covMatrixD[2]);

        fDeltaInfluence |= (abmReasoningFunction::THRESHOLD_IS_DISPERSION > deter_CovD);

        fAbsolutInfluence |=  (abmReasoningFunction::THRESHOLD_IS_DISPERSION > deter_CovB);

        fFromInfluence |= (abmReasoningFunction::THRESHOLD_IS_DISPERSION > deter_CovD) && (abmReasoningFunction::THRESHOLD_IS_DISPERSION > deter_CovB) ;
    }
}




