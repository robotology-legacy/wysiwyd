#include <adjKnowledge.h>


adjKnowledge::adjKnowledge()
{
    fTimingInfluence = false;
    fAbsolutInfluence = false;
    fDeltaInfluence = false;
    fFromInfluence = false;
}


bool adjKnowledge::fromBottle(Bottle bInput)
{

    return true;
}

/**
* Return the distance of a move from a cluster (skInput)
* First determines the influence of the cluster
* @param skInput is the spatialKnoledge to search the distance
* @param XY is the final state of the move
* @param MOVE is the delta of the move done
*/
pair <int, double> adjKnowledge::distFromMove(pair<double, double> XY, pair<double, double> MOVE)
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
pair <double, double> adjKnowledge::coordRelative(double Xo, double Yo, double Xh, double Yh)
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



void adjKnowledge::test()
{

}


/** Determine if the adjective influences the timing.
First check the adjtive globaly
then check for each action if there is a link.

update the parameter: fTimingInfluence
*/
void adjKnowledge::determineTimingInfluence()
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
void adjKnowledge::determineSpatialInfluence()
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

    fDeltaInfluence = (abmReasoningFunction::threshold_is_dispersion > deter_CovD);

    fAbsolutInfluence =  (abmReasoningFunction::threshold_is_dispersion > deter_CovB);

    fFromInfluence = fAbsolutInfluence && fDeltaInfluence;

    if (fDeltaInfluence || fAbsolutInfluence)
        return ;

    // 2 if no influence, check action by action

    for (map<string, pair< vector<pair<double, double> >, vector<pair<double, double> > > >::iterator itMap = mActionAbsolut.begin() ; itMap != mActionAbsolut.end() ; itMap ++)
    {
        covMatrixB = abmReasoningFunction::getCovMatrix(itMap->second.first);
        covMatrixD = abmReasoningFunction::getCovMatrix(mActionDelta[itMap->first].first);

        deter_CovB = (covMatrixB[0] * covMatrixB[3])-(covMatrixB[1] * covMatrixB[2]);
        deter_CovD = (covMatrixD[0] * covMatrixD[3])-(covMatrixD[1] * covMatrixD[2]);

        fDeltaInfluence |= (abmReasoningFunction::threshold_is_dispersion > deter_CovD);

        fAbsolutInfluence |=  (abmReasoningFunction::threshold_is_dispersion > deter_CovB);

        fFromInfluence |= (abmReasoningFunction::threshold_is_dispersion > deter_CovD) && (abmReasoningFunction::threshold_is_dispersion > deter_CovB) ;
    }



}