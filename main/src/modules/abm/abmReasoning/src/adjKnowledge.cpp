#include <adjKnowledge.h>


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

