// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
* Authors: Grégoire Pointeau
* email:   gregoire.pointeau@inserm.fr
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* wysiwyd/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details


/!\ This module is under development and is not useable as it is right now /!\

Function used by abmReasoning to reason about ABM data 

@b timeDiff()string sTime1, string sTime2) : return a boolean informing if sTime1<sTime2. Not currently available.

@b string2Time : convert a string (ie : result of SQL query) to struct time.

@b time2string : convert s struct time to string (ie : for query)

@b ago2string : take a delay (int) and a unit (minut, hour, day, week, month), and return the date of delay*units to (delay-1)*unit


\section tested_os_sec Tested OS
Windows

\author Grégoire Pointeau, Maxime Petit
*/ 

#include <abmReasoningFunction.h>

using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;

string abmReasoningFunction::s_realOPC = "opc";
string abmReasoningFunction::s_mentalOPC = "mentalOPC";

double abmReasoningFunction::X_center   =  -0.68;
double abmReasoningFunction::Y_center   = -0.8;
double abmReasoningFunction::X_origin   =  0;
double abmReasoningFunction::Y_origin   = 0;
double abmReasoningFunction::table_radius   = 0.68;

double abmReasoningFunction::threshold_time_sequence = 3.   ;       //threshold of a same sequence
double abmReasoningFunction::height_location = 0.016        ;       // coordonate in Z of a location in the OPC
double abmReasoningFunction::size_location = 0.0005;                    // size in Z of a location

int abmReasoningFunction::color_dream_R = 255;                  // color of the dreamed object in the OPC
int abmReasoningFunction::color_dream_G = 255;
int abmReasoningFunction::color_dream_B = 255;

int abmReasoningFunction::color_loc_R = 255;                    // color of the locations in the OPC
int abmReasoningFunction::color_loc_G = 255;
int abmReasoningFunction::color_loc_B = 255;

int abmReasoningFunction::DIFFERENCE_DATE_IN_SECOND = 10000;    // threshold return of second if 2 actions are at different dates

unsigned int abmReasoningFunction::THRESHOLD_DETERMINE_INFLUENCE = 3;         // number of tries before determine if location

double abmReasoningFunction::FACTOR_LOCATION = 2 ;                  // factor of the size of a location : center +/- FACTOR_LOCATION * std dev
double abmReasoningFunction::THRESHOLD_IS_AT_LOCATION = 4;
double abmReasoningFunction::THRESHOLD_IS_AT_TEMPORAL_LOCATION = 12;
double abmReasoningFunction::THRESHOLD_IS_DISPERSION = 0.0001;

double abmReasoningFunction::LIFETIME_RELATION = 2. ;               // life time of a relation about the objects in the OPC

// PDDL
double abmReasoningFunction::THRESHOLD_INTERSECT_SUP = 0.75;
double abmReasoningFunction::THRESHOLD_INTERSECT_INF = 0.25;
double abmReasoningFunction::THRESHOLD_PRESENCE = 0.9;
double abmReasoningFunction::THRESHOLD_ABSENCE = 0.1;


//TAGS
string abmReasoningFunction::TAG_LOCATION   = "location";
string abmReasoningFunction::TAG_IS_AT_LOC  = "isAtLoc";
string abmReasoningFunction::TAG_DEFAULT    = "default";
string abmReasoningFunction::TAG_SPEAKER    = "speaker";
string abmReasoningFunction::TAG_ADRESSEE   = "addressee";
string abmReasoningFunction::TAG_SUBJECT    = "subject";
string abmReasoningFunction::TAG_AGENT      = "agent";
string abmReasoningFunction::TAG_NONE       = abmReasoningFunction::TAG_NONE;


//DB
string abmReasoningFunction::TAG_DB_ACTION      = "action";
string abmReasoningFunction::TAG_DB_COMPLEX     = "complex";
string abmReasoningFunction::TAG_DB_BEHAVIOR    = "behavior";
string abmReasoningFunction::TAG_DB_SHARED_PLAN = "sharedplan";
string abmReasoningFunction::TAG_DB_ARGUMENT    = "argument";
string abmReasoningFunction::TAG_DB_NONE        = abmReasoningFunction::TAG_NONE;
string abmReasoningFunction::TAG_DB_MANNER      = "manner";
string abmReasoningFunction::TAG_DB_UNKNOWN     = "unknown";


//GK
string abmReasoningFunction::TAG_SPEAKER_IS_RECEIVER    = "speaker_is_addressee";
string abmReasoningFunction::TAG_ADRESSEE_IS_SPEAKER = "addressee_is_speaker";
string abmReasoningFunction::TAG_SPEAKER_IS_AGENT   = "speaker_is_agent" ;
string abmReasoningFunction::TAG_AGENT_IS_SPEAKER   = "agent_is_speaker" ;
string abmReasoningFunction::TAG_AGENT_IS_RECEIVER  = "agent_is_addressee" ;
string abmReasoningFunction::TAG_ADRESSEE_IS_AGENT  = "addressee_is_agent" ;
int abmReasoningFunction::SIGMA_LEARNING_GRAMMAR = 1;
double abmReasoningFunction::THRESHOLD_CONFIDENCE_GRAMMAR = 0.13;


// ADJ KNOWLEDGE
double abmReasoningFunction::THRESHOLD_PVALUE_INFLUENCE_TIMING = 0.05;


abmReasoningFunction::abmReasoningFunction(ResourceFinder &rf)
{
    Bottle &bOPC = rf.findGroup("opc");

    s_realOPC = (bOPC.check("s_realOPC",Value("OPC")).asString());  
    s_mentalOPC = (bOPC.check("s_mentalOPC",Value("mentalOPC")).asString());  

    Bottle &bTable = rf.findGroup("table");

    X_center = (bTable.check("X_center",Value(-0.68)).asDouble());  
    Y_center = (bTable.check("Y_center",Value(-0.8)).asDouble());  
    X_origin = (bTable.check("X_origin",Value(0)).asDouble());  
    Y_origin = (bTable.check("Y_origin",Value(0)).asDouble());  

    table_radius = sqrt((X_center-X_origin)*(X_center-X_origin)+(Y_center-Y_origin)*(Y_center-Y_origin));

    Bottle &bMental = rf.findGroup("mental");

    threshold_time_sequence = bMental.check("threshold_time_sequence", Value(3)).asDouble();
    height_location = bMental.check("height_location", Value(0.016)).asDouble();
    size_location   = bMental.check("size_location", Value(0.005)).asDouble();

    color_dream_R = bMental.check("color_dream_R", Value(255)).asInt();
    color_dream_G = bMental.check("color_dream_G", Value(255)).asInt();
    color_dream_B = bMental.check("color_dream_B", Value(255)).asInt();

    color_loc_R = bMental.check("color_loc_R", Value(255)).asInt();
    color_loc_G = bMental.check("color_loc_G", Value(255)).asInt();
    color_loc_B = bMental.check("color_loc_B", Value(255)).asInt();
    DIFFERENCE_DATE_IN_SECOND = bMental.check("DIFFERENCE_DATE_IN_SECOND", Value(10000)).asInt();


    LIFETIME_RELATION   = bMental.check("LIFETIME_RELATION", Value(0.005)).asDouble();


    Bottle &bSpatialisation = rf.findGroup("spatialisation");

    THRESHOLD_DETERMINE_INFLUENCE = bSpatialisation.check("THRESHOLD_DETERMINE_INFLUENCE", Value(3)).asInt();
    FACTOR_LOCATION = bSpatialisation.check("FACTOR_LOCATION", Value(2)).asDouble();
    THRESHOLD_IS_AT_LOCATION = bSpatialisation.check("THRESHOLD_IS_AT_LOCATION", Value(4)).asDouble();
    THRESHOLD_IS_AT_TEMPORAL_LOCATION = bSpatialisation.check("THRESHOLD_IS_AT_TEMPORAL_LOCATION", Value(12)).asDouble();
    THRESHOLD_IS_DISPERSION = bSpatialisation.check("THRESHOLD_IS_DISPERSION", Value(0.0001)).asDouble();


    Bottle &bPDDL = rf.findGroup("PDDL");

    THRESHOLD_INTERSECT_SUP = (bPDDL.check("THRESHOLD_INTERSECT_SUP", Value(0.75)).asDouble());
    THRESHOLD_INTERSECT_INF = (bPDDL.check("THRESHOLD_INTERSECT_INF", Value(0.25)).asDouble());
    THRESHOLD_PRESENCE = (bPDDL.check("THRESHOLD_PRESENCE", Value(0.9)).asDouble());
    THRESHOLD_ABSENCE = (bPDDL.check("THRESHOLD_ABSENCE", Value(0.1)).asDouble());


    Bottle &bTag = rf.findGroup("TAGS");

    TAG_LOCATION = (bTag.check("TAG_LOCATION",Value("location")).asString());  
    TAG_IS_AT_LOC = (bTag.check("TAG_IS_AT_LOC", Value("isAtLoc")).asString());
    TAG_DEFAULT = (bTag.check("TAG_DEFAULT", Value("default")).asString());
    TAG_SPEAKER = (bTag.check("TAG_SPEAKER", Value("speaker")).asString());
    TAG_ADRESSEE = (bTag.check("TAG_ADRESSEE", Value("addressee")).asString());
    TAG_SUBJECT = (bTag.check("TAG_SUBJECT", Value("subject")).asString());
    TAG_AGENT = (bTag.check("TAG_AGENT", Value("agent")).asString());
    TAG_NONE = (bTag.check("TAG_NONE", Value("none")).asString());



    Bottle &bDB = rf.findGroup("DB");

    TAG_DB_ACTION       = bDB.check("TAG_DB_ACTION", Value("action")).asString();
    TAG_DB_COMPLEX      = bDB.check("TAG_DB_COMPLEX", Value("complex")).asString();
    TAG_DB_BEHAVIOR     = bDB.check("TAG_DB_BEHAVIOR", Value("behavior")).asString();
    TAG_DB_SHARED_PLAN  = bDB.check("TAG_DB_SHARED_PLAN", Value("sharedplan")).asString();
    TAG_DB_ARGUMENT     = bDB.check("TAG_DB_ARGUMENT", Value("argument")).asString();
    TAG_DB_NONE         = bDB.check("TAG_DB_NONE", Value(abmReasoningFunction::TAG_NONE)).asString();
    TAG_DB_MANNER       = bDB.check("TAG_DB_MANNER", Value("manner")).asString();
    TAG_DB_UNKNOWN      = bDB.check("TAG_DB_UNKNOWN", Value("unknown")).asString();


    Bottle &bGK = rf.findGroup("GK");
    TAG_SPEAKER_IS_RECEIVER         = bGK.check("TAG_SPEAKER_IS_RECEIVER", Value("speaker_is_addressee")).asString();
    TAG_ADRESSEE_IS_SPEAKER         = bGK.check("TAG_ADRESSEE_IS_SPEAKER", Value("addressee_is_speaker")).asString();
    TAG_SPEAKER_IS_AGENT            = bGK.check("TAG_SPEAKER_IS_AGENT", Value("speaker_is_agent")).asString();
    TAG_AGENT_IS_SPEAKER            = bGK.check("TAG_AGENT_IS_SPEAKER", Value("agent_is_speaker")).asString();
    TAG_AGENT_IS_RECEIVER           = bGK.check("TAG_AGENT_IS_RECEIVER", Value("agent_is_addressee")).asString();
    TAG_ADRESSEE_IS_AGENT           = bGK.check("TAG_ADRESSEE_IS_AGENT", Value("addressee_is_agent")).asString();
    SIGMA_LEARNING_GRAMMAR          = bGK.check("SIGMA_LEARNING_GRAMMAR", Value(1)).asInt();
    THRESHOLD_CONFIDENCE_GRAMMAR    = bGK.check("THRESHOLD_CONFIDENCE_GRAMMAR", Value(0.13)).asDouble();

    Bottle &bADJ = rf.findGroup("ADJ");
    THRESHOLD_PVALUE_INFLUENCE_TIMING    = bADJ.check("THRESHOLD_PVALUE_INFLUENCE_TIMING", Value(0.05)).asDouble();


}


// return if TM1 < TM2
bool abmReasoningFunction::timeDiff(struct tm TM1, struct tm TM2)
{
    if (TM1.tm_year<TM2.tm_year)
        return true;
    if (TM1.tm_year>TM2.tm_year)
        return false;
    if (TM1.tm_yday<TM2.tm_yday)
        return true;
    if (TM1.tm_yday>TM2.tm_yday)
        return false;
    if (TM1.tm_hour<TM2.tm_hour)
        return true;
    if (TM1.tm_hour>TM2.tm_hour)
        return false;
    if (TM1.tm_min<TM2.tm_min)
        return true;
    if (TM1.tm_min>TM2.tm_min)
        return false;
    if (TM1.tm_sec<TM2.tm_sec)
        return true;
    if (TM1.tm_sec>TM2.tm_sec)
        return false;
    return false;
}

/*
* Return the number of second btw T1 and T2
*
*/
int abmReasoningFunction::timeDiffSecondFromString(string T1, string T2)
{
    int iSecondReturn;
    int iHours = 0,     // number of hours of differences
        iMinutes = 0;   // number of minutes of differences

    struct tm TM1 = string2Time(T1);
    struct tm TM2 = string2Time(T2);

    // if the 2 actions didn't occured the same day
    if (TM1.tm_year != TM2.tm_year || TM1.tm_mday != TM2.tm_mday || TM1.tm_mon != TM2.tm_mon)
        return DIFFERENCE_DATE_IN_SECOND;

    iHours = (TM2.tm_hour-TM1.tm_hour);
    iMinutes = iHours*60 + (TM2.tm_min - TM1.tm_min);
    iSecondReturn = iMinutes*60 + (TM2.tm_sec - TM1.tm_sec);

    return iSecondReturn;
}

struct tm  abmReasoningFunction::string2Time(string sTime)
{
    char *cBuffer;
    cBuffer =  (char*)sTime.c_str();
    unsigned int i = 0;
    int iLevel = 0;
    //  int iHH,iMM,iSS; //iYear,iMonth,iDay,
    string sYear, sMonth, sDay, sHH, sMM, sSS = "";
    //  bool bYear,bMonth,bDay,bHH,bMM = false;
    while (cBuffer[i] != '\0')
    {
        char cTemp = cBuffer[i];
        if (cTemp == ' ' || cTemp == '-' || cTemp == ':' || cTemp == '+')
        {
            iLevel++;
        }
        else if (cTemp != '"')
        {
            switch (iLevel)
            {
            case 0:
                sYear += cTemp;
                break;
            case 1:
                sMonth += cTemp;
                break;
            case 2:
                sDay += cTemp;
                break;
            case 3:
                sHH += cTemp;
                break;
            case 4:
                sMM += cTemp;
                break;
            case 5:
                sSS += cTemp;
                break;
            }
        }
        i++;
    }
    struct tm *tOutput;
    time_t myTime;
    time(&myTime);  

    tOutput = localtime(&myTime);
    tOutput->tm_hour = atoi(sHH.c_str());
    tOutput->tm_min = atoi(sMM.c_str());
    tOutput->tm_sec = atoi(sSS.c_str());
    tOutput->tm_year = atoi(sYear.c_str());
    tOutput->tm_mon = atoi(sMonth.c_str());
    tOutput->tm_mday = atoi(sDay.c_str());
    mktime(tOutput);

    return *tOutput;
}


string abmReasoningFunction::time2string(struct tm Time)
{

    // get information
    int iHH, iMM, iSS, iMonth, iDay, iYear;
    iHH = Time.tm_hour;
    iMM = Time.tm_min;
    iSS = Time.tm_sec;
    iMonth = Time.tm_mon;
    iDay = Time.tm_mday;
    iYear = Time.tm_year;
    char cBuffer[19];
    string sRet;
    sRet ="";

    sprintf(cBuffer, "%d", int(iYear));
    sRet+=(cBuffer);
    sRet+=("-");

    sprintf(cBuffer, "%d", int(iMonth));
    sRet+=(cBuffer);
    sRet+=("-");

    sprintf(cBuffer, "%d", int(iDay));
    sRet+=(cBuffer);
    sRet+=(" ");

    sprintf(cBuffer, "%d", int(iHH));
    sRet+=(cBuffer);
    sRet+=(":");

    sprintf(cBuffer, "%d", int(iMM));
    sRet+=(cBuffer);
    sRet+=(":");

    sprintf(cBuffer, "%d", int(iSS));
    sRet+=(cBuffer);

    return sRet;
}


pair<string, string> abmReasoningFunction::ago2string(pair<int, string> pInput)
{
    // Get delay and unit in input, and return the 2 dates.

    pair<string, string> pOutput;
    int iDelay = pInput.first;
    string sUnit = pInput.second;
    struct tm * tmDate;
    string sBegin, sEnd;
    time_t myTime;
    time(&myTime);                  // get unix time
    tmDate = localtime(&myTime);

    // set begin and end time
    if (sUnit == "month" || sUnit == "months")
    {
        tmDate->tm_mon += (- iDelay );
        tmDate->tm_mday = 1;
        tmDate->tm_hour = 0;
        tmDate->tm_min = 0;
        tmDate->tm_sec = 0;
        mktime (tmDate);
        sBegin = time2string(*tmDate);
        tmDate = localtime(&myTime);
        tmDate->tm_mon += (- iDelay + 1);
        tmDate->tm_mday = 1;
        tmDate->tm_hour = 0;
        tmDate->tm_min = 0;
        tmDate->tm_sec = 0;
        mktime(tmDate);
        sEnd = time2string(*tmDate);
    }

    if (sUnit == "week" || sUnit == "weeks")
    {
        tmDate->tm_mday += (- 7*iDelay );
        tmDate->tm_hour = 0;
        tmDate->tm_min = 0;
        tmDate->tm_sec = 0;
        mktime (tmDate);
        sBegin = time2string(*tmDate);
        tmDate = localtime(&myTime);
        tmDate->tm_mday += (- 7*iDelay + 7);
        tmDate->tm_hour = 0;
        tmDate->tm_min = 0;
        tmDate->tm_sec = 0;
        mktime(tmDate);
        sEnd = time2string(*tmDate);
    }

    if (sUnit == "day" || sUnit == "days")
    {
        tmDate->tm_mday += (- iDelay );
        tmDate->tm_hour = 0;
        tmDate->tm_min = 0;
        tmDate->tm_sec = 0;
        mktime (tmDate);
        sBegin = time2string(*tmDate);
        tmDate = localtime(&myTime);
        tmDate->tm_mday += (- iDelay + 1);
        tmDate->tm_hour = 0;
        tmDate->tm_min = 0;
        tmDate->tm_sec = 0;
        mktime(tmDate);
        sEnd = time2string(*tmDate);
    }

    if (sUnit == "hour" || sUnit == "hours")
    {
        tmDate->tm_hour += (- iDelay );
        tmDate->tm_min = 0;
        tmDate->tm_sec = 0;
        mktime (tmDate);
        sBegin = time2string(*tmDate);
        tmDate = localtime(&myTime);
        tmDate->tm_hour += (- iDelay + 1);
        tmDate->tm_min = 0;
        tmDate->tm_sec = 0;
        mktime(tmDate);
        sEnd = time2string(*tmDate);
    }

    if (sUnit == "minut" || sUnit == "minuts")
    {   
        tmDate->tm_min += (- iDelay );
        tmDate->tm_sec = 0;
        mktime (tmDate);
        sBegin = time2string(*tmDate);
        tmDate = localtime(&myTime);
        tmDate->tm_min += (- iDelay + 1);
        tmDate->tm_sec = 0;
        mktime(tmDate);
        sEnd = time2string(*tmDate);
    }

    pOutput.first = sBegin;
    pOutput.second = sEnd;

    return pOutput;
}


vector<double> abmReasoningFunction::getCovMatrix(vector<pair<double, double> > vXY)
{
    int N = vXY.size(); // number of element

    double  muX = 0, // means
        muY = 0;

    for (int i = 0; i < N; i++)
    {
        muX += vXY[i].first;
        muY += vXY[i].second;
    }

    muX  /= (N*1.);
    muY  /= (N*1.);

    vector<double>  XimuX,  // (Xi - muX)
        YimuY;  // (Yi - muX)

    /* Calcul of the determinant of the covariance matrix.

    1/N * M *tM =   | a , b |
    .               | c , d |

    but here :  | a , b |
    .           | b , d |

    */
    double  a = 0,
        b = 0,
        c = 0,
        d = 0;

    // Creation of the matrix M for the abs values and M' the relatives values
    for (int i = 0; i < N ; i++)
    {
        XimuX.push_back(vXY[i].first-muX);
        YimuY.push_back(vXY[i].second-muY);
    }

    for (int i = 0; i < N ; i++)
    {
        a       += XimuX[i] * XimuX[i];
        b       += XimuX[i] * YimuY[i];
        c       += YimuY[i] * XimuX[i];
        d       += YimuY[i] * YimuY[i];
    }

    a /= (N*1.);
    b /= (N*1.);
    c /= (N*1.);
    d /= (N*1.);

    vector<double> vOutput;

    vOutput.push_back(a);
    vOutput.push_back(b);
    vOutput.push_back(c);
    vOutput.push_back(d);

    return vOutput;
}



vector<double> abmReasoningFunction::getCovMatrix(vector<double> vX, vector<double> vY)
{
    if (vX.size() == vY.size() )
    {
        cout << "Error in abmReasoningFunction::getCovMatrix(vector<double> vX, vector<double> vY) : vX and vY size different" << endl;
        vector<double> vOutput;
        return vOutput;
    }

    int N = vX.size(); // number of element

    vector<pair<double, double> > vectDouble;

    for (int i = 0 ; i < N ; i++)
    {
        pair<double, double> pTemp(vX[i], vY[i]);
        vectDouble.push_back(pTemp);
    }

    return getCovMatrix(vectDouble);

}



/**
* Return the Mahalanobis distance of a point to a cluster
* @param vX, vY : cluster
* @param XY : point to analyse
*/
double abmReasoningFunction::getMahalaDist(vector<double> vX, vector<double> vY, pair<double, double> XY)
{

    vector<double> covMatrix = getCovMatrix(vX, vY);

    double X, Y, Xp, Yp,D, a, b, c, d, muX=0, muY=0;

    for (unsigned int i = 0; i < vX.size(); i++)
    {
        muX += vX[i];
        muY += vY[i];
    }

    muX /= (vX.size()*1.);
    muY /= (vY.size()*1.);

    X = XY.first - muX;
    Y = XY.second - muY;

    a = covMatrix[0];
    b = covMatrix[1];
    c = covMatrix[2];
    d = covMatrix[3];

    /*
    Mahalanobis distance : D
    Covariance Matrix : | a , b |
    | c , d |


    Inverse : 

    1     | d , -b | 
    --- * |        |
    det   |-c ,  a |


    Point : X, Y

    | d ,-b |(X)
    1          | -c, a |(Y)
    D = ---( X , Y )( X', Y')(D)
    det
    */

    Xp =  d*X - c*Y;
    Yp = -b*X + a*Y;

    D = X*Xp + Y*Yp;

    D /= (a*d-b*c);

    return D;
}


/*
* return a couple of coordonates from a string input.
*/
pair<double, double> abmReasoningFunction::coordFromString(string sInput)
{
    char *cInput;
    cInput = (char*)sInput.c_str();
    int iLevel = 0;
    unsigned int data =0;
    pair<double, double> pOutput;
    string sX = "", sY = "";
    while (cInput[data] != '\0')
    {
        char cTemp = cInput[data];
        if (cTemp == ',')
        {
            iLevel++;
        }
        if (cTemp != '{' && cTemp != ',')
        {
            switch (iLevel)
            {
            case 0:
                sX += cTemp;
                break;
            case 1:
                sY+=cTemp;
                break;
            }
        }
        data++;
    }
    pOutput.first = atof(sX.c_str());
    pOutput.second = atof(sY.c_str());

    return pOutput;
}


/*
* return a tuple of 3 int from a string input
*/
tuple<int, int, int> abmReasoningFunction::tupleIntFromString(string sInput)
{
    tuple<int, int, int>    tOutput;
    char *cInput;
    cInput = (char*)sInput.c_str();
    int iLevel = 0;
    unsigned int data =0;
    string sX = "", sY = "", sZ ="";
    while (cInput[data] != '\0')
    {
        char cTemp = cInput[data];
        if (cTemp == ',')
        {
            iLevel++;
        }
        if (cTemp != '{' && cTemp != ',')
        {
            switch (iLevel)
            {
            case 0:
                sX += cTemp;
                break;
            case 1:
                sY += cTemp;
            case 2:
                sZ += cTemp;
                break;
            }
        }
        data++;
    }

    get<0>(tOutput) = atoi(sX.c_str());
    get<1>(tOutput) = atoi(sY.c_str());
    get<2>(tOutput) = atoi(sZ.c_str());

    return tOutput;
}


/*
* return a tuple of 3 double from a string input
*/
tuple<double, double, double> abmReasoningFunction::tupleDoubleFromString(string sInput)
{
    tuple<double, double, double>   tOutput;
    char *cInput;
    cInput = (char*)sInput.c_str();
    int iLevel = 0;
    unsigned int data =0;
    string sX = "", sY = "", sZ ="";
    while (cInput[data] != '\0')
    {
        char cTemp = cInput[data];
        if (cTemp == ',')
        {
            iLevel++;
        }
        if (cTemp != '{' && cTemp != ',')
        {
            switch (iLevel)
            {
            case 0:
                sX += cTemp;
                break;
            case 1:
                sY += cTemp;
            case 2:
                sZ += cTemp;
                break;
            }
        }
        data++;
    }

    get<0>(tOutput) = atof(sX.c_str());
    get<1>(tOutput) = atof(sY.c_str());
    get<2>(tOutput) = atof(sZ.c_str());

    return tOutput;
}


