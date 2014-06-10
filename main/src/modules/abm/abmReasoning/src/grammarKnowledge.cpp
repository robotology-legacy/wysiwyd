// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2012 EFAA Consortium, European Commission FP7 Project IST-270490
* Authors: Grégoire Pointeau, Maxime Petit
* email:   gregoire.pointeau@inserm.fr, maxime.petit@inserm.fr
* website: http://efaa.upf.edu/ 
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* $EFAA_ROOT/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details

\section tested_os_sec Tested OS
Windows

\author Grégoire Pointeau, Maxime Petit
*/ 

#include <grammarKnowledge.h>

const double weightSimple = 5.;
const double weightDouble = 2;
const double weightTriple = 5.;

pair<string, double> grammarKnowledge::findAgent(string X, string Y, string P)
{
    pair<string, double>    pReturn("none", 0);

    /*
    P is Subject (pronom)
    X is Speaker
    Y is Addressee
    Z is Agent
    */

    pronom currentPronom;

    bool fPronom = false;
    // get the pronom :
    for (vector<pronom>::iterator itPronom = listPronom.begin() ; itPronom != listPronom.end() ; itPronom++)
    {
        itPronom->m3Data.addLabel(X);
        itPronom->m3Data.addLabel(Y);
        if (itPronom->sSubject == P)
        {
            fPronom = true;
            currentPronom = *itPronom;
        }
    }

    vector<pair<string, double> >   vScore;

    // if the pronom is known : check the properties
    if (!fPronom)
    {
        pronom newPronom;
        for (vector<string>::iterator itLabel = listPronom.begin()->m3Data.vsLabels.begin() ; itLabel < listPronom.begin()->m3Data.vsLabels.end() ; itLabel++)
        {
            newPronom.m3Data.addLabel(*itLabel);
        }

        newPronom.sSubject = P;
        listPronom.push_back(newPronom);

        currentPronom =  listPronom.back();
    }

    matrix3D currentData = currentPronom.m3Data;
    int sumMat = currentData.iSum;

    for (vector<string>::iterator itAgent = currentData.vsLabels.begin() ; itAgent != currentData.vsLabels.end() ; itAgent++)
    {
        string Z = *itAgent;

        scoreProp   tXYZP; // X given Y Z and P

        scoreProp   tXY;    // X == Y given P
        scoreProp   tXZ;    // X == Z given P
        scoreProp   tYZ;    // Y == Z given P

        scoreProp   tXP;    // X == P given P
        scoreProp   tYP;    // Y == P given P
        scoreProp   tZP;    // Z == P given P

        double scoreTest = 0;

        // X Y Z P 
        tXYZP.A = currentData.get(X,Y,Z);
        tXYZP.C = currentData.sumLineXZ(X,Y) - tXYZP.A;

        /*-------------*/

        // using X == Y given P
        tXY.A   = currentData.sumDiagDouble("z");
        tXY.B   = sumMat - tXY.A;

        // using X == Z given P
        tXZ.A   = currentData.sumDiagDouble("y");
        tXZ.B   = sumMat - tXZ.A;

        // using Y == Z given P
        tYZ.A   = currentData.sumDiagDouble("x");
        tYZ.B   = sumMat - tYZ.A;

        /*-------------*/

        // using X == P given P
        tXP.A   = currentData.sumPlan("x", X);
        tXP.B   = sumMat - tXP.A;

        // using Y == P given P
        tYP.A   = currentData.sumPlan("y", Y);
        tYP.B   = sumMat - tYP.A;

        // using Z == P given P
        tZP.A   = currentData.sumPlan("z", Z);
        tZP.B   = sumMat - tZP.A;

        // FOR EACH SUB PRONOM
        for (vector<pronom>::iterator itSubPronom = listPronom.begin() ; itSubPronom != listPronom.end() ; itSubPronom++)
        {
            matrix3D SubMat = itSubPronom->m3Data;
            int iSumSubMat = SubMat.getSum();

            // IF PRONOM != SUBPRONOM
            if (itSubPronom->sSubject != P)
            {
                // using X == Y given P
                tXY.C   += SubMat.sumDiagDouble("z");
                tXY.D   += iSumSubMat - SubMat.sumDiagDouble("z");

                // using X == Z given P
                tXZ.C   += SubMat.sumDiagDouble("y");
                tXZ.D   += iSumSubMat - SubMat.sumDiagDouble("y");

                // using Y == Z given P
                tYZ.C   += SubMat.sumDiagDouble("x");
                tYZ.D   += iSumSubMat - SubMat.sumDiagDouble("x");

                /*-------------*/

                // using X == P given P
                tXP.C   += SubMat.sumPlan("x", X);
                tXP.D   += iSumSubMat - SubMat.sumPlan("x", X);

                // using Y == P given P
                tYP.C   += SubMat.sumPlan("y", Y);
                tYP.D   += iSumSubMat - SubMat.sumPlan("y", Y);

                // using Z == P given P
                tZP.C   += SubMat.sumPlan("z", Z);
                tZP.D   += iSumSubMat - SubMat.sumPlan("z", Z);
            }


        }
        //END FOR EACH SUBPRONOM


        // SUM of the SCORE of each PROPERTY
        scoreTest   +=  weightTriple*tXYZP.getScoreSum(true); // X given Y Z and P

        scoreTest   +=  weightDouble*tXY.getScoreSum(X==Y); // X == Y given P
        scoreTest   +=  weightDouble*tXZ.getScoreSum(X==Z); // X == Z given P
        scoreTest   +=  weightDouble*tYZ.getScoreSum(Y==Z); // Y == Z given P
        
        scoreTest   +=  weightSimple*tXP.getScoreSum(X==P); // X == P given P
        scoreTest   +=  weightSimple*tYP.getScoreSum(Y==P); // Y == P given P
        scoreTest   +=  weightSimple*tZP.getScoreSum(Z==P); // Z == P given P

        cout << endl << "agent : " << Z << endl;
        cout << "XYZP  pv : " << tXYZP.chiSquare() << "\t A=" << tXYZP.A << "\t B=" << tXYZP.B << "\t C=" << tXYZP.C << "\t D=" << tXYZP.D << "\t score : " <<  weightTriple*tXYZP.getScoreSum(true) <<  endl;
        cout << "XY    pv : " << tXY.chiSquare()  << "\t A=" << tXY.A  << "\t B=" << tXY.B  << "\t C=" << tXY.C << "\t D=" << tXY.D << "\t score : " <<         weightDouble*tXY.getScoreSum(X==Y) <<  endl;
        cout << "XZ    pv : " << tXZ.chiSquare()  << "\t A=" << tXZ.A  << "\t B=" << tXZ.B  << "\t C=" << tXZ.C << "\t D=" << tXZ.D << "\t score : " <<         weightDouble*tXZ.getScoreSum(X==Z) <<  endl;
        cout << "YZ    pv : " << tYZ.chiSquare()  << "\t A=" << tYZ.A  << "\t B=" << tYZ.B  << "\t C=" << tYZ.C << "\t D=" << tYZ.D << "\t score : " <<         weightDouble*tYZ.getScoreSum(Y==Z) <<  endl;
        cout << "XP    pv : " << tXP.chiSquare()  << "\t A=" << tXP.A  << "\t B=" << tXP.B  << "\t C=" << tXP.C << "\t D=" << tXP.D << "\t score : " <<         weightSimple*tXP.getScoreSum(X==P) <<  endl;
        cout << "YP    pv : " << tYP.chiSquare()  << "\t A=" << tYP.A  << "\t B=" << tYP.B  << "\t C=" << tYP.C << "\t D=" << tYP.D << "\t score : " <<         weightSimple*tYP.getScoreSum(Y==P) <<  endl;
        cout << "ZP    pv : " << tZP.chiSquare()  << "\t A=" << tZP.A  << "\t B=" << tZP.B  << "\t C=" << tZP.C << "\t D=" << tZP.D << "\t score : " <<         weightSimple*tZP.getScoreSum(Z==P) <<  endl;

        pair<string, double>    pTemp(Z, scoreTest);

        vScore.push_back(pTemp);
    }

    cout << endl;


    double dScoreMax = -100000;
    string sResult;
    cout << "Hesitate between Product : "  << endl;
    for (vector<pair<string, double > >::iterator it_SubSc = vScore.begin() ; it_SubSc != vScore.end() ; it_SubSc++)
    {
        double dScoreTemp = it_SubSc->second;
        cout << "\t" << it_SubSc->first << "\t score = " << dScoreTemp << endl;
        if (dScoreTemp > dScoreMax)
        {
            dScoreMax = dScoreTemp;
            sResult = it_SubSc->first;
        }
    }   
    
    if (dScoreMax <= 0)
    {
        cout << "Can't take a decision on which ag to select" << endl;
        sResult = "none";
    }

    cout << "Final choice : " << sResult << "\t score : " << dScoreMax << endl;
    pReturn.first = sResult;
    pReturn.second = dScoreMax;

    return pReturn;
}

pair<string, double> grammarKnowledge::findAddressee(string X, string Z, string P)
{
    pair<string, double>    pReturn("none", 0);

    /*
    P is Subject (pronom)
    X is Speaker
    Y is Addressee
    Z is Agent
    */

    pronom currentPronom;

    bool fPronom = false;
    // get the pronom :
    for (vector<pronom>::iterator itPronom = listPronom.begin() ; itPronom != listPronom.end() ; itPronom++)
    {
        itPronom->m3Data.addLabel(X);
        itPronom->m3Data.addLabel(Z);
        if (itPronom->sSubject == P)
        {
            fPronom = true;
            currentPronom = *itPronom;
        }
    }

    vector<pair<string, double> >   vScore;

    // if the pronom is known : check the properties
    if (!fPronom)
    {
        pronom newPronom;
        for (vector<string>::iterator itLabel = listPronom.begin()->m3Data.vsLabels.begin() ; itLabel < listPronom.begin()->m3Data.vsLabels.end() ; itLabel++)
        {
            newPronom.m3Data.addLabel(*itLabel);
        }

        newPronom.sSubject = P;
        listPronom.push_back(newPronom);

        currentPronom =  listPronom.back();
    }

    matrix3D currentData = currentPronom.m3Data;
    int sumMat = currentData.iSum;

    for (vector<string>::iterator itAddressee = currentData.vsLabels.begin() ; itAddressee != currentData.vsLabels.end() ; itAddressee++)
    {
        string Y = *itAddressee;

        scoreProp   tXYZP; // X given Y Z and P

        scoreProp   tXY;    // X == Y given P
        scoreProp   tXZ;    // X == Z given P
        scoreProp   tYZ;    // Y == Z given P

        scoreProp   tXP;    // X == P given P
        scoreProp   tYP;    // Y == P given P
        scoreProp   tZP;    // Z == P given P

        double scoreTest = 0;

        // X Y Z P 
        tXYZP.A = currentData.get(X,Y,Z);
        tXYZP.C = currentData.sumLineXZ(X,Z) - tXYZP.A;

        /*-------------*/

        // using X == Y given P
        tXY.A   = currentData.sumDiagDouble("z");
        tXY.B   = sumMat - tXY.A;

        // using X == Z given P
        tXZ.A   = currentData.sumDiagDouble("y");
        tXZ.B   = sumMat - tXZ.A;

        // using Y == Z given P
        tYZ.A   = currentData.sumDiagDouble("x");
        tYZ.B   = sumMat - tYZ.A;

        /*-------------*/

        // using X == P given P
        tXP.A   = currentData.sumPlan("x", X);
        tXP.B   = sumMat - tXP.A;

        // using Y == P given P
        tYP.A   = currentData.sumPlan("y", Y);
        tYP.B   = sumMat - tYP.A;

        // using Z == P given P
        tZP.A   = currentData.sumPlan("z", Z);
        tZP.B   = sumMat - tZP.A;

        // FOR EACH SUB PRONOM
        for (vector<pronom>::iterator itSubPronom = listPronom.begin() ; itSubPronom != listPronom.end() ; itSubPronom++)
        {
            matrix3D SubMat = itSubPronom->m3Data;
            int iSumSubMat = SubMat.getSum();

            // IF PRONOM != SUBPRONOM
            if (itSubPronom->sSubject != P)
            {
                // using X == Y given P
                tXY.C   += SubMat.sumDiagDouble("z");
                tXY.D   += iSumSubMat - SubMat.sumDiagDouble("z");

                // using X == Z given P
                tXZ.C   += SubMat.sumDiagDouble("y");
                tXZ.D   += iSumSubMat - SubMat.sumDiagDouble("y");

                // using Y == Z given P
                tYZ.C   += SubMat.sumDiagDouble("x");
                tYZ.D   += iSumSubMat - SubMat.sumDiagDouble("x");

                /*-------------*/

                // using X == P given P
                tXP.C   += SubMat.sumPlan("x", X);
                tXP.D   += iSumSubMat - SubMat.sumPlan("x", X);

                // using Y == P given P
                tYP.C   += SubMat.sumPlan("y", Y);
                tYP.D   += iSumSubMat - SubMat.sumPlan("y", Y);

                // using Z == P given P
                tZP.C   += SubMat.sumPlan("z", Z);
                tZP.D   += iSumSubMat - SubMat.sumPlan("z", Z);
            }


        }
        //END FOR EACH SUBPRONOM


        // SUM of the SCORE of each PROPERTY
        scoreTest   +=  weightTriple*tXYZP.getScoreSum(true); // X given Y Z and P
    
        scoreTest   +=  weightDouble*tXY.getScoreSum(X==Y); // X == Y given P
        scoreTest   +=  weightDouble*tXZ.getScoreSum(X==Z); // X == Z given P
        scoreTest   +=  weightDouble*tYZ.getScoreSum(Y==Z); // Y == Z given P
    
        scoreTest   +=  weightSimple*tXP.getScoreSum(X==P); // X == P given P
        scoreTest   +=  weightSimple*tYP.getScoreSum(Y==P); // Y == P given P
        scoreTest   +=  weightSimple*tZP.getScoreSum(Z==P); // Z == P given P

        cout << endl << "addressee : " << Y << endl;
        cout << "XYZP  pv : " << tXYZP.chiSquare() << "\t A=" << tXYZP.A << "\t B=" << tXYZP.B << "\t C=" << tXYZP.C << "\t D=" << tXYZP.D << "\t score : " <<  weightTriple*tXYZP.getScoreSum(true) <<  endl;
        cout << "XY    pv : " << tXY.chiSquare()  << "\t A=" << tXY.A  << "\t B=" << tXY.B  << "\t C=" << tXY.C << "\t D=" << tXY.D << "\t score : " <<         weightDouble*tXY.getScoreSum(X==Y) <<  endl;
        cout << "XZ    pv : " << tXZ.chiSquare()  << "\t A=" << tXZ.A  << "\t B=" << tXZ.B  << "\t C=" << tXZ.C << "\t D=" << tXZ.D << "\t score : " <<         weightDouble*tXZ.getScoreSum(X==Z) <<  endl;
        cout << "YZ    pv : " << tYZ.chiSquare()  << "\t A=" << tYZ.A  << "\t B=" << tYZ.B  << "\t C=" << tYZ.C << "\t D=" << tYZ.D << "\t score : " <<         weightDouble*tYZ.getScoreSum(Y==Z) <<  endl;
        cout << "XP    pv : " << tXP.chiSquare()  << "\t A=" << tXP.A  << "\t B=" << tXP.B  << "\t C=" << tXP.C << "\t D=" << tXP.D << "\t score : " <<         weightSimple*tXP.getScoreSum(X==P) <<  endl;
        cout << "YP    pv : " << tYP.chiSquare()  << "\t A=" << tYP.A  << "\t B=" << tYP.B  << "\t C=" << tYP.C << "\t D=" << tYP.D << "\t score : " <<         weightSimple*tYP.getScoreSum(Y==P) <<  endl;
        cout << "ZP    pv : " << tZP.chiSquare()  << "\t A=" << tZP.A  << "\t B=" << tZP.B  << "\t C=" << tZP.C << "\t D=" << tZP.D << "\t score : " <<         weightSimple*tZP.getScoreSum(Z==P) <<  endl;

        pair<string, double>    pTemp(Y, scoreTest);

        vScore.push_back(pTemp);
    }

    cout << endl;


    double dScoreMax = -100000;
    string sResult;
    cout << "Hesitate between Product : "  << endl;
    for (vector<pair<string, double > >::iterator it_SubSc = vScore.begin() ; it_SubSc != vScore.end() ; it_SubSc++)
    {
        double dScoreTemp = it_SubSc->second;
        cout << "\t" << it_SubSc->first << "\t score = " << dScoreTemp << endl;
        if (dScoreTemp > dScoreMax)
        {
            dScoreMax = dScoreTemp;
            sResult = it_SubSc->first;
        }
    }

    if (dScoreMax <= 0)
    {
        cout << "Can't take a decision on which ad to select" << endl;
        sResult = "none";
    }

    cout << endl << "Final choice : " << sResult << "\t score : " << dScoreMax << endl;
    pReturn.first = sResult;
    pReturn.second = dScoreMax;

    return pReturn;
}

pair<string, double> grammarKnowledge::findSpeaker(string Y, string Z, string P)
{
    pair<string, double>    pReturn("none", 0);

    /*
    P is Subject (pronom)
    X is Speaker
    Y is Addressee
    Z is Agent
    */

    pronom currentPronom;

    bool fPronom = false;
    // get the pronom :
    for (vector<pronom>::iterator itPronom = listPronom.begin() ; itPronom != listPronom.end() ; itPronom++)
    {
        itPronom->m3Data.addLabel(Y);
        itPronom->m3Data.addLabel(Z);
        if (itPronom->sSubject == P)
        {
            fPronom = true;
            currentPronom = *itPronom;
        }
    }

    vector<pair<string, double> >   vScore;

    // if the pronom is known : check the properties
    if (!fPronom)
    {
        pronom newPronom;
        for (vector<string>::iterator itLabel = listPronom.begin()->m3Data.vsLabels.begin() ; itLabel < listPronom.begin()->m3Data.vsLabels.end() ; itLabel++)
        {
            newPronom.m3Data.addLabel(*itLabel);
        }

        newPronom.sSubject = P;
        listPronom.push_back(newPronom);

        currentPronom =  listPronom.back();
    }

    matrix3D currentData = currentPronom.m3Data;
    int sumMat = currentData.iSum;

    for (vector<string>::iterator itSpeaker = currentData.vsLabels.begin() ; itSpeaker != currentData.vsLabels.end() ; itSpeaker++)
    {
        string X = *itSpeaker;

        scoreProp   tXYZP; // X given Y Z and P

        scoreProp   tXY;    // X == Y given P
        scoreProp   tXZ;    // X == Z given P
        scoreProp   tYZ;    // Y == Z given P

        scoreProp   tXP;    // X == P given P
        scoreProp   tYP;    // Y == P given P
        scoreProp   tZP;    // Z == P given P

        double scoreTest = 0;

        // X Y Z P 
        tXYZP.A = currentData.get(X,Y,Z);
        tXYZP.C = currentData.sumLineYZ(Y,Z) - tXYZP.A;

        /*-------------*/

        // using X == Y given P
        tXY.A   = currentData.sumDiagDouble("z");
        tXY.B   = sumMat - tXY.A;

        // using X == Z given P
        tXZ.A   = currentData.sumDiagDouble("y");
        tXZ.B   = sumMat - tXZ.A;

        // using Y == Z given P
        tYZ.A   = currentData.sumDiagDouble("x");
        tYZ.B   = sumMat - tYZ.A;

        /*-------------*/

        // using X == P given P
        tXP.A   = currentData.sumPlan("x", X);
        tXP.B   = sumMat - tXP.A;

        // using Y == P given P
        tYP.A   = currentData.sumPlan("y", Y);
        tYP.B   = sumMat - tYP.A;

        // using Z == P given P
        tZP.A   = currentData.sumPlan("z", Z);
        tZP.B   = sumMat - tZP.A;

        // FOR EACH SUB PRONOM
        for (vector<pronom>::iterator itSubPronom = listPronom.begin() ; itSubPronom != listPronom.end() ; itSubPronom++)
        {
            matrix3D SubMat = itSubPronom->m3Data;
            int iSumSubMat = SubMat.getSum();

            // IF PRONOM != SUBPRONOM
            if (itSubPronom->sSubject != P)
            {
                // using X == Y given P
                tXY.C   += SubMat.sumDiagDouble("z");
                tXY.D   += iSumSubMat - SubMat.sumDiagDouble("z");

                // using X == Z given P
                tXZ.C   += SubMat.sumDiagDouble("y");
                tXZ.D   += iSumSubMat - SubMat.sumDiagDouble("y");

                // using Y == Z given P
                tYZ.C   += SubMat.sumDiagDouble("x");
                tYZ.D   += iSumSubMat - SubMat.sumDiagDouble("x");

                /*-------------*/

                // using X == P given P
                tXP.C   += SubMat.sumPlan("x", X);
                tXP.D   += iSumSubMat - SubMat.sumPlan("x", X);

                // using Y == P given P
                tYP.C   += SubMat.sumPlan("y", Y);
                tYP.D   += iSumSubMat - SubMat.sumPlan("y", Y);

                // using Z == P given P
                tZP.C   += SubMat.sumPlan("z", Z);
                tZP.D   += iSumSubMat - SubMat.sumPlan("z", Z);
            }


        }
        //END FOR EACH SUBPRONOM


        // SUM of the SCORE of each PROPERTY
        scoreTest   +=  weightTriple*tXYZP.getScoreSum(true); // X given Y Z and P
    
        scoreTest   +=  weightDouble*tXY.getScoreSum(X==Y); // X == Y given P
        scoreTest   +=  weightDouble*tXZ.getScoreSum(X==Z); // X == Z given P
        scoreTest   +=  weightDouble*tYZ.getScoreSum(Y==Z); // Y == Z given P

        scoreTest   +=  weightSimple*tXP.getScoreSum(X==P); // X == P given P
        scoreTest   +=  weightSimple*tYP.getScoreSum(Y==P); // Y == P given P
        scoreTest   +=  weightSimple*tZP.getScoreSum(Z==P); // Z == P given P

        cout << endl << "speaker : " << X << endl;
        cout << "XYZP  pv : " << tXYZP.chiSquare() << "\t A=" << tXYZP.A << "\t B=" << tXYZP.B << "\t C=" << tXYZP.C << "\t D=" << tXYZP.D << "\t score : " <<  weightTriple*tXYZP.getScoreSum(true) <<  endl;
        cout << "XY    pv : " << tXY.chiSquare()  << "\t A=" << tXY.A  << "\t B=" << tXY.B  << "\t C=" << tXY.C << "\t D=" << tXY.D << "\t score : " <<         weightDouble*tXY.getScoreSum(X==Y) <<  endl;
        cout << "XZ    pv : " << tXZ.chiSquare()  << "\t A=" << tXZ.A  << "\t B=" << tXZ.B  << "\t C=" << tXZ.C << "\t D=" << tXZ.D << "\t score : " <<         weightDouble*tXZ.getScoreSum(X==Z) <<  endl;
        cout << "YZ    pv : " << tYZ.chiSquare()  << "\t A=" << tYZ.A  << "\t B=" << tYZ.B  << "\t C=" << tYZ.C << "\t D=" << tYZ.D << "\t score : " <<         weightDouble*tYZ.getScoreSum(Y==Z) <<  endl;
        cout << "XP    pv : " << tXP.chiSquare()  << "\t A=" << tXP.A  << "\t B=" << tXP.B  << "\t C=" << tXP.C << "\t D=" << tXP.D << "\t score : " <<         weightSimple*tXP.getScoreSum(X==P) <<  endl;
        cout << "YP    pv : " << tYP.chiSquare()  << "\t A=" << tYP.A  << "\t B=" << tYP.B  << "\t C=" << tYP.C << "\t D=" << tYP.D << "\t score : " <<         weightSimple*tYP.getScoreSum(Y==P) <<  endl;
        cout << "ZP    pv : " << tZP.chiSquare()  << "\t A=" << tZP.A  << "\t B=" << tZP.B  << "\t C=" << tZP.C << "\t D=" << tZP.D << "\t score : " <<         weightSimple*tZP.getScoreSum(Z==P) <<  endl;

        pair<string, double>    pTemp(X, scoreTest);

        vScore.push_back(pTemp);
    }

    cout << endl;


    double dScoreMax = -100000;
    string sResult;
    cout << "Hesitate between Product : "  << endl;
    for (vector<pair<string, double > >::iterator it_SubSc = vScore.begin() ; it_SubSc != vScore.end() ; it_SubSc++)
    {
        double dScoreTemp = it_SubSc->second;
        cout << "\t" << it_SubSc->first << "\t score = " << dScoreTemp << endl;
        if (dScoreTemp > dScoreMax)
        {
            dScoreMax = dScoreTemp;
            sResult = it_SubSc->first;
        }
    }

    if (dScoreMax <= 0)
    {
        cout << "Can't take a decision on which sp to select" << endl;
        sResult = "none";
    }

    cout << endl << "Final choice : " << sResult << "\t score : " << dScoreMax << endl;
    pReturn.first = sResult;
    pReturn.second = dScoreMax;

    return pReturn;
}

pair<string, double> grammarKnowledge::findSubject(string X, string Y, string Z)
{
    double SIGMA_LEARNING_GRAMMAR = 1. * abmReasoningFunction::SIGMA_LEARNING_GRAMMAR;

    int iPropertiesTakenIntoAccount;
    pair<string, double>    pReturn;
    vector<pair<string, double> >   vScore;

    /*
    P is Subject (pronom)
    X is Speaker
    Y is Addressee
    Z is Agent
    */

    // FOR EACH PRONOM, TEST THE 7 PROPERTIES + if PRONOM = X, Y or Z
    for (vector<pronom>::iterator itPronom = listPronom.begin() ; itPronom != listPronom.end() ; itPronom++)
    {
        itPronom->m3Data.addLabel(X);
        itPronom->m3Data.addLabel(Y);
        itPronom->m3Data.addLabel(Z);

        string P = itPronom->sSubject;

        scoreProp   tXYZ(0,0,0,0);      // what is the preferential pronom for X Y and Z

        scoreProp   tXY(0,0,0,0);       // what is the preferential pronom for X and Y fixed, and Z variable
        scoreProp   tXZ(0,0,0,0);       // etc ...
        scoreProp   tYZ(0,0,0,0);

        scoreProp   tX(0,0,0,0);
        scoreProp   tY(0,0,0,0);
        scoreProp   tZ(0,0,0,0);

        scoreProp   tPX(0,0,0,0);
        scoreProp   tPY(0,0,0,0);
        scoreProp   tPZ(0,0,0,0);

        double scoreTest = 0;
        iPropertiesTakenIntoAccount = 0;
        matrix3D    currentMatrix = itPronom->m3Data;
        int iSumCurrentMatrix = currentMatrix.getSum();

        // X Y Z
        tXYZ.A = currentMatrix.get(X, Y, Z);
        tXYZ.C = currentMatrix.getSum() - currentMatrix.get(X, Y, Z);

        // X Y
        tXY.A += currentMatrix.sumDiagDouble("z");
        tXY.C += iSumCurrentMatrix - tXY.A;

        // X Z
        tXZ.A += currentMatrix.sumDiagDouble("y");
        tXZ.C += iSumCurrentMatrix - tXZ.A;

        // Y Z
        tYZ.A += currentMatrix.sumDiagDouble("x");
        tYZ.C += iSumCurrentMatrix - tYZ.A;

        // X 
        tX.A += currentMatrix.sumPlan("x", X);
        tX.C += iSumCurrentMatrix - tX.A;

        // Y 
        tY.A += currentMatrix.sumPlan("y", Y);
        tY.C += iSumCurrentMatrix - tY.A;

        // Z 
        tZ.A += currentMatrix.sumPlan("z", Z);
        tZ.C += iSumCurrentMatrix - tZ.A;

        // FOR EACH SUB PRONOM
        for (vector<pronom>::iterator itSubPronom = listPronom.begin() ; itSubPronom != listPronom.end() ; itSubPronom++)
        {

            bool bIsSame = false;

            // IF PRONOM != SUBPRONOM
            if (itSubPronom->sSubject != itPronom->sSubject)
            {
                // X Y Z
                tXYZ.B += itSubPronom->m3Data.get(X, Y, Z);
                tXYZ.D += itSubPronom->m3Data.getSum() - itPronom->m3Data.get(X, Y, Z);

                // X Y
                tXY.B += itSubPronom->m3Data.sumDiagDouble("z");
                tXY.D += itSubPronom->m3Data.getSum() - itSubPronom->m3Data.sumDiagDouble("z");

                // X Z
                tXZ.B += itSubPronom->m3Data.sumDiagDouble("y");
                tXZ.D += itSubPronom->m3Data.getSum() - itSubPronom->m3Data.sumDiagDouble("y");

                // Y Z
                tYZ.B += itSubPronom->m3Data.sumDiagDouble("x");
                tYZ.D += itSubPronom->m3Data.getSum() - itSubPronom->m3Data.sumDiagDouble("x");

                // X 
                tX.B += itSubPronom->m3Data.sumPlan("x", X);
                tX.D += itSubPronom->m3Data.getSum() - itSubPronom->m3Data.sumPlan("x", X);

                // Y 
                tY.B += itSubPronom->m3Data.sumPlan("y", Y);
                tY.D += itSubPronom->m3Data.getSum() - itSubPronom->m3Data.sumPlan("y", Y);

                // Z 
                tZ.B += itSubPronom->m3Data.sumPlan("z", Z);
                tZ.D += itSubPronom->m3Data.getSum() - itSubPronom->m3Data.sumPlan("z", Z);
            }
            //END IF PRONOM != SUBPRONOM

            // SPEAKER IS SUBJECT
            tPX.addScore(itSubPronom->pSubject_Is_Speaker, bIsSame);

            // ADDRESSEE IS SUBJECT
            tPY.addScore(itSubPronom->pSubject_Is_Addressee, bIsSame);

            // AGENT IS SUBJECT
            tPZ.addScore(itSubPronom->pSubject_Is_Agent, bIsSame);
        }
        //END FOR EACH SUBPRONOM


        // SUM of the SCORE of each PROPERTY

        scoreTest += weightTriple*tXYZ.getScoreSum();   

        scoreTest += weightDouble*tXY.getScoreSum(X==Y);
        scoreTest += weightDouble*tXZ.getScoreSum(X==Z);
        scoreTest += weightDouble*tYZ.getScoreSum(Y==Z);

        scoreTest += weightSimple*tX.getScoreSum();
        scoreTest += weightSimple*tY.getScoreSum();
        scoreTest += weightSimple*tZ.getScoreSum();
        
        cout << endl << "Sub : " << P << endl;
        cout << "XYZP  pv : " << tXYZ.chiSquare() << "\t A=" << tXYZ.A << "\t B=" << tXYZ.B << "\t C=" << tXYZ.C << "\t D=" << tXYZ.D << "\t score : " <<   weightTriple*tXYZ.getScoreSum() <<  endl;
        cout << "XY    pv : " << tXY.chiSquare()  << "\t A=" << tXY.A  << "\t B=" << tXY.B  << "\t C=" << tXY.C << "\t D=" << tXY.D << "\t score : " <<     weightDouble*tXY.getScoreSum(X==Y) <<  endl;
        cout << "XZ    pv : " << tXZ.chiSquare()  << "\t A=" << tXZ.A  << "\t B=" << tXZ.B  << "\t C=" << tXZ.C << "\t D=" << tXZ.D << "\t score : " <<     weightDouble*tXZ.getScoreSum(X==Z) <<  endl;
        cout << "YZ    pv : " << tYZ.chiSquare()  << "\t A=" << tYZ.A  << "\t B=" << tYZ.B  << "\t C=" << tYZ.C << "\t D=" << tYZ.D << "\t score : " <<     weightDouble*tYZ.getScoreSum(Y==Z) <<  endl;
        cout << "X     pv : " << tX.chiSquare()   << "\t A=" << tX.A   << "\t B=" << tX.B   << "\t C=" << tX.C  << "\t D=" << tX.D  << "\t score : " <<     weightSimple*tX.getScoreSum() <<  endl;
        cout << "Y     pv : " << tY.chiSquare()   << "\t A=" << tY.A   << "\t B=" << tY.B   << "\t C=" << tY.C  << "\t D=" << tY.D  << "\t score : " <<     weightSimple*tY.getScoreSum() <<  endl;
        cout << "Z     pv : " << tZ.chiSquare()   << "\t A=" << tZ.A   << "\t B=" << tZ.B   << "\t C=" << tZ.C  << "\t D=" << tZ.D  << "\t score : " <<     weightSimple*tZ.getScoreSum() <<  endl;

        pair<string, double>    pTemp(itPronom->sSubject, scoreTest);

        vScore.push_back(pTemp);
    }
    // ENF FOR EACH PRONOM


    double dScoreMax = -100000;
    string sResult;
    cout << "speaker \t " << X << endl << "addressee \t " << Y << endl << "agent \t\t " << Z << endl ;
    cout << "Hesitate between Product : "  << endl;
    for (vector<pair<string, double > >::iterator it_SubSc = vScore.begin() ; it_SubSc != vScore.end() ; it_SubSc++)
    {
        double dScoreTemp = it_SubSc->second;
        cout << "\t" << it_SubSc->first << "\t score = " << dScoreTemp << endl;
        if (dScoreTemp > dScoreMax)
        {
            dScoreMax = dScoreTemp;
            sResult = it_SubSc->first;
        }
    }

    // if no pronom has been selected, test if X Y or Z could be pronom
    if (dScoreMax <= 0)
    {
        cout << "Can't take a decision on which pronom to select" << endl;
        sResult = "none";
    }

    cout << endl << "Final choice : " << sResult << "\t score : " << dScoreMax << endl;
    pReturn.first = sResult;
    pReturn.second = dScoreMax;

    return pReturn;
}


bool grammarKnowledge::addInteraction(Bottle bSentence)
{
    string  sSpeaker    = bSentence.get(0).toString().c_str(),
        sAddressee  = bSentence.get(1).toString().c_str(),
        sSubject    = bSentence.get(2).toString().c_str(),
        sAgent      = bSentence.get(3).toString().c_str();

    ofstream fileOut("C:/Users/rclab/Desktop/res/dataLearning.csv",ios::out | ios::app);

    if (fileOut)
    {   
        fileOut << sSubject << ";" << sSpeaker << ";" << sAddressee << ";" << sAgent  << endl;
    }


    bool bFound = false;

    addLabel(sSpeaker);
    addLabel(sAddressee);
    addLabel(sAgent);

    for (vector<pronom>::iterator it_GK = listPronom.begin() ; it_GK != listPronom.end() ; it_GK++)
    {
        if (!bFound && it_GK->sSubject == sSubject)
        {
            bFound = true;
            it_GK->AddInstance(bSentence);
        }
    }

    // if the pronom doesn't exist yet
    if (!bFound)
    {
        pronom newPR;
        newPR.sSubject = sSubject;
        newPR.AddInstance(bSentence);

        listPronom.push_back(newPR);
    }

    bFound = false;
    for (vector<string>::iterator itStr = listAddressee.begin() ; itStr != listAddressee.end() ; itStr++)
    {
        if (!bFound && *itStr == sAddressee) bFound = true;
    }
    if (!bFound) listAddressee.push_back(sAddressee);

    bFound = false;
    for (vector<string>::iterator itStr = listAgent.begin() ; itStr != listAgent.end() ; itStr++)
    {
        if (!bFound && *itStr == sAgent) bFound = true;
    }
    if (!bFound) listAgent.push_back(sAgent);

    bFound = false;
    for (vector<string>::iterator itStr = listSpeaker.begin() ; itStr != listSpeaker.end() ; itStr++)
    {
        if (!bFound && *itStr == sSpeaker) bFound = true;
    }
    if (!bFound) listSpeaker.push_back(sSpeaker);


    return bFound;
}

void grammarKnowledge::updateLabel()
{
    for (vector<string>::iterator itLabel = listAgent.begin() ; itLabel != listAgent.end() ; itLabel++)
    {
        for (vector<pronom>::iterator itPr = listPronom.begin() ; itPr != listPronom.end() ; itPr++)
        {
            itPr->m3Data.addLabel(*itLabel);
        }
    }
}

void grammarKnowledge::addLabel(string sLabel)
{
    bool bFound = false;

    for (vector<string>::iterator itLabel = listLabel.begin() ; itLabel != listLabel.end() ; itLabel++)
    {
        if (*itLabel == sLabel)     bFound = true;
    }

    if (!bFound)
    {
        listLabel.push_back(sLabel);
        updateLabel();
    }
}


void grammarKnowledge::testModel(int iInstances, int condition)
{

    vector<string> vsLabelToAdd;

    //vsLabelToAdd.push_back("John");
    //vsLabelToAdd.push_back("iCub");

    cout << "Starting test of model" << endl << endl;
    cout << "Starting test of Sp" << endl;
    testSp(iInstances, vsLabelToAdd, condition);
    cout << "End test of Sp" << endl;
    cout << "Starting test of Ad" << endl;
    testAd(iInstances, vsLabelToAdd, condition);
    cout << "End test of Ad" << endl;
    cout << "Starting test of Su" << endl;
    testSu(iInstances, vsLabelToAdd, condition);
    cout << "End test of Su" << endl;
    cout << "Starting test of Ag" << endl;
    testAg(iInstances, vsLabelToAdd, condition);
    cout << "End test of Ag" << endl;
    cout << endl << "End test of model" << endl << endl;

}


void grammarKnowledge::testSp(int iInstances, vector<string> vsLabelToAdd, int condition)
{
    vector<string> vsLabel;
    vsLabel = listLabel;
    for (vector<string>::iterator itL = vsLabelToAdd.begin(); itL != vsLabelToAdd.end() ; itL++)
    {
        vsLabel.push_back(*itL);
    }

    ostringstream osFileOut;
    osFileOut << "C:/Users/rclab/Desktop/res/" << condition << "resultSimSP.csv";
    string sFileOut = osFileOut.str();
    ofstream fileOut(sFileOut,ios::out | ios::trunc);

    vector<string> vsSubjects;
    vsSubjects = vsLabel;
    vsSubjects.push_back("I");
    vsSubjects.push_back("You");

    string Su, Ad, Ag, Sp;
    double dScore;


    if (fileOut)
    {   
        fileOut << "Subject;Speaker;Addressee;Agent;Score"<<endl;
    }
    else
    {
        cout << "Couldn't open file" << endl;
    }


    for (int i = 0 ; i < iInstances ; i++)
    {
        Su = vsSubjects[rand()%vsSubjects.size()];
        bool AgCheck = false;
        bool AdCheck = false;

        while (!AdCheck)
        {
            Ad = vsLabel[rand()%vsLabel.size()];
            if (Su !="I" && Su != "You")
            {
                if (Ad != Su)   AdCheck = true;
            }
            else
            {
                AdCheck = true;
            }
        }

        while (!AgCheck)
        {
            Ag = vsLabel[rand()%vsLabel.size()];

            if (Su == "I")
            {
                if (Ad != Ag)   AgCheck = true;
            }
            else if (Su == "You")
            {
                if (Ad == Ag)   AgCheck = true;
            }
            else
            {
                if (Ag == Su)   AgCheck = true;
            }
        }

        pair<string, double> res = findSpeaker(Ad,Ag,Su);
        dScore = res.second;
        Sp = res.first;

        if (fileOut)
        {   
            fileOut << Su << ";" << Sp << ";" << Ad << ";" << Ag << ";" << dScore << endl;
        }
    }
}

void grammarKnowledge::testAd(int iInstances, vector<string> vsLabelToAdd, int condition)
{
    vector<string> vsLabel;
    vsLabel = listLabel;
    for (vector<string>::iterator itL = vsLabelToAdd.begin(); itL != vsLabelToAdd.end() ; itL++)
    {
        vsLabel.push_back(*itL);
    }

    vector<string> vsSubjects;
    vsSubjects = vsLabel;
    vsSubjects.push_back("I");
    vsSubjects.push_back("You");

    string Su, Ad, Ag, Sp;
    double dScore;
    
    ostringstream osFileOut;
    osFileOut << "C:/Users/rclab/Desktop/res/" << condition << "resultSimAD.csv";
    string sFileOut = osFileOut.str();
    ofstream fileOut(sFileOut,ios::out | ios::trunc);

    if (fileOut)
    {   
        fileOut << "Subject;Speaker;Addressee;Agent;Score"<<endl;
    }
    else
    {
        cout << "Couldn't open file" << endl;
    }


    for (int i = 0 ; i < iInstances ; i++)
    {
        Su = vsSubjects[rand()%vsSubjects.size()];
        bool SpCheck = false;
        bool AgCheck = false;

        while (!AgCheck)
        {
            Ag = vsLabel[rand()%vsLabel.size()];

            if (Su !="I" && Su != "You")
            {
                if (Ag == Su)   AgCheck = true;
            }
            else
            {
                AgCheck = true;
            }
        }


        while (!SpCheck)
        {
            Sp = vsLabel[rand()%vsLabel.size()];

            if (Su == "I")
            {
                if (Sp == Ag)   SpCheck = true;
            }
            else
            {
                if (Sp != Ag)   SpCheck = true;
            }
        }

        pair<string, double> res = findAddressee(Sp, Ag, Su);
        dScore = res.second;
        Ad = res.first;

        if (fileOut)
        {   
            fileOut << Su << ";" << Sp << ";" << Ad << ";" << Ag << ";" << dScore << endl;
        }
    }

}

void grammarKnowledge::testAg(int iInstances, vector<string> vsLabelToAdd, int condition)
{
    vector<string> vsLabel;
    vsLabel = listLabel;
    for (vector<string>::iterator itL = vsLabelToAdd.begin(); itL != vsLabelToAdd.end() ; itL++)
    {
        vsLabel.push_back(*itL);
    }

    vector<string> vsSubjects;
    if (vsLabel.size() != 2)    vsSubjects = vsLabel;
    vsSubjects.push_back("I");
    vsSubjects.push_back("You");

    string Su, Ad, Ag, Sp;
    double dScore;

    ostringstream osFileOut;
    osFileOut << "C:/Users/rclab/Desktop/res/" << condition << "resultSimAG.csv";
    string sFileOut = osFileOut.str();
    ofstream fileOut(sFileOut,ios::out | ios::trunc);

    if (fileOut)
    {   
        fileOut << "Subject;Speaker;Addressee;Agent;Score"<<endl;
    }
    else
    {
        cout << "Couldn't open file" << endl;
    }


    for (int i = 0 ; i < iInstances ; i++)
    {
        Su = vsSubjects[rand()%vsSubjects.size()];
        bool SpCheck = false;
        bool AdCheck = false;

        while (!AdCheck)
        {
            Ad = vsLabel[rand()%vsLabel.size()];
            if (Ad != Su)   AdCheck = true;
        }

        while (!SpCheck)
        {
            Sp = vsLabel[rand()%vsLabel.size()];

            if (Sp != Ad && Sp !=Su )
            {
                SpCheck = true;
            }
        }

        pair<string, double> res = findAgent(Sp, Ad, Su);
        dScore = res.second;
        Ag = res.first;

        if (fileOut)
        {   
            fileOut << Su << ";" << Sp << ";" << Ad << ";" << Ag << ";" << dScore << endl;
        }
    }

}

void grammarKnowledge::testSu(int iInstances, vector<string> vsLabelToAdd, int condition)
{
    vector<string> vsLabel;
    vsLabel = listLabel;
    for (vector<string>::iterator itL = vsLabelToAdd.begin(); itL != vsLabelToAdd.end() ; itL++)
    {
        vsLabel.push_back(*itL);
    }

    string Su, Ad, Ag, Sp;
    double dScore;
    
    ostringstream osFileOut;
    osFileOut << "C:/Users/rclab/Desktop/res/" << condition << "resultSimSU.csv";
    string sFileOut = osFileOut.str();
    ofstream fileOut(sFileOut,ios::out | ios::trunc);

    if (fileOut)
    {   
        fileOut << "Subject;Speaker;Addressee;Agent;Score"<<endl;
    }
    else
    {
        cout << "Couldn't open file" << endl;
    }


    for (int i = 0 ; i < iInstances ; i++)
    {
        Sp = vsLabel[rand()%vsLabel.size()];

        bool AdCheck = false;

        while (!AdCheck)
        {
            Ad = vsLabel[rand()%vsLabel.size()];
            if (Sp != Ad)   AdCheck = true;
        }

        Ag = vsLabel[rand()%vsLabel.size()];

        pair<string, double> res = findSubject(Sp, Ad, Ag);
        dScore = res.second;
        Su = res.first;

        if (fileOut)
        {   
            fileOut << Su << ";" << Sp << ";" << Ad << ";" << Ag << ";" << dScore << endl;
        }
    }
}

void    grammarKnowledge::simulateLearning(int Case,int iNbRep)
{
    cout << "Starting learning simulation" << endl;

    if (Case == 7)
    {
        // autistic N2
        cout <<"Learning with Peter talking, iCub listening" << endl;

        simulateInstance("Peter", "iCub", "Peter", "I", iNbRep);
        simulateInstance("Peter", "iCub", "iCub", "You", iNbRep);
    }
    else if (Case == 1)
    {
        // diadic spect
        simulateInstance("Peter", "Maxime", "Peter", "I", iNbRep);
        simulateInstance("Maxime", "Peter", "Maxime", "I", iNbRep);
        simulateInstance("Peter", "Maxime", "Maxime", "You", iNbRep);
        simulateInstance("Maxime", "Peter", "Peter", "You", iNbRep);
    }
    else if (Case == 5)
    {
        // blind N2
        simulateInstance("Peter", "iCub", "iCub", "You", iNbRep);
    }
    else if (Case == 4)
    {
        // blind N3
        simulateInstance("Peter", "iCub", "iCub", "You", iNbRep);
        simulateInstance("Maxime", "iCub", "iCub", "You", iNbRep);
        simulateInstance("Maxime", "Peter", "iCub", "iCub", iNbRep);
        simulateInstance("Peter", "Maxime", "iCub", "iCub", iNbRep);
    }
    else if (Case == 6)
    {
        // autistic N3
        simulateInstance("Peter", "iCub", "iCub", "You", iNbRep);
        simulateInstance("Maxime", "iCub", "iCub", "You", iNbRep);
        simulateInstance("Maxime", "iCub", "Maxime", "I", iNbRep);
        simulateInstance("Peter", "iCub", "Peter", "I", iNbRep);
        simulateInstance("Peter", "iCub", "Maxime", "Maxime", iNbRep);
        simulateInstance("Maxime", "iCub", "Peter", "Peter", iNbRep);
    }
    else if (Case == 3)
    {
        // triadic act
        simulateInstance("Peter", "Maxime", "Maxime", "You", iNbRep);
        simulateInstance("Maxime", "Peter", "Peter", "You", iNbRep);
        simulateInstance("Maxime", "Peter", "Maxime", "I", iNbRep);
        simulateInstance("Peter", "Maxime", "Peter", "I", iNbRep);
        simulateInstance("Peter", "iCub", "iCub", "You", iNbRep);
        simulateInstance("Maxime", "iCub", "iCub", "You", iNbRep);
        simulateInstance("Maxime", "iCub", "Maxime", "I", iNbRep);
        simulateInstance("Peter", "iCub", "Peter", "I", iNbRep);
        simulateInstance("Peter", "iCub", "Maxime", "Maxime", iNbRep);
        simulateInstance("Maxime", "iCub", "Peter", "Peter", iNbRep);
        simulateInstance("Peter", "Maxime", "iCub", "iCub", iNbRep);
        simulateInstance("Maxime", "Peter", "iCub", "iCub", iNbRep);
    }
}

void    grammarKnowledge::simulateInstance(string Sp, string Ad, string Ag, string Su, int iNbRep)
{

    Bottle bMessenger;
    bMessenger.clear();

    bMessenger.addString(Sp);
    bMessenger.addString(Ad);
    bMessenger.addString(Su);
    bMessenger.addString(Ag);
    for (int i = 0 ; i < iNbRep ; i++)
    {
        addInteraction(bMessenger);
    }
}

grammarKnowledge::grammarKnowledge()
{
    ofstream fileOut("C:/Users/rclab/Desktop/res/dataLearning.csv",ios::out | ios::trunc);

    if (fileOut)
    {   
        fileOut << "Subject;Speaker;Addressee;Agent"<<endl;
    }

}

void    grammarKnowledge::clear()
{
    listPronom.clear();
    listAddressee.clear();
    listAgent.clear();
    listSpeaker.clear();
    listLabel.clear();
}