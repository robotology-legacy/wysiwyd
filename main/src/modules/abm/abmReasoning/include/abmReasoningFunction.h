#ifndef _ABMREASONINGFUNCTION_H_
#define _ABMREASONINGFUNCTION_H_

#include <yarp/os/all.h>
#include "wrdac/clients/icubClient.h"
#include <numeric>
#include <tuple>
#include <time.h>


using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;


const double PI = 3.141592654;


class abmReasoningFunction
{
public:

	abmReasoningFunction(ResourceFinder &rf);

	static string s_realOPC ;               // name of the real OPC
	static string s_mentalOPC;              // name of the mental OPC

	//table
	static double   X_center;               // X of the center of the table
	static double   Y_center;               // Y of the center of the table
	static double   X_origin;               // X Origin of the reference of the table
	static double   Y_origin;               // Y Origin of the reference of the table
	static double   table_radius;           // Distance from the user to the center of the table

	//mental
	static double threshold_time_sequence;       //threshold of a same sequence
	static double height_location;      // coordonate in Z of a location in the OPC
	static double size_location;                    // size in Z of a location

	static int  color_dream_R;                  // color of the dreamed object in the OPC
	static int  color_dream_G;
	static int  color_dream_B;

	static int  color_loc_R;                    // color of the locations in the OPC
	static int  color_loc_G;
	static int  color_loc_B;

	static int  difference_date_in_second;      // threshold return of second if 2 actions are at different dates

	static double lifetime_relation;                // life time of a relation about the objects in the OPC

	//Spatialisation
	static unsigned int  threshold_determine_Location;           // number of tries before determine if location
	static double factor_location;                  // factor of the size of a location : center +/- factor_location * std dev
	static double threshold_is_at_location;
	static double threshold_is_at_temporal_location;
	static double threshold_is_dispersion;          // is lower, then the dispersion of a cloud of point is "null".

	// PDDL
	static double threshold_intersect_sup;
	static double threshold_intersect_inf;
	static double threshold_presence;
	static double threshold_absence;

	//TAGS
	static string TAG_LOCATION;
	static string TAG_IS_AT_LOC;
	static string TAG_DEFAULT;
	static string TAG_SPEAKER;
	static string TAG_ADRESSEE;
	static string TAG_SUBJECT;
	static string TAG_AGENT;
	static string TAG_NONE;

	//DB
	static string TAG_DB_ACTION;
	static string TAG_DB_COMPLEX;
	static string TAG_DB_BEHAVIOR;
	static string TAG_DB_SHARED_PLAN;
	static string TAG_DB_ARGUMENT;
	static string TAG_DB_NONE;
	static string TAG_DB_MANNER;
	static string TAG_DB_UNKNOWN;

	//GK
	static string TAG_SPEAKER_IS_RECEIVER;
	static string TAG_ADRESSEE_IS_SPEAKER;
	static string TAG_SPEAKER_IS_AGENT;
	static string TAG_AGENT_IS_SPEAKER;
	static string TAG_AGENT_IS_RECEIVER;
	static string TAG_ADRESSEE_IS_AGENT;
	static int  SIGMA_LEARNING_GRAMMAR;
	static double   THRESHOLD_CONFIDENCE_GRAMMAR;



	static pair<double, double> coordFromString(string);

	static bool timeDiff(struct tm TM1, struct tm TM2);

	static struct tm string2Time(string sTime);

	static string time2string(struct tm Time);

	static int  timeDiffSecondFromString(string T1, string T2);

	static pair<string, string> ago2string(pair<int, string> pInput);

	static vector<double> getCovMatrix(vector<double> vX, vector<double> vY);

	static double getMahalaDist(vector<double> vX, vector<double> vY, pair<double, double> XY);

	static tuple<int,int,int> tupleIntFromString(string sInput);
	static tuple<double,double,double> tupleDoubleFromString(string sInput);

};


class matrix3D          // personnal class of 3D matrix (cubic)
	// X is the speaker
	// Y is the addressee
	// Z is the agent
{
protected:
	vector<int>     viData;         // data of the matrix


public:

	// Variables : 
	int             iSize;          // size of each size of the 3D matrix (cubic)
	int             iSum;
	vector<string>  vsLabels;       // label associated to each col/row/deepth

	//Constructor 
	matrix3D() {iSize = 0; iSum = 0;}

	// Functions
	int oneCoord(int x, int y, int z) {return (x+y*iSize+z*iSize*iSize);}       // return the 1D coordinate from a 3D coordinate    

	int get(int x, int y, int z) {return viData[oneCoord(x, y, z)];}            // get the x y z position in the matrix

	int get(string sSpeaker, string sAddressee, string sAgent)
	{
		addLabel(sSpeaker);
		addLabel(sAddressee);
		addLabel(sAgent);

		int X = -1,
			Y = -1,
			Z = -1;

		for (int i = 0 ; i < iSize ; i++)
		{
			if (vsLabels[i] == sSpeaker) X = i;
			if (vsLabels[i] == sAddressee) Y = i;
			if (vsLabels[i] == sAgent) Z = i;
		}

		// check 
		if (X==-1 || Y == -1 || Z == -1)
		{
			//cout << endl << "Error in abmReasoning::pronom.h::matrix3D::get(string, string, string) | One of the label is missing" << endl;
			return 0;
		}

		return viData[oneCoord(X, Y, Z)];
	}           // get the x y z position in the matrix

	void incr(int x, int y, int z) {viData[oneCoord(x, y, z)]++;}               // increment the x y z position of the matrix of 1

	void addLabel(string sLabel)    // add 1 to the x y and z size, and add the label to the list
	{
		//check if label already in the matrix

		bool bFound = false;
		for (vector<string>::iterator itS = vsLabels.begin() ; itS != vsLabels.end() ; itS++)
		{
			if (*itS == sLabel)     bFound = true;
		}

		if (bFound)
		{
			//cout << endl << "Error in abmReasoning::pronom.h::matrix3D::addLabel | Label already existing" << endl;
			return;
		}


		vsLabels.push_back(sLabel);

		vector<int> matTemp;
		for (int k = 0 ; k < iSize+1 ; k ++)
		{
			for (int j = 0 ; j < iSize+1 ; j++)
			{
				for (int i = 0 ; i < iSize+1 ; i++)
				{

					if (i == iSize || j == iSize || k == iSize)
					{
						matTemp.push_back(0);
					}
					else
					{
						matTemp.push_back(get(i,j,k));
					}
				}
			}
		}
		viData = matTemp;
		iSize++;
	}

	void incr(string sSpeaker, string sAddressee, string sAgent)
		// increment in the matrix for the use of a pronom with information about the sentence
	{
		// first check if the speaker, receiver and agent are known
		addLabel(sSpeaker);
		addLabel(sAddressee);
		addLabel(sAgent);

		int X = -1,
			Y = -1,
			Z = -1;

		for (int i = 0 ; i < iSize ; i++)
		{
			if (vsLabels[i] == sSpeaker) X = i;
			if (vsLabels[i] == sAddressee) Y = i;
			if (vsLabels[i] == sAgent) Z = i;
		}

		// check 
		if (X==-1 || Y == -1 || Z == -1)
		{
			//cout << endl << "Error in abmReasoning::pronom.h::matrix3D::incr(string, string, string) | One of the label is missing" << endl;
			return;
		}

		incr(X,Y,Z);
		iSum++;
	}

	int getSum()    {return iSum;}
	int getSize()   {return iSize;}

	/* get the sum of the diagonal of the correspondant plan (x, y or z) */
	int sumDiagDouble(string W)
	{
		int sum = 0;
		if (W != "x" && W != "y" && W != "z")
		{cout << endl << "Error in abmReasoning::pronom.h::matrix3D::sumDiag(string) | wrong coordinate ('x', 'y' or 'z')" << endl; }
		for (int b = 0 ; b < iSize ; b++)
		{
			for (int a = 0 ; a < iSize ; a++)
			{
				if (W =="x") sum+=get(a,b,b);
				if (W =="y") sum+=get(b,a,b);
				if (W =="z") sum+=get(b,b,a);
			}
		}
		return sum;
	}


	int sumPlan(string W, string sLabel)    // W is x y or z and Label is the name
	{
		int iLabel= -1;
		for (int i = 0 ; i < iSize ; i++)
		{
			if (vsLabels[i] == sLabel) iLabel = i;
		}

		int sum = 0;
		if (W != "x" && W != "y" && W != "z")
		{cout << endl << "Error in abmReasoning::pronom.h::matrix3D::sumPlan(string) | wrong coordinate ('x', 'y' or 'z')" << endl; }
		for (int a = 0 ; a < iSize ; a++)
		{
			for (int b = 0 ; b < iSize ; b++)
			{
				if (W =="x") sum+=get(iLabel,a,b);
				if (W =="y") sum+=get(a,iLabel,b);
				if (W =="z") sum+=get(a,b,iLabel);
			}
		}
		return sum;
	}

	/* For a given X and Y, return the sum of the Z line*/
	int sumLineXY(string X, string Y)
	{
		int xLabel= -1,
			yLabel = -1;
		for (int i = 0 ; i < iSize ; i++)
		{
			if (vsLabels[i] == X) xLabel = i;
			if (vsLabels[i] == Y) yLabel = i;
		}

		int sum = 0;
		for (int a = 0 ; a < iSize ; a++)
		{
			sum += get(xLabel, yLabel, a);
		}
		return sum;
	}

	/* For a given X and Z, return the sum of the Y line*/
	int sumLineXZ(string X, string Z)
	{
		int xLabel= -1,
			zLabel = -1;
		for (int i = 0 ; i < iSize ; i++)
		{
			if (vsLabels[i] == X) xLabel = i;
			if (vsLabels[i] == Z) zLabel = i;
		}

		int sum = 0;
		for (int a = 0 ; a < iSize ; a++)
		{
			sum += get(xLabel, a, zLabel);
		}
		return sum;
	}

	/* For a given Y and Z, return the sum of the X line*/
	int sumLineYZ(string Y, string Z)
	{
		int zLabel= -1,
			yLabel = -1;
		for (int i = 0 ; i < iSize ; i++)
		{
			if (vsLabels[i] == Z) zLabel = i;
			if (vsLabels[i] == Y) yLabel = i;
		}

		int sum = 0;
		for (int a = 0 ; a < iSize ; a++)
		{
			sum += get(a, yLabel, zLabel);
		}
		return sum;
	}
};



//----------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------      MATRIX 3D NON CUBIC     -----------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------


class matrix3D_nonCubic          // personnal class of 3D matrix
{
protected:
	vector<int>         viData;         // data of the matrix
	vector<string>      vLabelX;
	vector<string>      vLabelY;
	vector<string>      vLabelZ;

public:

	// Variables : 
	int             iSum;

	//Constructor 
	matrix3D_nonCubic() {iSum = 0;}

	// Functions
	int oneCoord(int x, int y, int z) {return (x+y*vLabelY.size()+z*vLabelZ.size()*vLabelZ.size());}       // return the 1D coordinate from a 3D coordinate    

	int get(int x, int y, int z) {return viData[oneCoord(x, y, z)];}            // get the x y z position in the matrix


	void incr(int x, int y, int z) {viData[oneCoord(x, y, z)]++;}               // increment the x y z position of the matrix of 1

	void addLabelX(string sLabel)    // add 1 to the x size, and add the label to the list
	{

		//check if label already in the matrix
		for (vector<string>::iterator it = vLabelX.begin(); it != vLabelX.end() ; it++)
		{
			if (sLabel == *it)
				return;
		}

		vector<int>     matrixTemp;

		for (unsigned int i = 0 ; i < vLabelZ.size() ; i++)
		{

			for (unsigned int j = 0 ; j < vLabelY.size() ; j++)
			{

				for (unsigned int k = 0 ; k < vLabelX.size() ; k++)
				{
					matrixTemp.push_back(get(i,j,k));
				}
				matrixTemp.push_back(0);
			}
		}

		vLabelX.push_back(sLabel);
		viData = matrixTemp;
	}

	void addLabelY(string sLabel)    // add 1 to the y size, and add the label to the list
	{

		//check if label already in the matrix
		for (vector<string>::iterator it = vLabelY.begin(); it != vLabelY.end() ; it++)
		{
			if (sLabel == *it)
				return;
		}

		vector<int>     matrixTemp;

		for (unsigned int i = 0 ; i < vLabelZ.size() ; i++)
		{
			for (unsigned int j = 0 ; j < vLabelY.size() ; j++)
			{
				for (unsigned int k = 0 ; k < vLabelX.size() ; k++)
				{
					matrixTemp.push_back(get(i,j,k));
				}
			}
			for (unsigned int k = 0 ; k < vLabelX.size() ; k++)
			{
				matrixTemp.push_back(0);
			}
		}

		vLabelY.push_back(sLabel);
		viData = matrixTemp;
	}

	void addLabelZ(string sLabel)    // add 1 to the z size, and add the label to the list
	{

		//check if label already in the matrix
		for (vector<string>::iterator it = vLabelZ.begin(); it != vLabelZ.end() ; it++)
		{
			if (sLabel == *it)
				return;
		}

		vector<int>     matrixTemp;

		for (unsigned int i = 0 ; i < vLabelZ.size() ; i++)
		{
			for (unsigned int j = 0 ; j < vLabelY.size() ; j++)
			{
				for (unsigned int k = 0 ; k < vLabelX.size() ; k++)
				{
					matrixTemp.push_back(get(i,j,k));
				}
			}
			for (unsigned int j = 0 ; j < vLabelY.size() ; j++)
			{
				for (unsigned int k = 0 ; k < vLabelX.size() ; k++)
				{
					matrixTemp.push_back(0);
				}
			}
		}

		vLabelZ.push_back(sLabel);
		viData = matrixTemp;
	}

	void incr(string sX, string sY, string sZ)
		// increment in the matrix for the use of a pronom with information about the sentence
	{
		// first check if the speaker, receiver and agent are known
		addLabelX(sX);
		addLabelY(sY);
		addLabelZ(sZ);

		int iX = -1,
			iY = -1,
			iZ = -1;

		// search for X
		for (unsigned int i = 0 ; i < vLabelX.size() ; i++)
		{
			if (vLabelX[i] == sX) iX = i;
		}

		// search for Y
		for (unsigned int i = 0 ; i < vLabelY.size() ; i++)
		{
			if (vLabelY[i] == sY) iY = i;
		}

		// search for Z
		for (unsigned int i = 0 ; i < vLabelZ.size() ; i++)
		{
			if (vLabelZ[i] == sZ) iZ = i;
		}

		// check 
		if (iX==-1 || iY == -1 || iZ == -1)
		{
			cout << endl << "Error in abmReasoning::pronom.h::matrix3D_nonCubic::incr(string, string, string) | One of the label is missing" << endl;
			return;
		}

		incr(iX,iY,iZ);
		iSum++;
	}

	/* For a given X and Y, return the sum of the Z line*/
	int sumLineXY(string sX, string sY)
	{
		int iX = -1,
			iY = -1;

		// search for X
		for (unsigned int i = 0 ; i < vLabelX.size() ; i++)
		{
			if (vLabelX[i] == sX) iX = i;
		}

		// search for Y
		for (unsigned int i = 0 ; i < vLabelY.size() ; i++)
		{
			if (vLabelY[i] == sY) iY = i;
		}

		int sum = 0;
		for (int a = 0 ; a < vLabelZ.size() ; a++)
		{
			sum += get(iX, iY, a);
		}
		return sum;
	}

	/* For a given X and Y, return the sum of the Z line*/
	int sumLineYZ(string sY, string sZ)
	{
		int iY = -1,
			iZ = -1;

		// search for X
		for (unsigned int i = 0 ; i < vLabelZ.size() ; i++)
		{
			if (vLabelZ[i] == sZ) iZ = i;
		}

		// search for Y
		for (unsigned int i = 0 ; i < vLabelY.size() ; i++)
		{
			if (vLabelY[i] == sY) iY = i;
		}

		int sum = 0;
		for (int a = 0 ; a < vLabelX.size() ; a++)
		{
			sum += get(a, iY, iZ);
		}
		return sum;
	}

	/* For a given X and Y, return the sum of the Z line*/
	int sumLineXZ(string sX, string sZ)
	{
		int iX = -1,
			iZ = -1;

		// search for X
		for (unsigned int i = 0 ; i < vLabelX.size() ; i++)
		{
			if (vLabelX[i] == sX) iX = i;
		}

		// search for Y
		for (unsigned int i = 0 ; i < vLabelZ.size() ; i++)
		{
			if (vLabelZ[i] == sZ) iZ = i;
		}

		int sum = 0;
		for (int a = 0 ; a < vLabelY.size() ; a++)
		{
			sum += get(iX, a, iZ);
		}
		return sum;
	}
};


/* SCORE PROP */

class scoreProp
{

public:
	int A;
	int B;
	int C;
	int D;

	/*------------------

	|-----------|-----------|
	|    A      |   B       |
	|-----------|-----------|
	|    C      |   D       |
	|-----------|-----------|

	*/

	double chiSquare()
	{

		/*
		input format :

		|   pop1    |   pop2    |
		------------|-----------|-----------|
		properties  |    A      |   B       |
		------------|-----------|-----------|
		~properties |    C      |   D       |
		------------|-----------|-----------|


		(N*(A*D - B*C)*(A*D - B*C)) 
		CHI2 = -----------------------------  
		(A+B)*(C+D)*(A+C)*(B+D)

		*/


		double dPValue;
		int N = A+B+C+D;
		double chi2 = (N*(A*D - B*C)*(A*D - B*C) ) / (1.*(A+B)*(C+D)*(A+C)*(B+D) );

		dPValue = pvalue(chi2);

		return dPValue;
	}

	double pvalue(double X) {
		/*-------------------------------------------------------- 
		Adapted From:
		Hill, I. D. and Pike, M. C. Algorithm 299
		Collected Algorithms for the CACM 1967 p. 243
		Updated for rounding errors based on remark in
		ACM TOMS June 1985, page 185   
		---------------------------------------------------------*/

		if (X <= 0.0)  return 1.0;

		return (2.0 * poz(-sqrt(X)));
	}

	double poz (double z) {
		/*-------------------------------------------------------------------------
		POZ  --  probability of standard normal z value

		Adapted from a polynomial approximation in:
		Ibbetson D, Algorithm 209
		Collected Algorithms of the CACM 1963 p. 616

		Note:
		This routine has six digits accuracy, so it is only useful for absolute
		z values < 6.  For z values >=  6.0, poz() returns 0.0.                 
		--------------------------------------------------------------------------*/
		double Y, X, w, zmax;
		zmax = 6.0;
		if (z == 0.0)
			X = 0.0;
		else {
			Y = 0.5 * fabs(z);
			if (Y >= zmax * 0.5)
				X = 1.0;
			else if (Y < 1.0) {
				w = Y * Y;
				X = ((((((((0.000124818987*w -0.001075204047)*w + 0.005198775019)*w -0.019198292004)*w + 0.059054035642)*w
					-0.151968751364)*w + 0.319152932694) * w - 0.5319230073)*w + 0.7978845605929999)*Y*2;
			}
			else {
				Y = Y - 2;
				X = (((((((((((((-0.000045255659*Y + 0.00015252929)*Y -0.000019538132)*Y - 6.769049860000001E-04)*Y
					+0.001390604284)*Y-0.00079462082)*Y -0.002034254874)*Y + 0.006549791214)*Y - 0.010557625006)*Y
					+0.011630447319)*Y-9.279453341000001E-03)*Y + 0.005353579108)*Y - 0.002141268741)*Y
					+0.000535310849)*Y + 0.999936657524;
			}
		}
		if (z > 0.0)
			return ((X + 1) * 0.5);
		else
			return ((1 - X) * 0.5);

	}

	double  getScore()
	{
		return ((1.*abmReasoningFunction::SIGMA_LEARNING_GRAMMAR+A) / (B+C+D+abmReasoningFunction::SIGMA_LEARNING_GRAMMAR));
	}

	// if !bPositive multiplie the result by -1
	double  getScoreSum(bool bPositive = true)
	{
		if (B+C+D == 0 && A !=0)    return 1.;
		if (A+B == 0 || A+C == 0 || C+D == 0)   return 0.;
		if (B+D == 0) 
		{
			return (2*A/(A+C*1.)-1);
		}
		if (chiSquare() > abmReasoningFunction::THRESHOLD_CONFIDENCE_GRAMMAR)   return 0.;

		double dFactor;
		bPositive ? dFactor = 1. : dFactor = -1.;
		int iDistrib = 1;
		if ((1.*abmReasoningFunction::SIGMA_LEARNING_GRAMMAR+A) / (C+1.*abmReasoningFunction::SIGMA_LEARNING_GRAMMAR) - (1.*abmReasoningFunction::SIGMA_LEARNING_GRAMMAR+B) / (D+1.*abmReasoningFunction::SIGMA_LEARNING_GRAMMAR) < 0) iDistrib = -1;

		return (dFactor*iDistrib*(1-chiSquare()));
	}

	scoreProp()
	{
		A=0;
		B=0;
		C=0;
		D=0;
	}

	scoreProp(int a, int b, int c, int d)
	{
		A = a;
		B = b;
		C = c;
		D = d;
	}

	void plus(scoreProp score2)
	{
		A += score2.A;
		B += score2.B;
		C += score2.C;
		D += score2.D;
	}

	void addScore(pair<int, int> pInput, bool bFirstCol)
	{
		if (bFirstCol)  {
			A += pInput.first;
			C += pInput.second; }
		else    {
			B += pInput.first;
			D += pInput.second; }
	}


};

#endif