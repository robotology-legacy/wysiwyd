#include <abmReasoningFunction.h>

using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;


class pronom
{	
public:
	string								sSubject;		// or pronom

	matrix3D							m3Data;

	pair<int, int>						pSubject_Is_Agent;
	pair<int, int>						pSubject_Is_Speaker;
	pair<int, int>						pSubject_Is_Addressee;

	bool		AddInstance(Bottle bInput);			//	Add an instance to the stats


	// TODO : TO BE REMOVE
	pair<int, int>						pSpeaker_Is_Addressee;
	pair<int, int>						pSpeaker_Is_Subject;
	pair<int, int>						pSpeaker_Is_Agent;

	pair<int, int>						pAddressee_Is_Subject;
	pair<int, int>						pAddressee_Is_Agent;

	vector<pair<string, int> >			vAddresseeIs;	//	Vector with the name of addressee and interances
	vector<pair<string, int> >			vAgentIs;	//	Vector with the name of addressee and interances
	vector<pair<string, int> >			vSpeakerIs;	//	Vector with the name of addressee and interances

	
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
	|    A		|   B		|
	|-----------|-----------|
	|	 C		|	D		|
	|-----------|-----------|

	*/

	double chiSquare()
	{

		/*
		input format :

		|	pop1	|	pop2	|
		------------|-----------|-----------|
		properties	|    A		|   B		|
		------------|-----------|-----------|
		~properties	|	 C		|	D		|
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
	
	double	getScore()
	{
		return ((1.*abmReasoningFunction::SIGMA_LEARNING_GRAMMAR+A) / (B+C+D+abmReasoningFunction::SIGMA_LEARNING_GRAMMAR));
	}
	
	// if !bPositive multiplie the result by -1
	double	getScoreSum(bool bPositive = true)
	{
		if (B+C+D == 0 && A !=0)	return 1.;
		if (A+B == 0 || A+C == 0 || C+D == 0)	return 0.;
		if (B+D == 0) 
		{
			return (2*A/(A+C*1.)-1);
		}
		if (chiSquare() > abmReasoningFunction::THRESHOLD_CONFIDENCE_GRAMMAR)	return 0.;

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
		if (bFirstCol)	{
			A += pInput.first;
			C += pInput.second;	}
		else	{
			B += pInput.first;
			D += pInput.second;	}
	}


};






