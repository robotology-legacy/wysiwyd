#include "mex.h"
#include "mexGetMatchScore.h"

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
    enum{indH, valH, X}; // Input
    enum{MatchScore}; // Output
    
    int* pIndH;
    double* pValH;
    double* pX;
    
    int size = mxGetM(prhs[0]);
    pIndH = (int*)mxGetPr(prhs[indH]);
    pValH = mxGetPr(prhs[valH]);
    pX = mxGetPr(prhs[X]);
    
    plhs[MatchScore] = mxCreateDoubleScalar(mxREAL);
    double* pMatchScore = mxGetPr(plhs[MatchScore]);
    
    getMatchScore(pIndH, pValH, size, pX, pMatchScore);
}