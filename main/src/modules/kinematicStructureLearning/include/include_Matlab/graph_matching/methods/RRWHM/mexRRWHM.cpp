#include "mex.h"
#include "math.h"
#include "mexOliUtil.h"

#include "mexRRWHM.h"

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray*prhs[] )
{ 
    enum{ Xi , indH1i , valH1i , indH2i , valH2i , indH3i , valH3i , maxIteri, ci };
    enum{ Xo, nIter };
    oliCheckArgNumber(nrhs,9,nlhs,2);
    int Nt1,Nt2,Nt3,N1,N2;
    double* pX = oliCheckArg(prhs,Xi,&N1,&N2,oliDouble);
    int* pIndH1 = (int*)oliCheckArg(prhs,indH1i,&Nt1,1,oliInt);
    double* pValH1 = oliCheckArg(prhs,valH1i,Nt1,1,oliDouble);
    int* pIndH2 = (int*)oliCheckArg(prhs,indH2i,&Nt2,2,oliInt);
    double* pValH2 = oliCheckArg(prhs,valH2i,Nt2,1,oliDouble);
    int* pIndH3 = (int*)oliCheckArg(prhs,indH3i,&Nt3,3,oliInt);
    double* pValH3 = oliCheckArg(prhs,valH3i,Nt3,1,oliDouble);
    double* pMaxIter = oliCheckArg(prhs,maxIteri,1,1,oliDouble);
    double* pC = oliCheckArg(prhs,ci,1,1,oliDouble);

    plhs[Xo] = mxCreateDoubleMatrix(N1, N2, mxREAL);
    plhs[nIter] = mxCreateDoubleScalar(0);
    double* pXout = mxGetPr(plhs[Xo]);
    double* pNIter = mxGetPr(plhs[nIter]);

    RRWM(pX,N1,N2,pIndH1,pValH1,Nt1,pIndH2,pValH2,Nt2,pIndH3,pValH3,Nt3,(int)(*pMaxIter),pC,pXout,pNIter);
}














