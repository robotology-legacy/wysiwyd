/*
 * ==============================================================
 * RMP_mult.c  Compute a matrix vector multiplication for Max-Pooling Matching
 *
 * Minsu CHO
 * 8 Sep 2013
 * =============================================================
 */

#include "mex.h"

/*
 * The mex function computes the reweighted max-pooling 
 */
void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    int i, j, k, g, ge;
    int mrows, ncols, ngroups;
    
    /* sparse matrix */
    mwIndex *G_row, *G_col;
    bool *G_val;
    
    double *x;
    double *y;
    double *matM;
    
    double val, vmax;
    int imax;
    bool bSame;
    
    if (nrhs != 3) 
    {
        mexErrMsgTxt("3 inputs required.");
    }
    else if (nlhs > 1) 
    {
        mexErrMsgTxt("Too many output arguments");
    }
    
    /* The first input must be a nonsparse matrix. */
    mrows = mxGetM(prhs[0]);
    ncols = mxGetN(prhs[0]);
    if (mxIsSparse(prhs[0]) ||
        !mxIsDouble(prhs[0]) || 
        mxIsComplex(prhs[0])) 
    {
        mexErrMsgTxt("Input must be a noncomplex nonsparse matrix.");
    }
        
    /* The second input must be a vector. */
    if (mxGetM(prhs[1])*mxGetN(prhs[1]) != ncols ||
        mxIsSparse(prhs[1]) || !mxIsDouble(prhs[1]))
    {
        mexErrMsgTxt("Invalid vector.");
    }
    
    /* The third input must be a nonsparse matrix. */
    ngroups = mxGetN(prhs[2]);
    if (!mxIsSparse(prhs[2]) || mxGetM(prhs[2]) != mrows ||
        !mxIsLogical(prhs[2])) 
    {
        mexErrMsgTxt("Invalid group matrix.");
    }
    
    /* Get the sparse matrix */
    G_val = mxGetLogicals(prhs[2]);
    G_row = mxGetIr(prhs[2]);
    G_col = mxGetJc(prhs[2]);
    
    /* Get the nonsparse matrix */
    matM = mxGetPr(prhs[0]);
    
    /* Get the vector x */
    x = mxGetPr(prhs[1]);
    
    plhs[0] = mxCreateDoubleMatrix(mrows,1,mxREAL);
    y = mxGetPr(plhs[0]);
    
    for (j = 0; j < mrows; j++)
        y[j] = 0;
    
    for (j = 0; j < mrows; j++)
    {
        for (g = 0; g < ngroups; ++g)
        {
            vmax = -9999;
            bSame = false;
            
            for (ge = G_col[g]; ge < G_col[g+1]; ++ge)
            {
                if ( G_row[ge] == j )
                {
                    bSame = true;
                    break;
                }
                
                val = x[G_row[ge]] * matM[ j + mrows*G_row[ge] ];
                
                if ( val > vmax ){
                    vmax = val;
                    imax = G_row[ge];
                }
            }
            
            if ( !bSame )
            {
                y[j] += vmax*vmax;  
            }
        }
        
    }
}

