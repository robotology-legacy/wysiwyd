#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <float.h>
#include "mex.h"

/* Input Arguments */
#define  MATX_IN        prhs[0]
#define  WARPWIN_IN     prhs[1]
#define  QUERYSKIP_IN   prhs[2]
#define  TARGETSKIP_IN  prhs[3]
#define  JUMPCOST_IN    prhs[4]      


/* Output Arguments */
#define  PATHCOST_OUT   plhs[0]
#define  INDXROW_OUT    plhs[1]
#define  INDXCOL_OUT    plhs[2]

/* Constants */
const double INF = DBL_MAX;


static int findpathDAG( double matx[], int m, int n, int warpwin,
                         int queryskip, int targetskip, double jumpcost,
                         double *pathcost, int **indxrow, int **indxcol )
{
    int *camefromcol, *camefromrow, *tmp;
    double *weight;

    int i, j, stoprowjump, rowjump, stopk, k, mincol, minrow, mincoltemp,
        indxSize, idx1, idx2;
    double newweight;

    weight = (double *)malloc( m * n * sizeof(double) );
    camefromcol = (int *)malloc( m * n * sizeof(int) );
    camefromrow = (int *)malloc( m * n * sizeof(int) );

    /* Initialize matrices */
    for ( i = 0; i < m*n; i++ )
    {
        weight[i] = INF;
        camefromcol[i] = -1;
        camefromrow[i] = -1;
    }

    /* Initialize first row - remember, MatLab uses "column major" */
    for ( j = 0; j < n; j++ )
    {
        weight[j*m] = matx[j*m];
    }

    /* Initialize first column - remember, MatLab uses "column major" */
    for ( j = 0; j < m; j++ )
    {
        weight[j] = matx[j];
    }

    for ( i = 0; i < m-1; i++ )
    {
        for ( j = 0; j < n-1; j++ )
        {
            if ( fabs(i-j) <= warpwin )
            {
                stoprowjump = (m < i+1+queryskip)?m:i+1+queryskip;
                for ( rowjump = i+1; rowjump < stoprowjump; rowjump++ )
                {
                    stopk = (n < j+1+targetskip)?n:j+1+targetskip;
                    for ( k = j+1; k < stopk; k++ )
                    {
                        idx1 = i + (j * m);
                        idx2 = rowjump + (k * m);
                        newweight = weight[idx1] + matx[idx2] +
                                    ((rowjump-i-1) + (k-j-1)) * jumpcost;
                        if ( weight[idx2] > newweight )
                        {
                            weight[idx2] = newweight;
                            camefromrow[idx2] = i;
                            camefromcol[idx2] = j;
                        }
                    }
                }
            }
        }
    } /* End i loop */

    *pathcost = weight[m*n-1];

    indxSize = 0;
    mincol = n-1;
    minrow = m-1;

    while ( minrow >= 1 && mincol >= 1 )
    {
        if ( (tmp = (int *)malloc((indxSize+1) * sizeof(int))) == NULL )
            fprintf( stderr, "ERROR: malloc failed on indxcol" );
        tmp[0] = mincol;
        for ( i = 1; i < indxSize+1; i++ )
        {
            tmp[i] = (*indxcol)[i-1];
        }
        if ( indxcol != NULL )
          free(*indxcol);
        *indxcol = tmp;

        if ( (tmp = (int *)malloc((indxSize+1) * sizeof(int))) == NULL )
            fprintf( stderr, "ERROR: malloc failed on indxrow" );
        tmp[0] = minrow;
        for ( i = 1; i < indxSize+1; i++ )
        {
            tmp[i] = (*indxrow)[i-1];
        }
        if ( indxrow != NULL )
          free(*indxrow);
        *indxrow = tmp;

        indxSize++;
        mincoltemp = camefromcol[minrow + (mincol*m)];
        minrow = camefromrow[minrow + (mincol*m)];
        mincol = mincoltemp;
    }

    /* Convert from 0-based array indices to 1-based array indices */
    /* Note: Because we now subtract one because of the added first row
     * and column, comment out the next few lines
    for ( i = 0; i < indxSize; i++ )
    {
        (*indxcol)[i]++;
        (*indxrow)[i]++;
    }
    */
    
    free(weight);
    free(camefromcol);
    free(camefromrow);
    tmp = NULL;
    return indxSize;
}


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
    int *indxrow = NULL, *indxcol = NULL;
    double *matx, *tempPathcost, *tempRow, *tempCol;    

    int m, n, i, warpwin, queryskip, targetskip, indxSize;
    double jumpcost, pathcost;

    /* Check for proper number of arguments */
    if ( nrhs != 5 )
        mexErrMsgTxt( "Five input arguments required." );
    else if ( nlhs > 3 )
        mexErrMsgTxt( "Too many output arguments." );

    /* Extract MATX and its dimensions */
    matx = mxGetPr( MATX_IN );
    m = mxGetM( MATX_IN );
    n = mxGetN( MATX_IN );

    /* Extract other input variables */ 
    warpwin = mxGetScalar( WARPWIN_IN );
    queryskip = mxGetScalar( QUERYSKIP_IN );
    targetskip = mxGetScalar( TARGETSKIP_IN );
    jumpcost = mxGetScalar( JUMPCOST_IN );
    
    indxSize = findpathDAG( matx, m, n, warpwin, queryskip, targetskip, jumpcost, &pathcost, &indxrow, &indxcol);
    /* Decrement indxSize to remove the last index which is from the added row and column */
    if (indxSize > 0)
        indxSize--;
    
    /* Creating pointers for output variables */
    PATHCOST_OUT = mxCreateDoubleMatrix( 1, 1, mxREAL );
    INDXROW_OUT = mxCreateDoubleMatrix(1, indxSize, mxREAL );
    INDXCOL_OUT = mxCreateDoubleMatrix(1, indxSize, mxREAL );
    tempPathcost = mxGetPr( PATHCOST_OUT );
    tempRow = mxGetPr( INDXROW_OUT );
    tempCol = mxGetPr( INDXCOL_OUT );
    
    tempPathcost[0] = pathcost;
    for ( i = 0; i < indxSize; i++ )
    {
        tempRow[i] = indxrow[i];
        tempCol[i] = indxcol[i];
    }
    free(indxrow);
    free(indxcol);
    return;
}

   
