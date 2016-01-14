 /**
 * Demo Code for BCAGM algorithm (CVPR 2015)
 *
 * Quynh Nguyen, Antoine Gautier, Matthias Hein
 * A Flexible Tensor Block Coordinate Ascent Scheme for Hypergraph Matching
 * Proc. of the IEEE International Conference on Computer Vision and Pattern Recognition (CVPR 2015)
 * 
 * Please cite our work if you find this code useful for your research
 * 
 * written by Quynh Nguyen, 2014, Saarland University, Germany
 * http://www.ml.uni-saarland.de/people/nguyen.htm 
 * 
 * Version: 1.0
 * Date: 28.04.2015
 **/

#ifndef BCAGM_LIN_SOLVER_H
#define BCAGM_LIN_SOLVER_H 1

#include <iostream>
#include <exception>
#include <time.h>
#include <stdlib.h>
#include <float.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <algorithm>
#include "mathUtil.h"
using namespace std;

#define FOR(i, n) for(int i = 0; i < n; ++i)
#define FOR2(i, a, b) for(int i = a; i <= b; ++i)
#define FORD(i, n) for(int i = n-1; i >= 0; i--)
#define FORD2(i, a, b) for(int i = b; i >= a; i--)

// compute the gradient of F(xyzt) w.r.t the 1st variable stored in x[dim*3]
void computeMultGradient(int* indH3, double* valH3, int Nt3, int* ha, int* hb,
        double* x, int N1, int N2, double* grad) {
    int dim = N1 * N2;
    FOR(k, dim) grad[k] = 0;
    FOR(i, dim) if (x[dim+i] > 0)  {
        FOR(j, dim) if (x[dim*2+j] > 0) {
            int ij = i*dim + j;
            int a = ha[ij];
            int b = hb[ij];
            if (a < 0) continue;
            FOR2(t, a, b) {
                int k = indH3[t*3+2];
                grad[k] += valH3[t] * x[dim+i] * x[dim*2+j];
            }
        }
    }
}

// compute the gradient of F_alpha(xyzt) = F(xyzt) + alpha * G(xyzt) w.r.t. the i-th variable (i=0..3)
void computeBigGrad( int* indH3, double* valH3, int Nt3, int* ha, int* hb,
               int i, double* x, int N1, int N2, double alpha, double* gradi) {
    int dim = N1 * N2;
    FOR(t, dim) gradi[t] = 0;
    double norms[4];
    FOR(j, 4) norms[j] = pNorm(x+j*dim, dim, 1.0);
    double prod = 1;
    FOR(j, 4) prod *= norms[j];
    double* y = new double[dim*3];
    double* g = new double[dim];
    
    FOR(j, 4) {
        int n = 0;
        FOR2(k, i, i+3) {
            if (k % 4 != j) {
                FOR(t, dim) y[n++] = x[(k%4)*dim + t];
            }
        }
        computeMultGradient(indH3, valH3, Nt3, ha, hb, y, N1, N2, g);

        if (j != i) {
            FOR(t, dim) gradi[t] += norms[j] * g[t];
        } else {
            double s = 0;
            FOR(t, dim) s += g[t] * y[t];
            FOR(t, dim) gradi[t] += s;
        }
    }
    delete[] y;
    delete[] g;
    
    if (alpha > 0) {
        double s;
        s = innerProd(x+((i+1)%4)*dim, x+((i+2)%4)*dim, dim);
        FOR(t, dim) gradi[t] += alpha/3 * s * x[((i+3)%4)*dim + t];

        s = innerProd(x+((i+1)%4)*dim, x+((i+3)%4)*dim, dim);
        FOR(t, dim) gradi[t] += alpha/3 * s * x[((i+2)%4)*dim + t];

        s = innerProd(x+((i+2)%4)*dim, x+((i+3)%4)*dim, dim);
        FOR(t, dim) gradi[t] += alpha/3 * s * x[((i+1)%4)*dim + t];
    }
}

// compute the objective F_alpha(xyzt)
double getBigObj( int* indH3, double* valH3, int Nt3, int* ha, int* hb,
            double* x, int N1, int N2, double alpha) {
    int dim = N1 * N2;
    double* gradi = new double[dim];
    int i = 0;
    computeBigGrad(indH3, valH3, Nt3, ha, hb, i, x, N1, N2, alpha, gradi);
    double s = 0;
    FOR(j, dim) s += gradi[j] * x[i*dim + j];
    delete[] gradi;
    return s;
}

// check if all the arguments are the same
bool isHomogeneous(double* x, int dim) {
    double diff = 0;
    FOR(i, 3) {
        FOR(j, dim) {
            diff = max(diff, fabs(x[i*dim+j] - x[3*dim+j]));
        }
    }
    return diff < 1e-12;
}

// main algorithm
// NOTE that the input arrays: problem.indH3 and problem.valH3 should be symmetric themselves. 
// This means they should store all the six permutations of each non-zero entries instead of storing only one
void bcagm(int* indH1, double* pvalH1, int Nt1, 
        int* indH2, double* pvalH2, int Nt2, 
        int* indH3, double* pvalH3, int Nt3,
        double* x0, int N1, int N2, int maxIter, double* xout, double* objs, double* nIter) {

    double* valH3 = new double[Nt3];
    FOR(i, Nt3) valH3[i] = pvalH3[i];

    // normalize tensors to avoid numerical errors
    normalize(valH3, Nt3);

    double tol = 1e-12;
    int dim = N1*N2;
    int L = dim * 4;

    double* x = new double[L];
    FOR(j, L) x[j] = fabs(x0[j]);
    normalize(x, L);

    double* xnew = new double[L];
    double* tmp = new double[L];
    double* var = new double[dim];
    double* gradi = new double[dim];
    *nIter = 0;
    objs[0] = 0;
    double bound = 0;
    FOR(i, Nt3) bound += valH3[i] * valH3[i];
    bound = 3*sqrt(bound);
    double alpha = 0;
    bool finish = false;

    // create mapping for nonzeros
    int L2 = dim*dim;
    int* ha = new int[L2];
    int* hb = new int[L2];
    FOR(i, L2) ha[i] = hb[i] = -1;
    FOR(t, Nt3) {
        int i = indH3[3*t];
        int j = indH3[3*t+1];
        int k = i*dim + j;
        if (ha[k] == -1) ha[k] = t;
        hb[k] = t;
    }

    while (*nIter < 20) {
        double oldf = objs[(int)*nIter];
        *nIter += 1;
        FOR(i, 4) {
            computeBigGrad(indH3, valH3, Nt3, ha, hb, i, x, N1, N2, alpha, gradi);
            discritize(gradi, N1, N2, var);
            FOR(j, dim) x[i*dim+j] = var[j];
        }
        double curf = 0;
        FOR(j, dim) curf += gradi[j] * var[j];
        objs[(int)*nIter] = curf;
        
        printf("LinSolver: iter %d, obj %.12f \n", (int)*nIter, curf);
        
        if (curf - oldf < tol) {
            if (alpha == 0.0) {
                        if (isHomogeneous(x, dim)) {
                                FOR(j, dim) xout[j] = x[j];
                    finish = true;
                    break;
                        } else {
                                alpha = bound;
                                *nIter += 1;
                                objs[(int)*nIter] = getBigObj(indH3, valH3, Nt3, ha, hb, x, N1, N2, alpha);
                                printf("LinSolver: ALPHA TURNED ON\n");
                        }
            } else {
                double best = 0;
                FOR(i, 4) {
                    FOR(j, L) tmp[j] = x[i*dim + j%dim];
                    double f = getBigObj(indH3, valH3, Nt3, ha, hb, tmp, N1, N2, alpha);
                    if (best < f) {
                        best = f;
                        FOR(j, L) xnew[j] = tmp[j];
                    }
                }
                if (best == curf) {
                    FOR(j, dim) xout[j] = xnew[j];
                    finish = true;
                    break;
                }
                FOR(j, L) x[j] = xnew[j];
                *nIter += 1;
                objs[(int)*nIter] = best;
                printf("LinSolver: jump to alpha %.12f, obj %.12f \n", alpha, best);
            }
        }
        if (finish) break;
    }

    if (!finish) {
        FOR(j, dim) xout[j] = x[j];
    }
    
    delete[] gradi;
    delete[] var;
    delete[] tmp;
    delete[] xnew;
    delete[] x;
    delete[] valH3;
    delete[] ha;
    delete[] hb;
}


#endif

