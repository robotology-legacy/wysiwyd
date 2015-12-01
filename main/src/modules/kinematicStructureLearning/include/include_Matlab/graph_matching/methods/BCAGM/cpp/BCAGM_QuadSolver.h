 /**
 * Demo Code for BCAGM+Psi algorithm (CVPR 2015)
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
#ifndef BCAGM_QUAD_SOLVER_H
#define BCAGM_QUAD_SOLVER_H 1

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

void computeBigMat(int* indH3, double* valH3, int Nt3, 
			 int i, double* x, int N1, int N2, double alpha, double* mat) {
	int dim = N1 * N2;
	int L = dim * 2;
	FOR(j, dim*dim) mat[j] = 0;

	double w = 0;
	FOR(j, dim) w += x[(1-i)*dim + j];

	double* v = new double[dim];
	FOR(j, dim) v[j] = 0;
	
	int p[3];
	FOR(k, Nt3) {
		int i1 = indH3[k*3];
		int i2 = indH3[k*3+1];
		int i3 = indH3[k*3+2];
		mat[dim*i1+i2] += 2 * valH3[k] * x[(1-i)*dim+i3] * w;
		v[i1] += valH3[k] * x[(1-i)*dim+i2] * x[(1-i)*dim+i3];
	}
	FOR(k, dim) FOR(l, dim) mat[k*dim + l] += (v[k] + v[l]);

	// for convexity
	if (alpha > 0) {
		double s = 0;
		FOR(j, dim) s += x[(1-i)*dim + j] * x[(1-i)*dim + j];
		FOR(k, dim) mat[k*dim + k] += 2*alpha/3 * s;
		FOR(k, dim) FOR(l, dim) mat[k*dim+l] += alpha*4/3 * x[(1-i)*dim+k] * x[(1-i)*dim+l];
	}

	delete[] v;
}

// compute the objective F_alpha(xyzt)
double getBigObj_Quad(int* indH3, double* valH3, int Nt3,
			double* x, int N1, int N2, double alpha) {
	int dim = N1 * N2;
	double* mat = new double[dim*dim];
	int i = 0;
	computeBigMat(indH3, valH3, Nt3, i, x, N1, N2, alpha, mat);
	double s = 0;
	FOR(k, dim) FOR(l, dim) s += mat[k*dim + l] * x[i*dim + k] * x[i*dim + l];
	delete[] mat;
	return s;
}

void maxPooling(const double* mat, int N1, int N2, double* v) {
	int dim = N1 * N2;
	double* pv = new double[dim];
	double* ppv = new double[dim];
	FOR(i, dim) v[i] = pv[i] = ppv[i] = 1.0 / dim;
	double tol = 1e-12;
	double diff = 1;
	int iter = 0;
	double oldf = 0;
	double curf = 0;

	while (diff > tol && iter < 30)  {
		iter++;
		FOR(i, dim) ppv[i] = pv[i];
		FOR(i, dim) pv[i] = v[i];
		FOR(i, dim) v[i] = 0;
		
		FOR(i, dim) {
			v[i] = pv[i] * mat[i*dim + i];
			FOR(j1, N1) {
				double best = 0;
				FOR(j2, N2) {
					int j = j1*N2 + j2;
					double s = mat[i*dim + j] * pv[j];
					if (best < s) best = s;
				}
				v[i] += best;
			}
		}
		double s = 0;
		FOR(i, dim) s += v[i]*v[i];
		s = sqrt(s);
		FOR(i, dim) v[i] /= s;

		double diff1 = 0, diff2 = 0;
		FOR(i, dim) diff1 += (v[i]-pv[i])*(v[i]-pv[i]);
		FOR(i, dim) diff2 += (v[i]-ppv[i])*(v[i]-ppv[i]);
		diff = (diff1 < diff2) ?diff1 :diff2;
	}
	delete[] ppv;	
	delete[] pv;
}

void ipfp(const double* mat, const double* x, int N1, int N2, double* v) {
	int dim = N1 * N2;
	double* y = new double[dim];
	/*FOR(i, dim) y[i] = x[i];*/
	FOR(i, dim) y[i] = 1.0 / dim;
	double* grad = new double[dim];
	double* b = new double[dim];
	double* prev_y = new double[dim];
	double tol = 1e-12;
	int nIter = 0;
	
	double opt = 0;
	FOR(i, dim) v[i] = x[i];
	FOR(k, dim) FOR(l, dim) opt += mat[k*dim+l]*v[k]*v[l];

	while (nIter < 20) {
		FOR(i, dim) prev_y[i] = y[i];
		nIter++;
		FOR(j, dim) grad[j] = 0;
		FOR(j, dim) {
			FOR(k, dim) {
				grad[j] += mat[j*dim + k] * y[k];
			}
		}
		discritize(grad, N1, N2, b);
		double C = 0, D = 0, f = 0;
		FOR(k, dim) FOR(l, dim) {
			C += mat[k*dim+l]*y[k]*(b[l]-y[l]);
			D += mat[k*dim+l]*(b[k]-y[k])*(b[l]-y[l]);
			f += mat[k*dim+l]*b[k]*b[l];
		}
		if (D >= 0) {
			FOR(i, dim) y[i] = b[i];
		} else {
			double r = min(-C/D, 1.0);
			FOR(i, dim) y[i] += r * (b[i]-y[i]);
		}
		if (opt < f) {
			opt = f;
			FOR(i, dim) v[i] = b[i];
		}
		double diff = 0;
		FOR(i, dim) diff = max(diff, fabs(y[i]-prev_y[i]));
		if (diff < tol) break;
	}
	delete[] grad;
	delete[] b;
	delete[] y;
	delete[] prev_y;
}

// check if all arguments are the same
int isHomogeneous(double* x, int dim) {
	double diff = 0;
	FOR(j, dim) {
		diff = max(diff, fabs(x[j] - x[dim+j]));
	}
	return (diff < 1e-12) ?1 :0;
}

// BCAGM+MP algorithm
void bcagm_mp_proj(int* indH3, double* pvalH3, int Nt3,
			double* x0, int N1, int N2, double* xout, double* objs, double* nIter) {
	double* valH3 = new double[Nt3];
	FOR(i, Nt3) valH3[i] = pvalH3[i];

	// normalize tensors to avoid numerical errors
	normalize(valH3, Nt3);

	double tol = 1e-12;
	int dim = N1*N2;
	int L = dim * 2;

	double* x = new double[L];
	FOR(j, L) x[j] = fabs(x0[j]);
	normalize(x, L);

	double* xnew = new double[L];
	double* tmp = new double[L];
	double* mat = new double[dim*dim];
	double* var = new double[dim];
	double bound = 0;
	FOR(i, Nt3) bound += valH3[i] * valH3[i];
	bound = 3*sqrt(bound);
	double alpha = 0;
	bool finish = false;
	*nIter = 0;
	objs[0] = 0;

	while (*nIter < 20) {
		double oldf = objs[(int)*nIter];
		double curf = (*nIter == 0) ?getBigObj_Quad(indH3, valH3, Nt3, x, N1, N2, alpha) :oldf;
		for(int i = 0; i < 2; ++i) {
			computeBigMat(indH3, valH3, Nt3, i, x, N1, N2, alpha, mat);
			maxPooling(mat, N1, N2, var);
			// project back to discrete domain
			FOR(j, dim) tmp[j] = var[j];
			discritize(tmp, N1, N2, var);
			// compute temporary objective
			FOR(j, L) tmp[j] = x[j];
			FOR(j, dim) tmp[i*dim+j] = var[j];
			double ftmp = getBigObj_Quad(indH3, valH3, Nt3, tmp, N1, N2, alpha);

			if (curf < ftmp) {
				FOR(j, dim) x[i*dim+j] = var[j];
				curf = ftmp;
			}
		}
		*nIter += 1;
		objs[(int)*nIter] = curf;

		printf("QuadSolver: iter %d, obj %.12f \n", (int)*nIter, curf);

		if (curf - oldf < tol) {
			if (alpha == 0.0) {
				if (isHomogeneous(x, dim)) {
					FOR(j, dim) xout[j] = x[j];
					finish = true;
					break;
				} else {
					alpha = bound;
					*nIter += 1;
					objs[(int)*nIter] = getBigObj_Quad(indH3, valH3, Nt3, x, N1, N2, alpha);
					printf("QuadSolver: ALPHA TURNED ON\n");
				}
			} else {
				double best = 0;
				FOR(i, 2) {
					FOR(j, L) tmp[j] = x[i*dim + j%dim];
					double f = getBigObj_Quad(indH3, valH3, Nt3, tmp, N1, N2, alpha);
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
				printf("QuadSolver: jump to alpha %.12f, obj %.12f \n", alpha, best);
			}
		}
		if (finish) break;
	}

	if (!finish) {
		FOR(j, dim) xout[j] = x[j];
	}
	
	delete[] tmp;
	delete[] xnew;
	delete[] mat;
	delete[] var;
	delete[] x;
	delete[] valH3;
}

// BCAGM+IPFP algorithm
void bcagm_ipfp(int* indH3, double* pvalH3, int Nt3,
			double* x0, int N1, int N2, double* xout, double* objs, double* nIter) {
	double* valH3 = new double[Nt3];
	FOR(i, Nt3) valH3[i] = pvalH3[i];

	// normalize tensors to avoid numerical errors
	normalize(valH3, Nt3);

	double tol = 1e-12;
	int dim = N1*N2;
	int L = dim * 2;

	double* x = new double[L];
	FOR(j, L) x[j] = fabs(x0[j]);
	normalize(x, L);

	double* xnew = new double[L];
	double* tmp = new double[L];
	double* mat = new double[dim*dim];
	double* var = new double[dim];
	double bound = 0;
	FOR(i, Nt3) bound += valH3[i] * valH3[i];
	bound = 3*sqrt(bound);
	double alpha = 0;
	bool finish = false;
	*nIter = 0;
	objs[0] = 0;

	while (*nIter < 20) {
		double oldf = objs[(int)*nIter];
		double curf = (*nIter == 0) ?getBigObj_Quad(indH3, valH3, Nt3, x, N1, N2, alpha) :oldf;
		for(int i = 0; i < 2; ++i) {
			computeBigMat(indH3, valH3, Nt3, i, x, N1, N2, alpha, mat);
			ipfp(mat, x+i*dim, N1, N2, var);
			FOR(j, L) tmp[j] = x[j];
			FOR(j, dim) tmp[i*dim+j] = var[j];
			double ftmp = getBigObj_Quad(indH3, valH3, Nt3, tmp, N1, N2, alpha);

			if (curf < ftmp) {
				FOR(j, dim) x[i*dim+j] = var[j];
				curf = ftmp;
			}
		}
		*nIter += 1;
		objs[(int)*nIter] = curf;

		printf("QuadSolver: iter %d, obj %.12f \n", (int)*nIter, curf);

		if (curf - oldf < tol) {
			if (alpha == 0.0) {
				if (isHomogeneous(x, dim)) {
					FOR(j, dim) xout[j] = x[j];
					finish = true;
					break;
				} else {
					alpha = bound;
					*nIter += 1;
					objs[(int)*nIter] = getBigObj_Quad(indH3, valH3, Nt3, x, N1, N2, alpha);
					printf("QuadSolver: ALPHA TURNED ON\n");
				}
			} else {
				double best = 0;
				FOR(i, 2) {
					FOR(j, L) tmp[j] = x[i*dim + j%dim];
					double f = getBigObj_Quad(indH3, valH3, Nt3, tmp, N1, N2, alpha);
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
				printf("QuadSolver: jump to alpha %.12f, obj %.12f \n", alpha, best);
			}
		}
		if (finish) break;
	}

	if (!finish) {
		FOR(j, dim) xout[j] = x[j];
	}
	
	delete[] tmp;
	delete[] xnew;
	delete[] mat;
	delete[] var;
	delete[] x;
	delete[] valH3;
}

// main program
// NOTE that the input arrays: problem.indH3 and problem.valH3 should be symmetric themselves. 
// This means they should store all the six permutations of each non-zero entries instead of storing only one
void bcagm_quad(int* indH1, double* pvalH1, int Nt1,
			int* indH2, double* pvalH2, int Nt2,
			int* indH3, double* pvalH3, int Nt3,
			double* x0, int N1, int N2, int mode, double* xout, double* objs, double* nIter) {
	if (mode == 1) {
		bcagm_mp_proj(indH3, pvalH3, Nt3, x0, N1, N2, xout, objs, nIter);
	} else if (mode == 2) {
		bcagm_ipfp(indH3, pvalH3, Nt3, x0, N1, N2, xout, objs, nIter);
	}	
}

#endif

