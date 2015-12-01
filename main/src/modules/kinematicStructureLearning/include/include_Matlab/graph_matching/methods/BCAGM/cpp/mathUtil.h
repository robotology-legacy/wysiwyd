/**
 * Utility functions (CVPR 2015)
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
 * History
 * 	- last update: 28.04.2015
 **/
#ifndef MATH_UTIL_H
#define MATH_UTIL_H 1

#include <sstream>
#include <math.h>
#include <utility>
#include <iostream>
#include "hungarian_Q.cpp"

#define FOR(i, n) for(int i = 0; i < n; ++i)
#define FOR2(i, a, b) for(int i = a; i <= b; ++i)
#define FORD(i, n) for(int i = n-1; i >= 0; i--)
#define FORD2(i, a, b) for(int i = b; i >= a; i--)

inline double norm2(const double* a, int n) {
    double s = 0;
    for (int i = 0; i < n; ++i) {
        s += a[i] * a[i];
    }
    return s;
}

inline double norm(const double* a, int n) {
    return sqrt(norm2(a, n));
}

inline double pNorm(const double* a, int n, double p) {
    double s = 0;
    FOR(i, n) {
        s = s + pow(fabs(a[i]), p);
    }
    return pow(s, 1/p);
}

inline double innerProd(const double* a, const double* b, int n) {
	double s = 0;
	FOR(i, n) {
		s += a[i]*b[i];
	}
	return s;
}

inline void normalize(double* x, int n) {
	double s = 0;
	FOR(i, n) s += x[i];
	FOR(i, n) x[i] /= s;
}

inline void discritize(const double* X, int n1, int n2, double* Y) {
	double** matrix = array_to_matrix(X, n1, n2);
	hungarian_problem_t hunger;
	hungarian_init(&hunger, matrix, n1, n2, HUNGARIAN_MODE_MAXIMIZE_UTIL) ;
	hungarian_solve(&hunger);
	FOR(i, n1) {
		FOR(j, n2) Y[i*n2+j] = (hunger.assignment[i][j] > 0) ?1: 0;
	}
	hungarian_free(&hunger);
	if (matrix != NULL) {
		FOR(i, n1) {
			if (matrix[i] != NULL) {
				free(matrix[i]);
			}
		}
		free(matrix);
	}
}

#endif
