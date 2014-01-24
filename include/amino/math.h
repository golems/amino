/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2010-2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef AA_MATH_H
#define AA_MATH_H

/**
 * \file amino/math.h
 */

/***********/
/* Scalars */
/***********/

#ifndef AA_EPSILON
/// a small number
#define AA_EPSILON .001
#endif //AA_EPSILON

/// maximum of a and b
#define AA_MAX(a,b) \
    ({ const typeof(a) aa_$_max_a = (a); \
       const typeof(b) aa_$_max_b = (b); \
       (aa_$_max_a > aa_$_max_b) ? aa_$_max_a : aa_$_max_b; })

/// minimum of a and b
#define AA_MIN(a,b) \
    ({ const typeof(a) aa_$_min_a = (a); \
       const typeof(b) aa_$_min_b = (b); \
       (aa_$_min_a < aa_$_min_b) ? aa_$_min_a : aa_$_min_b; })

/// force value to be within +/- level
AA_DEPRECATED static inline double aa_clamp( double val, double level) {
    if( val > level )  return level;
    if( val < -level ) return -level;
    return val;
}

/// return val within range (min,max)
static inline double aa_fclamp( double val, double min, double max) {
    if( val > max )  return max;
    if( val < min ) return min;
    return val;
}

/// apply deadzone to val
static inline double aa_fdeadzone( double val, double min, double max, double deadval) {
    if( min < val && max > val )  return deadval;
    else return val;
}

/// modify each element of v to be within range (min,max)
static inline void aa_vclamp( size_t n, double *v, double min, double max) {
    for( size_t i = 0; i < n; i++ ) {
        if( v[i] > max )  v[i] = max;
        else if( v[i] < min ) v[i] = min;
    }
}


/// return the sign of val, one of {-1,0,1}
static inline double aa_sign( double val ) {
    if( val > 0 )  return 1;
    if( val < 0 ) return -1;
    return 0;
}

/// returns one if x is not infinity or NAN
AA_API int aa_isfok( double x );

/// Fuzzy equals
static inline int aa_feq( double a, double b, double tol ) {
    return fabs(a-b) <= tol;
}

/// Fuzzy equals
AA_API int aa_veq( size_t n, const double *a, const double *b, double tol );


/// square
static inline double aa_fsq( double a ) {
    return a * a;
}

/// Fortran modulo, Ada mod
#define AA_MODULO(a,b) (((a) % (b)) + (b)) % (b);

/// Fortran modulo, Ada mod
static inline int aa_imodulo( int a, int b ) {
    //return ((a % b) + b) % b;
    return AA_MODULO(a,b);
}

/// Fortran modulo, Ada mod
static inline long aa_lmodulo( long a, long b ) {
    return AA_MODULO(a,b);
}

/// Fortran modulo, Ada mod
static inline int64_t aa_imodulo64( int64_t a, int64_t b ) {
    return AA_MODULO(a,b);
}

/// Fortran mod, Ada rem
static inline int aa_iremainder( int a, int b ) {
    return a % b;
}

/// Mathematical modulo, Fortran modulo, Ada mod
static inline double aa_fmodulo( double a, double b ) {
    return fmod(fmod(a, b) + b,  b);
}

/// Mathematical remainder, Fortran mod, Ada rem
static inline double aa_fremainder( double a, double b ) {
    return fmod(a , b);
}

/** Returns index of minimum element in array v.
 */
AA_API size_t aa_fminloc( size_t n, double *v );

/** Returns index of maximum element in array v.
 */
AA_API size_t aa_fmaxloc( size_t n, double *v );

/// uniform pseudo-random in [0,1.0]
AA_API double aa_frand();

/// fills v with random numbers in [0,1.0]
AA_API void aa_vrand(size_t n, double *v);


/********/
/* Stat */
/********/

/** Generate 2 gaussian random numbers with stddev=1 from two uniform random numbers in interval (0,1].
 *
 * See Box, G. E. P. and Muller, M. E. "A Note on the Generation of
 * Random Normal Deviates." Ann. Math. Stat. 29, 610-611, 1958.
 */
AA_API void aa_stat_box_muller(double x1, double x2, double *z1, double *z2);

/** Convert z-score to x-score a normal distribution.
 */
static inline double aa_stat_z2x(double z, double mu, double sigma) {
    return (z * sigma) + mu;
}

/** Convert x-score to z-score a normal distribution.
 */
static inline double aa_stat_x2z(double x, double mu, double sigma) {
    return (x-mu)/sigma;
}

/** Compute mean of vector x */
AA_API double aa_stat_mean( size_t n, const double *x);

/** Compute standard deviation of vector x */
AA_API double aa_stat_std( size_t n, const double *x);

/** Compute mean and standard deviation, excluding outliers.
 *
 *  \param n length of x
 *  \param x vector of data
 *  \param[out] pmu mean
 *  \param[out] psigma standard deviation
 *  \param zmin exclude all outliers below zmin standard deviations
 *  \param zmax exclude all outliers above zmax standard deviations
 *  \param zmin exclude all outliers below zmin standard deviations
 *  \param zmax exclude all outliers above zmax standard deviations
 *  \param max_iterations maximum number of iterations
 */
AA_API size_t aa_stat_excluded_mean_std( size_t n, const double *x,
                                         double *pmu, double *psigma,
                                         double zmin, double zmax,
                                         size_t max_iterations );





/** Compute mean of angles */
AA_API double aa_stat_circ_mean( size_t n, const double *x);

/** Compute standard deviation of vector x */
AA_API double aa_stat_circ_std( size_t n, const double *x);

/** Compute mean and standard deviation, excluding outliers.
 *
 *  \param n length of x
 *  \param x vector of angles in radians
 *  \param[out] pmu mean
 *  \param[out] psigma standard deviation
 *  \param zmin exclude all outliers below zmin standard deviations
 *  \param zmax exclude all outliers above zmax standard deviations
 *  \param max_iterations maximum number of iterations
 */
AA_API size_t aa_stat_excluded_circ_mean_std( size_t n, const double *x,
                                              double *pmu, double *psigma,
                                              double zmin, double zmax,
                                              size_t max_iterations );


/** Compute mean of vectors.
 *
 * \param m size of space
 * \param n number of samples
 * \param X matrix of samples, one per column
 * \param[out] mu vector, length m
 */
AA_API void aa_stat_vmean( size_t m, size_t n, const double *X,
                           double *mu);

/** Compute sample covariance of vectors.
 *
 * \param m size of space
 * \param n number of samples
 * \param X matrix of samples, one per column
 * \param mu vector, length m
 * \param E matrix, m*m
 */
AA_API void aa_stat_vmean_cov( size_t m, size_t n, const double *X,
                               double *mu, double *E);


/** Mahalanobis distance. */
double aa_stat_mahalanobis( size_t m, const double *x,
                            const double *mu, const double *E_inv);

/**********/
/* Angles */
/**********/

/// convert radians to degrees
static inline double aa_ang_rad2deg( double rad ) {
    return rad*180.0/M_PI;
}

/// convert radians to degrees
static inline double aa_ang_deg2rad( double deg ) {
    return deg*M_PI/180;
}


/// normalize angle on interval [0,2pi)
static inline double aa_ang_norm_2pi( double an ) {
    return aa_fmodulo( an, 2*M_PI );
}

/// normalize angle on interval (-pi,pi)
static inline double aa_ang_norm_pi( double an ) {
    //return aa_fmodulo( an + M_PI, 2*M_PI ) - M_PI;
    return remainder( an, 2*M_PI );
}


/// Difference between two angles, interval (-pi,pi)
static inline double aa_ang_delta( double a, double b) {
    return aa_ang_norm_pi( aa_ang_norm_pi(a) - aa_ang_norm_pi(b) );
}



/************************/
/* Dense Linear Algebra */
/************************/

/** Pointer to a column of a matrix
 *
 * @param A Matrix pointer
 * @param lda Leading dimension of A
 * @param col column of the matrix (indexed from zero)
 */
#define AA_MATCOL(A, lda, col) ((A)+(col)*(lda))

/** Reference an element in a column-major matrix.
 *
 * @param A Matrix pointer
 * @param lda Leading dimension of A
 * @param row row of the matrix (indexed from zero)
 * @param col column of the matrix (indexed from zero)
 */
#define AA_MATREF(A, lda, row, col) (AA_MATCOL(A,lda,col)[row])

/*--- Scalar Ops ---*/

/** min of vector */
AA_API double aa_la_min( size_t n, const double *x );

/** max of vector */
AA_API double aa_la_max( size_t n, const double *x );

/** Dot product.
 * \f[ {\mathbf x}^T  {\mathbf y} \f]
 */
AA_API double aa_la_dot( size_t n, const double *x, const double *y );

/** Euclidean norm of x.
 * \f[ \sqrt{ {\mathbf x}^T {\mathbf y} } \f]
 */
AA_API double aa_la_norm( size_t n, const double *x );

/** Sum of Squared Differences.
 * \f[ \sum_{i=0}^n (x_i-y_i)^2 \f]
 */
AA_API double aa_la_ssd( size_t n, const double *x, const double *y );

/** Euclidean Distance.
 * \f[ \sqrt{\sum_{i=0}^n (x_i-y_i)^2} \f]
 */
AA_API double aa_la_dist( size_t n, const double *x, const double *y );

/*--- Vector Ops ---*/

/** vector-scalar addition.
 * \f[ r_i \leftarrow \alpha + x_i \f]
 */
AA_API void aa_la_sadd( size_t n, double alpha, const double *x, double *r );


/** vector-scalar multiplication.
 * \f[ x_i \leftarrow \alpha * x_i \f]
 */
AA_API void aa_la_scal( size_t n, double alpha, double *x  );


/** increment by vector.
 * \f[ y_i \leftarrow x_i + y_i \f]
 */
AA_API void aa_la_vinc( size_t n, const double *x, double *y  );

/** increment by scalar.
 * \f[ x_i \leftarrow \alpha + x_i \f]
 */
AA_API void aa_la_sinc( size_t n, double alpha, double *x  );


/** increment by scale times vector.
 * \f[ y_i \leftarrow \alpha x_i + y_i \f]
 */
AA_API void aa_la_axpy( size_t n, double alpha, const double *x, double *y  );


/** increment by scale times vector.
 *
 * \f[ z_i \leftarrow \alpha x_i + y_i \f]
 *
 * three address version of the regular axpy
 */
AA_API void aa_la_axpy3( size_t n, double alpha,
                         const double *x, const double *y, double *z );

/** vector-scalar multiplication.
 * \f[ r_i \leftarrow \alpha * x_i \f]
 */
AA_API void aa_la_smul( size_t n, double alpha, const double *x, double *r );

/** vector-scalar subtraction.
 * \f[ r_i \leftarrow \alpha - x_i \f]
 */
AA_API void aa_la_ssub( size_t n, double alpha, const double *x, double *r );

/** vector-scalar division.
 * \f[ r_i \leftarrow \alpha / x_i \f]
 */
AA_API void aa_la_sdiv( size_t n, double alpha, const double *x, double *r );

/** Elementwise addition.
 * \f[ r_i \leftarrow x_i + y_i \f]
 */
AA_API void aa_la_vadd( size_t n, const double *x, const double *y, double *r );

/** Elementwise subtraction.
 * \f[ r_i \leftarrow x_i - y_i \f]
 */
AA_API void aa_la_vsub( size_t n, const double *x, const double *y, double *r );

/** Elementwise multiplication.
 * \f[ r_i \leftarrow x_i * y_i \f]
 */
AA_API void aa_la_vmul( size_t n, const double *x, const double *y, double *r );

/** Elementwise division.
 * \f[ r_i \leftarrow x_i / y_i \f]
 */
AA_API void aa_la_vdiv( size_t n, const double *x, const double *y, double *r );

/** Cross product.
 * \f[ c \leftarrow a \times b \f]
 */
AA_API void aa_la_cross( const double a[3], const double b[3], double c[3] );

/** Make x unit vector.
 * \f[ x \leftarrow \frac{x}{\|x\|}\f]
 */
AA_API void aa_la_normalize( size_t n, double *x );


/** Point Plane Distance
 *
 * \param n size of space
 * \param point vector of length n
 * \param plane vector of length n+1
 */
AA_API double aa_la_point_plane( size_t n,
                                 const double *point, const double *plane );

/*--- Matrix Ops --- */


/// transpose square matrix A in place
AA_API void aa_la_transpose( size_t n, double *A  );
/// transpose m*n matrix A into n*m matrix At
AA_API void aa_la_transpose2( size_t m, size_t n, const double *A, double *At  );

/** Set diagonal of A to x. */
static inline void
aa_la_diag( size_t n, double *A, double x ) {
    for( size_t i = 0; i < n; i ++ )
        A[i*n+i] = x;
}

/** Set A to the identity matrix */
static inline void
aa_la_ident( size_t n, double *A ) {
    AA_MEM_ZERO(A, n*n);
    aa_la_diag(n,A,1.0);
}

/// matrix-vector multiplication
static inline void
aa_la_mvmul( size_t m, size_t n, const double *A, const double *x, double *b ) {
    cblas_dgemv( CblasColMajor, CblasNoTrans, (int)m, (int)n,
                 1.0, A, (int)m,
                 x, 1, 0, b, 1 );
}

/** Weighted inner product.
 *
 * \f[ x^T A y \f]
 */
AA_API double aa_la_wdot( size_t n,
                          const double *x, const double *A, const double *y );


/** Singular Value Decomposition of A.
 *
 * \f[ A =  U \Sigma V^T \f]
 *
 * \param m rows
 * \param n columns
 * \param A \f$A \in \Re^m \times \Re^n\f$, column major
 * \param U \f$U \in \Re^m \times \Re^m\f$, column major.  If null, U
 * is returned in A.  U and Vt cannot both be null.
 * \param S \f$S \in \Re^m min(m,n)\f$, singular values
 * \param Vt \f$V^T \in \Re^n \times \Re^n\f$, singular vectors. If
 * null, Vt is returned in A.  Vt and U cannot both be null.
 */
AA_API int aa_la_svd( size_t m, size_t n, const double *A, double *U, double *S, double *Vt );

/** Inverse of A.
 *
 * \f[ A \leftarrow A^{-1} \f]
 *
 * \param A \f$A \in \Re^n \times \Re^n\f$, column major
 * \param n rows and columns
 */
AA_API int aa_la_inv( size_t n, double *A );


/** Inverse of 3x3 matrix R.
 *
 * This function is generated by Maxima.
 */
AA_API void aa_la_inverse3x3_( const double R[9], double S[9] );


/** Inverse of 3x3 matrix R.
 *
 * wrapper for the maxima-generated function
 */
static inline  void aa_la_inverse3x3( const double R[9], double S[9] ) {
    aa_la_inverse3x3_(R,S);
}


/** Determinant of 3x3 matrix R.
 *
 */
double aa_la_det3x3( const double R[AA_RESTRICT 9] );

/** Trace of a matrix.
 */
AA_API double aa_la_trace( size_t n, const double *A );

/** Damped Pseudo Inverse of A.
 *
 *  \f[ A^\ddagger \leftarrow A^T (AA^T + kI)^{-1} \f]
 *  \f[ A^\ddagger \leftarrow \sum_{i=0}^r \frac{\sigma_i}{{\sigma_i}^2+k} v_i {u_i}^T \f]
 *
 * The above two formulas are equivalent.  This function uses the SVD
 * method shown in the second formula.
 *
 * See "Introduction to inverse kinematics with jacobian transpose,
 * pseudoinverse and damped least squares methods". Buss, S.R. 2004
 *
 * \param m rows of A
 * \param n cols of A
 * \param A \f$ A \in \Re^m\times\Re^n\f$
 * \param A_star \f$ A^\ddagger \in \Re^n\times\Re^m\f$
 * \param k square of damping factor
 */
AA_API void aa_la_dpinv( size_t m, size_t n, double k,  const double *A, double *A_star );


/** Deadzone, Damped Pseudo Inverse of A.
 *
 *  \f[ A^\ddagger \leftarrow \sum_{i=0}^r \frac{\sigma_i}{{\sigma_i}^2+k} v_i {u_i}^T \f]
 *
 * The denominator \f ${\sigma_i}^2 \f$ goes to zero near
 * singularities.  This function fixes the minimum value of the
 * denominator at parameter s2_min.
 *
 * See "Introduction to inverse kinematics with jacobian transpose,
 * pseudoinverse and damped least squares methods". Buss, S.R. 2004
 *
 * \param m rows of A
 * \param n cols of A
 * \param A \f$ A \in \Re^m\times\Re^n\f$
 * \param A_star \f$ A^\ddagger \in \Re^n\times\Re^m\f$
 * \param s2_min Minimum acceptable value for squared Singular Value
 */
AA_API void aa_la_dzdpinv( size_t m, size_t n, double s2_min, const double *A, double *A_star ) ;

/** Damped Least Squares.
 * \f[ x = A^\ddagger b \f]
 *
 * See "Introduction to inverse kinematics with jacobian transpose,
 * pseudoinverse and damped least squares methods". Buss, S.R. 2004
 *
 * \param m rows in A
 * \param n cols in A
 * \param k square of damping factor
 * \param A \f$ A \in \Re^m\times\Re^n \f$
 * \param b \f$ b \in \Re^m \f$
 * \param x \f$ x \in \Re^n \f$
 */
AA_API void aa_la_dls( size_t m, size_t n, double k,  const double *A, const double *b, double *x );


/** Least Squares with Nullspace projection.
 *
 * \f[ x = A^* b + (I-A^\* A)x_p \f]
 *
 * See "Introduction to inverse kinematics with jacobian transpose,
 * pseudoinverse and damped least squares methods". Buss, S.R. 2004
 *
 * \param m rows in A
 * \param n cols in A
 * \param A \f$ A \in \Re^m\times\Re^n \f$
 * \param A* \f$ A \in \Re^m\times\Re^n \f$
 * \param b \f$ b \in \Re^m \f$
 * \param xp \f$ x \in \Re^n \f$
 * \param x \f$ x \in \Re^n \f$
 */
AA_API void aa_la_xlsnp( size_t m, size_t n,
                         const double *A, const double *A_star, const double *b,
                         const double *xp, double *x );



/** Damped Least Squares with Nullspace projection.
 *
 * \f[ x = A^\ddagger b + (I-A^\ddagger A)x_p \f]
 *
 * See "Introduction to inverse kinematics with jacobian transpose,
 * pseudoinverse and damped least squares methods". Buss, S.R. 2004
 *
 * \param m rows in A
 * \param n cols in A
 * \param k square of damping factor
 * \param A \f$ A \in \Re^m\times\Re^n \f$
 * \param b \f$ b \in \Re^m \f$
 * \param xp \f$ x \in \Re^n \f$
 * \param x \f$ x \in \Re^n \f$
 */
AA_API void aa_la_dlsnp( size_t m, size_t n, double k,  const double *A, const double *b, const double *xp, double *x );

/** Linear Least Squares.
 * \f[ b = Ax \f]
 * Solves for x.
 * \param m rows in A
 * \param n cols in A
 * \param p cols in b and x
 * \param A matrix
 * \param b offset matrix
 * \param x solution matrix
 */
AA_API void aa_la_lls( size_t m, size_t n, size_t p, const double *A, const double *b, double *x );

/** Solve the continuous-time Riccati equation.
 *
 *  \f[ A^TX + XA - XBX + C = 0 \f]
 *
 *  See Laub, Alan. "A Schur Method for Solving Algebraic Riccati
 *  Equations".  IEEE Transactions on Automatic Control. Dec 1979.
 *
 */
AA_API int aa_la_care_laub( size_t m, size_t n, size_t p,
                            const double *AA_RESTRICT A,
                            const double *AA_RESTRICT B,
                            const double *AA_RESTRICT C,
                            double *AA_RESTRICT X );


/** Linear interpolation.
    \param n size of the X vectors
    \param t0 independent variable
    \param X0 dependent variable at t0
    \param t1 independent variable
    \param X1 dependent variable at t1
    \param ti independent variable, interpolation point
    \param Xi dependent variable, interpolated values

*/
AA_API void aa_la_linterp( size_t n,
                           double t0, const double *X0,
                           double t1, const double *X1,
                           double ti, double *Xi );


/** Quadratic interpolation.
    \param n size of the X vectors
    \param t0 independent variable
    \param X0 dependent variable at t0
    \param t1 independent variable
    \param X1 dependent variable at t1
    \param t2 independent variable
    \param X2 dependent variable at t2
    \param ti independent variable, interpolation point
    \param Xi dependent variable, interpolated values

*/
AA_API void aa_la_quadterp( size_t n,
                            double t0, const double *X0,
                            double t1, const double *X1,
                            double t2, const double *X2,
                            double ti, double *Xi );

/** Quadratic interpolation, derivative
 */
AA_API void aa_la_quadterp_dx( size_t n,
                               double t0, const double *X0,
                               double t1, const double *X1,
                               double t2, const double *X2,
                               double ti, double *dXi );


/** Convert plane to hessian normal form
 *
 * \param plane Coefficients for plane
 * \param n length of plane vector, size of plane space is n-1
 */
AA_API void aa_la_plane_hessian( size_t n, double *plane );

/** Fit a plane to a set of points.
 *
 * \param m size of space
 * \param n number of points
 * \param points m*n matrix of points, point per column, column major
 * \param plane vector length m+1 to hold plane coefficients in hessian-normal form
 */
AA_API void aa_la_plane_fit( size_t m, size_t n,
                             const double *points, double *plane );

/*--- Systems and Signals --- */

/** Linear simulation step in discrete time
 *
 * \f[ x_1 \leftarrow Ax_0 + Bu \f]
 *
 * \param m state size
 * \param n input size
 * \param A process model, m*m matrix
 * \param B input model, m*n matrix
 * \param u input, n length vector
 * \param x0 initial state, m length vector
 * \param x1 resulting state, m length vector
 */
AA_API void aa_lsim_dstep( size_t m, size_t n,
                           const double *AA_RESTRICT A,
                           const double *AA_RESTRICT B,
                           const double *AA_RESTRICT x0,
                           const double *AA_RESTRICT u,
                           double *AA_RESTRICT x1 );

/** Linear simulation step with euler integration
 *
 * \f[ \dot{x} = Ax_0 + Bu \f]
 * \f[ x_1 \leftarrow x_0 + \Delta t \dot{x} \f]
 *
 * \param m state size
 * \param n input size
 * \param dt time step
 * \param A process model, m*m matrix
 * \param B input model, m*n matrix
 * \param u input, n length vector
 * \param x0 initial state, m length vector
 * \param x1 resulting state, m length vector
 */
AA_API void aa_lsim_estep( size_t m, size_t n,
                           double dt,
                           const double *AA_RESTRICT A,
                           const double *AA_RESTRICT B,
                           const double *AA_RESTRICT x0,
                           const double *AA_RESTRICT u,
                           double *AA_RESTRICT x1 );


/** A "Signal" function.
 *
 * \f[ y = f(cx, x) \f]
 */
typedef void aa_sys_fun( const void *cx,
                         double t, const double *AA_RESTRICT x,
                         double *AA_RESTRICT y );

/** Euler / Runge-Kutta-1 integration.
 *
 * \f[ x_1 \leftarrow dt*dx + x_0 \f]
 *
 */
AA_API void aa_odestep_rk1( size_t n, double dt,
                            const double *AA_RESTRICT dx,
                            const double *AA_RESTRICT x0,
                            double *AA_RESTRICT x1 );


/** Runge-Kutta-2 (Heun's Method) integration.
 *
 * \param n state size
 * \param sys function to integrate
 * \param cx Context struct for sys
 * \param t0 time
 * \param dt time step
 * \param x0 initial state
 * \param x1 integrated state
 */
AA_API void aa_odestep_rk2( size_t n, aa_sys_fun sys, const void *cx,
                            double t0, double dt,
                            const double *AA_RESTRICT x0, double *AA_RESTRICT x1 );

/** Runge-Kutta-4 integration.
 *
 * \param n state size
 * \param sys function to integrate
 * \param cx Context struct for sys
 * \param t0 time
 * \param dt time step
 * \param x0 initial state
 * \param x1 integrated state
 */
AA_API void aa_odestep_rk4( size_t n, aa_sys_fun sys, const void *cx,
                            double t0, double dt,
                            const double *AA_RESTRICT x0, double *AA_RESTRICT x1 );


/** Runge-Kutta-4-5 (Fehlberg Method) integration.
 *
 * See Erwin Fehlberg (1969). Low-order classical Runge-Kutta formulas
 * with step size control and their application to some heat transfer
 * problems. NASA Technical Report 315.
 *
 *
 * \param n state size
 * \param sys function to integrate
 * \param cx Context struct for sys
 * \param t0 time
 * \param dt time step
 * \param x0 initial state
 * \param k derivitive matix,size n*5, initially first column contains sys evaluated at x0
 * \param x4 fourth-order step
 * \param x5 fifth-order step
 *
 * \pre
 *   - First (initial) column of n*5 matrix k contains sys evalutated at x0
 * \post
 *   - First (initial) column of n*5 matrix k unmodified
 */
AA_API void aa_odestep_rkf45( size_t n, aa_sys_fun sys, const void *cx,
                              double t0, double dt,
                              const double *AA_RESTRICT x0,
                              double *AA_RESTRICT k,
                              double *AA_RESTRICT x4,
                              double *AA_RESTRICT x5 );

/** Runge-Kutta-4-5 (Cash-Karp Method) integration.
 *
 * See J. R. Cash, A. H. Karp. "A variable order Runge-Kutta method for
 * initial value problems with rapidly varying right-hand sides", ACM
 * Transactions on Mathematical Software 16: 201-222,
 * 1990.
 *
 * \param n state size
 * \param sys function to integrate
 * \param cx Context struct for sys
 * \param t0 time
 * \param dt time step
 * \param x0 initial state
 * \param k derivitive matix,size n*5, initially first column contains sys evaluated at x0
 * \param x4 fourth-order step
 * \param x5 fifth-order step
 *
 * \pre
 *   - First (initial) column of n*5 matrix k contains sys evalutated at x0
 * \post
 *   - First (initial) column of n*5 matrix k unmodified
 */
AA_API void aa_odestep_rkck45( size_t n, aa_sys_fun sys, const void *cx,
                               double t0, double dt,
                               const double *AA_RESTRICT x0,
                               double *AA_RESTRICT k,
                               double *AA_RESTRICT x4,
                               double *AA_RESTRICT x5 );


/** Runge-Kutta-4-5 (Dormand-Price Method) integration.
 *
 * See Dormand, J. R.; Prince, P. J. (1980), "A family of embedded
 * Runge-Kutta formulae", Journal of Computational and Applied
 * Mathematics 6 (1): 19–26,
 *
 * \param n state size
 * \param sys function to integrate
 * \param cx Context struct for sys
 * \param t0 time
 * \param dt time step
 * \param x0 initial state
 * \param k derivitive matix,size n*6, initially first column contains sys evaluated at x0,
 * overwritten with sys evalutated at x4
 * \param x4 fourth-order step
 * \param x5 fifth-order step
 *
 * \pre
 *   - First (initial) column of n*6 matrix k contains sys evalutated at x0
 * \post
 *   - First (initial) column of n*6 matrix k unmodified
 *   - Sixth (last) column of n*6 matrix k contains sys evalutated at x4
 */
AA_API void aa_odestep_dorpri45( size_t n, aa_sys_fun sys, const void *cx,
                                 double t0, double dt,
                                 const double *AA_RESTRICT x0,
                                 double *AA_RESTRICT k,
                                 double *AA_RESTRICT x4,
                                 double *AA_RESTRICT x5 );


/** Runge-Kutta-2-3 (Bogacki-Shampine Method) integration.
 *
 * See Bogacki, Przemyslaw; Shampine, Lawrence F. (1989), "A 3(2) pair
 * of Runge–Kutta formulas", Applied Mathematics Letters 2 (4)
 *
 * \param n state size
 * \param sys function to integrate
 * \param cx Context struct for sys
 * \param t0 time
 * \param dt time step
 * \param x0 initial state
 * \param k derivitive matix,size n*4, initially first column contains sys evaluated at x0,
 * overwritten with sys evalutated at x4
 * \param x4 fourth-order step
 * \param x5 fifth-order step
 *
 * \pre
 *   - First (initial) column of n*4 matrix k contains sys evalutated at x0
 * \post
 *   - First (initial) column of n*4 matrix k unmodified
 *   - Fourth (last) column of n*4 matrix k contains sys evalutated at x4
 */
AA_API void aa_odestep_rkbs23( size_t n, aa_sys_fun sys, const void *cx,
                               double t0, double dt,
                               const double *AA_RESTRICT x0,
                               double *AA_RESTRICT k,
                               double *AA_RESTRICT x4,
                               double *AA_RESTRICT x5 );


/** Linear simulation step with Runge-Kutta-4 integration
 *
 * \f[ \dot{x} = Ax_0 + Bu \f]
 *
 * \param m state size
 * \param n input size
 * \param dt time step
 * \param A process model, m*m matrix
 * \param B input model, m*n matrix
 * \param u input, n length vector
 * \param x0 initial state, m length vector
 * \param x1 resulting state, m length vector
 */
AA_API void aa_lsim_rk4step( size_t m, size_t n,
                             double dt,
                             const double *AA_RESTRICT A,
                             const double *AA_RESTRICT B,
                             const double *AA_RESTRICT x0,
                             const double *AA_RESTRICT u,
                             double *AA_RESTRICT x1 );


/** Context-struct for function aa_sys_affine.
 */
typedef struct {
    size_t n;   ///< state size
    double *A;  ///< state transition
    double *D;  ///< additive constant
} aa_sys_affine_t;

/** Affine system model function.
 *
 * \f[ \dot{x} = Ax + D \f]
 */
AA_API void aa_sys_affine( const aa_sys_affine_t *cx,
                           double t, const double *AA_RESTRICT x,
                           double *AA_RESTRICT dx );


/*--- GCC Vector Extensions --- */

//typedef doulbe aa_v2df_t __attribute__ (( vector_size(2*sizeof(double)) ));

#endif //AA_MATH_H
