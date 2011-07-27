/* -*- mode: C; c-basic-offset: 4  -*- */
/* ex: set shiftwidth=4 expandtab: */
/*
 * Copyright (c) 2010, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef AA_MATH_H
#define AA_MATH_H

/**
 * \file amino/aa_math.h
 */

/***********/
/* Scalars */
/***********/

#ifndef AA_EPSILON
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


AA_DEPRECATED static inline double aa_clamp( double val, double level) {
    if( val > level )  return level;
    if( val < -level ) return -level;
    return val;
}

static inline double aa_fclamp( double val, double min, double max) {
    if( val > max )  return max;
    if( val < min ) return min;
    return val;
}

static inline void aa_vclamp( size_t n, double *v, double min, double max) {
    for( size_t i = 0; i < n; i++ ) {
        if( v[i] > max )  v[i] = max;
        else if( v[i] < min ) v[i] = min;
    }
}


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
static inline int aa_imodulo( int a, int b ) {
    return ((a % b) + b) % b;
}

static inline long aa_lmodulo( long a, long b ) {
    return ((a % b) + b) % b;
}

/// Fortran modulo, Ada mod
static inline int64_t aa_imodulo64( int64_t a, int64_t b ) {
    return ((a % b) + b) % b;
}

/// Fortran mod, Ada rem
static inline int aa_iremainder( int a, int b ) {
    return a % b;
}

/// Fortran modulo, Ada mod
static inline double aa_fmodulo( double a, double b ) {
    return fmod(fmod(a , b) + b,  b);
}

/// Fortran mod, Ada rem
static inline double aa_fremainder( double a, double b ) {
    return fmod(a , b);
}

/** Returns index of minimum element in array v.
 */
AA_API size_t aa_fminloc( size_t n, double *v );

/** Returns index of maximum element in array v.
 */
AA_API size_t aa_fmaxloc( size_t n, double *v );

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
    return aa_fmodulo( an + M_PI, 2*M_PI ) - M_PI;
}

/************************/
/* Dense Linear Algebra */
/************************/

/** Reference an element in a column-major matrix. */
#define AA_MATREF(ptr, n, i, j) ((ptr)[(j)*(n)+(i)])

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

/** increment by vector.
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

static inline void
aa_la_ident( size_t n, double *A ) {
    aa_fset(A, 0, n*n);
    aa_la_diag(n,A,1.0);
}

// matrix-vector multiplication
static inline void
aa_la_mvmul( size_t m, size_t n, const double *A, const double *x, double *b ) {
    cblas_dgemv( CblasColMajor, CblasNoTrans, (int)m, (int)n,
                 1.0, A, (int)m,
                 x, 1, 0, b, 1 );
}

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
 * This function is generated by Maxima.
 */
AA_API void aa_la_det3x3_( const double R[9], double *d );


/** Determinant of 3x3 matrix R.
 *
 * wrapper for the maxima-generated function
 */
static inline double aa_la_det3x3( const double R[9] ) {
    double d;
    aa_la_det3x3_(R,&d);
    return d;
}

/** Trace of a matrix.
 */
AA_API double aa_la_trace( size_t n, const double *A );

/** Damped Pseudo Inverse of A.
 *  \f[ A^\ddagger \leftarrow A^T (AA^T + kI)^{-1} \f]
 *  \f[ A^\ddagger \leftarrow \sum_{i=0}^r \frac{\sigma_i}{{\sigma_i}^2+k} v_i {u_i}^T \f]
 *
 * The above two formulas are equivalent.  This function uses the SVD
 * method shown in the second formula.
 * \param m rows of A
 * \param n cols of A
 * \param A \f$ A \in \Re^m\times\Re^n\f$
 * \param A_star \f$ A^\ddagger \in \Re^n\times\Re^m\f$
 * \param k square of damping factor
 */
AA_API void aa_la_dpinv( size_t m, size_t n, double k,  const double *A, double *A_star );

/** Damped Least Squares.
 * \f[ x = A^\ddagger b \f]
 *
 * \param m rows in A
 * \param n cols in A
 * \param k square of damping factor
 * \param A \f$ A \in \Re^m\times\Re^n \f$
 * \param b \f$ b \in \Re^m \f$
 * \param x \f$ x \in \Re^n \f$
 */
AA_API void aa_la_dls( size_t m, size_t n, double k,  const double *A, const double *b, double *x );

/** Damped Least Squares with Nullspace projection.
 * \f[ x = A^\ddagger b + (I-A^\ddagger A)x_p \f]
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
 */
AA_API void aa_la_lls( size_t m, size_t n, size_t p, const double *A, const double *b, double *x );


AA_API int aa_la_care_laub( size_t m, size_t n, size_t p,
                            const double *AA_RESTRICT A, const double *AA_RESTRICT B, const double *AA_RESTRICT C,
                            double *AA_RESTRICT X, aa_region_t *reg );


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


#endif //AA_MATH_H
