/* -*- mode: C; c-basic-offset: 4  -*- */
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

/// Fuzzy equals
int aa_feq( double a, double b, double tol );

/// Fuzzy equals
int aa_veq( size_t n, double *a, double *b, double tol );


/// Fortran modulo, Ada mod
int aa_imodulo( int a, int b );

/// Fortran mod, Ada rem
int aa_irem( int a, int b );

/// Fortran modulo, Ada mod
double aa_fmodulo( double a, double b );

/// Fortran mod, Ada rem
double aa_frem( double a, double b );

/**********/
/* Angles */
/**********/

/// convert radians to degrees
double aa_an_rad2deg( double rad );

/// convert radians to degrees
double aa_an_deg2rad( double deg );


/// normalize angle on interval [0,2pi)
double aa_an_norm_2pi( double an );

/// normalize angle on interval (-pi,pi)
double aa_an_norm_pi( double an );

/************************/
/* Dense Linear Algebra */
/************************/

/*--- Scalar Ops ---*/

/** Dot product.
 * \f[ {\mathbf x}^T  {\mathbf y} \f]
 */
double aa_la_dot( size_t n, const double *x, const double *y );

/** Euclidean norm of x.
 * \f[ \sqrt{ {\mathbf x}^T {\mathbf y} } \f]
 */
double aa_la_norm( size_t n, const double *x );

/** Sum of Squared Differences.
 * \f[ \sum_{i=0}^n (x_i-y_i)^2 \f]
 */
double aa_la_ssd( size_t n, const double *x, const double *y );

/** Euclidean Distance.
 * \f[ \sqrt{\sum_{i=0}^n (x_i-y_i)^2} \f]
 */
double aa_la_dist( size_t n, const double *x, const double *y );

/*--- Vector Ops ---*/

/** vector-scalar addition.
 * \f[ r_i \leftarrow \alpha + x_i \f]
 */
void aa_la_sadd( size_t n, double alpha, const double *x, double *r );

/** vector-scalar multiplication.
 * \f[ r_i \leftarrow \alpha * x_i \f]
 */
void aa_la_smul( size_t n, double alpha, const double *x, double *r );

/** vector-scalar subtraction.
 * \f[ r_i \leftarrow \alpha - x_i \f]
 */
void aa_la_ssub( size_t n, double alpha, const double *x, double *r );

/** vector-scalar division.
 * \f[ r_i \leftarrow \alpha / x_i \f]
 */
void aa_la_sdiv( size_t n, double alpha, const double *x, double *r );

/** Elementwise addition.
 * \f[ r_i \leftarrow x_i + y_i \f]
 */
void aa_la_vadd( size_t n, const double *x, const double *y, double *r );

/** Elementwise subtraction.
 * \f[ r_i \leftarrow x_i - y_i \f]
 */
void aa_la_vsub( size_t n, const double *x, const double *y, double *r );

/** Elementwise multiplication.
 * \f[ r_i \leftarrow x_i * y_i \f]
 */
void aa_la_vmul( size_t n, const double *x, const double *y, double *r );

/** Elementwise division.
 * \f[ r_i \leftarrow x_i / y_i \f]
 */
void aa_la_vdiv( size_t n, const double *x, const double *y, double *r );

/** Cross product.
 * \f[ c \leftarrow a \times b \f]
 */
void aa_la_cross( const double a[3], const double b[3], double c[3] );

/*--- Matrix Ops --- */

/** Inverse of A.
 *
 * \f[ A \leftarrow A^{-1} \f]
 *
 * \param A \f$A \in \Re^m \times \Re^n\f$, column major
 * \param m rows
 * \param n columns
 */
int aa_la_inv(size_t m, size_t n, double *A );

/** Damped Least Squares Inverse of A.
 *  \f[ A^* \leftarrow A^T (AA^T + kI)^{-1} \f]
 */
void aa_la_dls( size_t m, size_t n, double k,  const double *A, double *A_star );


/**************/
/* Transforms */
/**************/


/** \brief A transform. */
typedef struct {
    double R[9]; ///< Rotation Matrix
    double t[3]; ///< Translation Vector
} aa_tf_t;


/** Normalize Quaternion */
void aa_tf_qnorm( double q[4] );

/** Quaternion conjugate */
void aa_tf_qconj( const double q[4], double r[4] );

/** Quaternion inverse */
void aa_tf_qinv( const double q[4], double r[4] );

/** Quaternion addition */
void aa_tf_qadd( const double a[4], const double b[4], double c[4] );

/** Quaternion subtraction */
void aa_tf_qadd( const double a[4], const double b[4], double c[4] );

/** Quaternion multiplication */
void aa_tf_qmul( const double a[4], const double b[4], double c[4] );

/** Quaternion SLERP */
void aa_tf_qslerp( double t, const double a[4], const double b[4], double c[4] );

/** Quaternion to axis-angle */
void aa_tf_quat2axang( const double q[4], double axang[4] );

#endif //AA_MATH_H
