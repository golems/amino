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

/** \file tf.h */
#ifndef AMINO_TF_H
#define AMINO_TF_H


/*********/
/* Types */
/*********/


/// A rotation matrix, column major
typedef double aa_tf_rotmat_t[9];



#define AA_TF_QUAT_W   3
#define AA_TF_QUAT_XYZ 0

/// A quaternion, x,y,z,w order
typedef struct aa_tf_quat {
    union {
        struct {
            double x;
            double y;
            double z;
            double w;
        };
        double data[4];
    };
} aa_tf_quat_t;


typedef struct aa_tf_eulerzyx {
    union {
        struct {
            double y;
            double p;
            double r;
        };
        double data[3];
    };
} aa_tf_eulerzyx_t;


typedef struct {
    union {
        struct {
            double x;
            double y;
            double z;
        };
        double data[3];
    };
} aa_tf_vec_t;

/** A transformation matrix.
 *  The first 9 elements are a column major rotation matrix.
 *  The last 3 elements are the origin vector.
 */
typedef struct aa_tf_tfmat {
    union {
        struct {
            double R[9];
            aa_tf_vec_t t;
        };
        double data[12];
    };
} aa_tf_tfmat_t;




/// index of dual quaternion real part
#define AA_TF_DUQU_REAL 0
/// index of dual quaternion dual part
#define AA_TF_DUQU_DUAL 4

/// index of dual quaternion real w
#define AA_TF_DUQU_REAL_W    (AA_TF_DUQU_REAL + AA_TF_QUAT_W)
/// index of dual quaternion real xyz
#define AA_TF_DUQU_REAL_XYZ  (AA_TF_DUQU_REAL + AA_TF_QUAT_XYZ)

/// index of dual quaternion dual w
#define AA_TF_DUQU_DUAL_W    (AA_TF_DUQU_DUAL + AA_TF_QUAT_W)
/// index of dual quaternion dual xyz
#define AA_TF_DUQU_DUAL_XYZ  (AA_TF_DUQU_DUAL + AA_TF_QUAT_XYZ)



typedef struct aa_tf_duqu {
    union {
        struct {
            aa_tf_quat_t real;
            aa_tf_quat_t dual;
        };
        double data[8];
    };
} aa_tf_duqu_t;


/// a small number
#define AA_TF_EPSILON .0001



#define AA_TF_IDENT_INITIALIZER {1,0,0, 0,1,0, 0,0,1, 0,0,0}
#define AA_TF_ROTMAT_IDENT_INITIALIZER {1,0,0, 0,1,0, 0,0,1}
#define AA_TF_QUAT_IDENT_INITIALIZER {0,0,0,1}
#define AA_TF_AXANG_IDENT_INITIALIZER {1,0,0,0}
#define AA_TF_ROTVEC_IDENT_INITIALIZER {0,0,0}

/// Identity transform
#define AA_TF_IDENT ( (double[12]) AA_TF_IDENT_INITIALIZER )
/// Identity rotation matrix
#define AA_TF_ROTMAT_IDENT ( (double[9] AA_TF_ROTMAT_IDENT_INITIALIZER )
/// Identity quaternion
#define AA_TF_QUAT_IDENT ( (double[4]) AA_TF_QUAT_IDENT_INITIALIZER )
/// Identity axis-angle
#define AA_TF_AXANG_IDENT ( (double[4]) AA_TF_AXANG_IDENT_INITIALIZER )
/// Identity Rotation Vector
#define AA_TF_ROTVEC_IDENT ( (double[3]) AA_TF_ROTVEC_IDENT_INITIALIZER )

static const double aa_tf_ident[12] = AA_TF_IDENT_INITIALIZER;
static const double aa_tf_rotmat_ident[9] = AA_TF_ROTMAT_IDENT_INITIALIZER;
static const double aa_tf_quat_ident[4] = AA_TF_QUAT_IDENT_INITIALIZER;
static const double aa_tf_axang_ident[4] = AA_TF_AXANG_IDENT_INITIALIZER;
static const double aa_tf_rotvec_ident[3] = AA_TF_ROTVEC_IDENT_INITIALIZER;

/**************/
/* Transforms */
/**************/

/// apply a euclidean transform
AA_API void aa_tf_12( const double T[AA_RESTRICT 12],
                      const double p0[AA_RESTRICT 3],
                      double p1[AA_RESTRICT 3] );
/// apply a euclidean transform
AA_API void aa_tf_93( const double R[AA_RESTRICT 9],
                      const double v[AA_RESTRICT 3],
                      const double p0[AA_RESTRICT 3],
                      double p1[AA_RESTRICT 4] );
/// apply a euclidean transform
AA_API void aa_tf_qv( const double quat[AA_RESTRICT 4],
                      const double v[AA_RESTRICT 3],
                      const double p0[AA_RESTRICT 3],
                      double p1[AA_RESTRICT 4] );

/// apply a euclidean transform
AA_API void aa_tf_9( const double R[AA_RESTRICT 9],
                     const double p0[AA_RESTRICT 3],
                     double p1[AA_RESTRICT 4] );

/// invert transform
AA_API void aa_tf_12inv( const double T[AA_RESTRICT 12],
                         double Ti[AA_RESTRICT 12] );
/// invert transform
AA_API void aa_tf_93inv( const double R[AA_RESTRICT 9],
                      const double v[AA_RESTRICT 3],
                         double Ri[AA_RESTRICT 9], double vi[AA_RESTRICT 3] );
/// invert transform
AA_API void aa_tf_q3inv( double q[AA_RESTRICT 4], double v[AA_RESTRICT 3] );

/// chain two transforms
AA_API void aa_tf_12chain( const double T1[AA_RESTRICT 12],
                           const double T2[AA_RESTRICT 12],
                           double T[AA_RESTRICT 12] );

/// Varargs transform chain
AA_API void aa_tf_v12chain( double T[AA_RESTRICT 12] ,
                            const double T1[AA_RESTRICT 12],
                            const double T2[AA_RESTRICT 12],
                            ... );

/// chain two transforms
AA_API void aa_tf_93chain( const double R0[AA_RESTRICT 9],
                           const double v0[AA_RESTRICT 3],
                           const double R1[AA_RESTRICT 9],
                           const double v1[AA_RESTRICT 3],
                           double R[AA_RESTRICT 9], double v[AA_RESTRICT 3] );
/// chain two transforms
AA_API void aa_tf_q3chain( const double q0[AA_RESTRICT 4],
                           const double v0[AA_RESTRICT 3],
                           const double q1[AA_RESTRICT 4],
                           const double v1[AA_RESTRICT 3],
                           double q[AA_RESTRICT 4],
                           double v[AA_RESTRICT 3] );

/// relative transform
AA_API void aa_tf_93rel( const double R1[AA_RESTRICT 9],
                         const double v1[AA_RESTRICT 3],
                         const double R2[AA_RESTRICT 9],
                         const double v2[AA_RESTRICT 3],
                         double Rrel[AA_RESTRICT 9],
                         double vrel[AA_RESTRICT 3] );

/// relative transform
AA_API void aa_tf_12rel( const double T1[AA_RESTRICT 12],
                         const double T2[AA_RESTRICT 12],
                         double Trel[AA_RESTRICT 12] );

/*********************/
/* Rotation Matrices */
/*********************/

/// tests if R is a rotation matrix
AA_API int aa_tf_isrotmat( const double R[AA_RESTRICT 9] );


/// Multiply two rotation matrices
AA_API void aa_tf_9mul( const double R0[AA_RESTRICT 9],
                        const double R1[AA_RESTRICT 9],
                        double R[AA_RESTRICT 9] );
/// rotate p0 by R
AA_API void aa_tf_9rot( const double R[AA_RESTRICT 9],
                        const double p0[AA_RESTRICT 3],
                        double p1[AA_RESTRICT 3] );

/// relative transform from R1 to R2
AA_API void aa_tf_9rel( const double R1[AA_RESTRICT 9], const double R2[AA_RESTRICT 9],
                        double Ri[AA_RESTRICT 9] );



/// Vararg multiply two rotation matrices
AA_API void aa_tf_v9mul( double R[AA_RESTRICT 9],
                         const double R1[AA_RESTRICT 9], const double R2[AA_RESTRICT 9], ... );

/***************/
/* Quaternions */
/***************/

/** Dual quaternion normalization */
AA_API void aa_tf_cross( const double a[AA_RESTRICT 3], const double b[AA_RESTRICT 3],
                         double c[AA_RESTRICT 3] ) ;


/** Normalize Quaternion.
 * \f[ \bf{q} \leftarrow \frac{\bf q}{\Arrowvert {\bf q} \Arrowvert} \f]
 */
AA_API void aa_tf_qnormalize( double q[AA_RESTRICT 4] );

/** Normailize quaternion.
 */
AA_API void aa_tf_qnormalize2( const double q[AA_RESTRICT 4], double qnorm[AA_RESTRICT 4] );

/** Quaternion conjugate */
AA_API void aa_tf_qconj( const double q[AA_RESTRICT 4],
                         double r[AA_RESTRICT 4] );

/** Quaternion inverse */
AA_API void aa_tf_qinv( const double q[AA_RESTRICT 4],
                        double r[AA_RESTRICT 4] );

/** Quaternion addition. */
AA_API void aa_tf_qadd( const double a[AA_RESTRICT 4],
                        const double b[AA_RESTRICT 4],
                        double c[AA_RESTRICT 4] );

/** Quaternion subtraction. */
AA_API void aa_tf_qsub( const double a[AA_RESTRICT 4],
                        const double b[AA_RESTRICT 4],
                        double c[AA_RESTRICT 4] );

/** Quaternion multiplication. */
AA_API void aa_tf_qmul( const double a[AA_RESTRICT 4],
                        const double b[AA_RESTRICT 4],
                        double c[AA_RESTRICT 4] );

/** Quaternion point rotation. */
void aa_tf_qrot( const double q[AA_RESTRICT 4],
                 const double v[AA_RESTRICT 3],
                 double p[AA_RESTRICT 3] );

/** Relative orientation.
    \f[ q_{\rm rel} = q_1 q_2^{-1} \f]
 */
AA_API void aa_tf_qrel(const double q1[AA_RESTRICT 4],
                       const double q2[AA_RESTRICT 4],
                       double q_rel[AA_RESTRICT 4]);

/** Quaternion SLERP. */
AA_API void aa_tf_qslerp( double tau, const double a[AA_RESTRICT 4],
                          const double b[AA_RESTRICT 4],
                          double c[AA_RESTRICT 4] );


/** Quaternion SLERP, computed algebraicly. */
AA_API void aa_tf_qslerpalg( double tau, const double a[AA_RESTRICT 4],
                             const double b[AA_RESTRICT 4],
                             double c[AA_RESTRICT 4] );

/** Derivative of quaternation SLERP WRT tau.
 *
 * This is NOT a time derivative.  Use the chain rule:
 * (dq/dt = * (dq/dtau)*(dt/dtau) )
 */
AA_API void aa_tf_qslerpdiff( double tau, const double a[AA_RESTRICT 4],
                              const double b[AA_RESTRICT 4],
                              double c[AA_RESTRICT 4] );


/** Derivative of quaternation SLERP WRT tau, computed algebraicly
 */
AA_API void aa_tf_qslerpdiffalg( double tau, const double a[AA_RESTRICT 4],
                                 const double b[AA_RESTRICT 4],
                                 double c[AA_RESTRICT 4] );

/** Chain-rule slerp differentiation */
AA_API void aa_tf_qslerpchaindiff( double u, double du,
                                   const double q1[AA_RESTRICT 4], const double dq1[AA_RESTRICT 4],
                                   const double q2[AA_RESTRICT 4], const double dq2[AA_RESTRICT 4],
                                   double q[AA_RESTRICT 4], double dq[AA_RESTRICT 4] );

/** Triad sequence of slerp differentiation */
AA_API void aa_tf_qslerp3diff( double u12, double du12,
                               const double q1[AA_RESTRICT 4], const double q2[AA_RESTRICT 4],
                               double u34, double du34,
                               const double q3[AA_RESTRICT 4], const double q4[AA_RESTRICT 4],
                               double u, double du,
                               double q[AA_RESTRICT 4], double dq[AA_RESTRICT 4] );

/* Quaternaion time derivate to angular velocity */
AA_API void aa_tf_qdiff2vel( const double q[AA_RESTRICT 4],
                             const double dq_dt[AA_RESTRICT 4],
                             double v[AA_RESTRICT 3] );

/* Angular velocity to quaternion time derivative */
AA_API void aa_tf_qvel2diff( const double q[AA_RESTRICT 4],
                             const double v[AA_RESTRICT 3],
                             double dq_dt[AA_RESTRICT 4] );



/** Integrate unit quaternion, Runge-Kutta-1 (euler) integration.
 *
 * \param q0 Initial rotation quaternion
 * \param dq Quaternion derivative
 * \param dt Time step
 * \param q1 Final rotation quaternion
 **/
AA_API void aa_tf_qrk1( const double q0[AA_RESTRICT 4],
                           const double dq[AA_RESTRICT 4],
                           double dt,
                           double q1[AA_RESTRICT 4] );

/** Integrate unit quaternion from angular velocity, Runge-Kutta-1
 *  (euler) integration.
 *
 * \param q0 Initial rotation quaternion
 * \param v Rotational velocity
 * \param dt Time step
 * \param q1 Final rotation quaternion
 **/
AA_API void aa_tf_qvelrk1( const double q0[AA_RESTRICT 4],
                           const double v[AA_RESTRICT 3],
                           double dt,
                           double q1[AA_RESTRICT 4] );


/** Integrate unit quaternion from angular velocity, Runge-Kutta-4
 *  (euler) integration.
 *
 * \param q0 Initial rotation quaternion
 * \param v Rotational velocity
 * \param dt Time step
 * \param q1 Final rotation quaternion
 **/
AA_API void aa_tf_qvelrk4( const double q0[AA_RESTRICT 4],
                           const double v[AA_RESTRICT 3],
                           double dt,
                           double q1[AA_RESTRICT 4] );


/** Integrate unit quaternion from constant angular velocity
 *
 * \param q0 Initial rotation quaternion
 * \param v Rotational velocity
 * \param dt Time step
 * \param q1 Final rotation quaternion
 **/
AA_API void aa_tf_qsvel( const double q0[AA_RESTRICT 4],
                         const double v[AA_RESTRICT 3],
                         double dt,
                         double q1[AA_RESTRICT 4] );

/*********/
/* Axang */
/*********/


/// copy x,y,z,theta into axang
AA_API void aa_tf_axang_make( double x, double y, double z, double theta,
                              double axang[AA_RESTRICT 4] );

/** Scales angle by k * 2 * pi.
 */
AA_API void aa_tf_axang_permute( const double rv[AA_RESTRICT 4], int k,
                                 double rv_p[AA_RESTRICT 4] );

/// find alternate equivalent representations of rv
AA_API void aa_tf_rotvec_permute( const double rv[AA_RESTRICT 3], int k,
                                  double rv_p[AA_RESTRICT 3] );

/** Scales rv by multiple of 2pi to minimized SSD with rv_near.
 */
AA_API void aa_tf_rotvec_near( const double rv[AA_RESTRICT 3],
                               const double rv_near[AA_RESTRICT 3],
                               double rv_p[AA_RESTRICT 3] );

/***************/
/* Conversions */
/***************/

/** Quaternion to axis-angle. */
AA_API void aa_tf_quat2axang( const double q[AA_RESTRICT 4],
                              double axang[AA_RESTRICT 4] );


/** axis-angle to quaternion. */
AA_API void aa_tf_axang2quat( const double axang[AA_RESTRICT 4],
                              double q[AA_RESTRICT 4] );



/// convert axis-angle to rotation vector
AA_API void aa_tf_axang2rotvec( const double axang[AA_RESTRICT 4],
                                double rotvec[AA_RESTRICT 3] );

/// convert rotation vector to axis-angle
AA_API void aa_tf_rotvec2axang( const double rotvec[AA_RESTRICT 3],
                                double axang[AA_RESTRICT 4] );

/// covert rotation vector to quaternion
AA_API void aa_tf_rotvec2quat( const double rotvec[AA_RESTRICT 3],
                               double q[AA_RESTRICT 4] );
/// covert quaternion to rotation vector
AA_API void aa_tf_quat2rotvec( const double q[AA_RESTRICT 4],
                               double rotvec[AA_RESTRICT 3] );


/// covert quaternion to rotation vector minimizing distance from rv_near
AA_API void aa_tf_quat2rotvec_near( const double q[AA_RESTRICT 4],
                                    const double rv_near[AA_RESTRICT 3],
                                    double rotvec[AA_RESTRICT 3] );

/// convert quaternion to rotation matrix
AA_API void aa_tf_quat2rotmat( const double quat[AA_RESTRICT 4],
                               double rotmat[AA_RESTRICT 9] );
/// convert rotation matrix to quaternion
AA_API void aa_tf_rotmat2quat( const double rotmat[AA_RESTRICT 9],
                               double quat[AA_RESTRICT 4] );

/// convert rotation matrix to axis angle
AA_API void aa_tf_rotmat2axang( const double R[AA_RESTRICT 9],
                                double ra[AA_RESTRICT 4] );
/// convert axis rotation matrix to rotation vector
AA_API void aa_tf_rotmat2rotvec( const double R[AA_RESTRICT 9],
                                 double rv[AA_RESTRICT 3] );

/// convert axis angle to rotation matrix
AA_API void aa_tf_axang2rotmat( const double ra[AA_RESTRICT 4],
                                double R[AA_RESTRICT 9] );
/// convert rotatoin vector to rotation matrix
AA_API void aa_tf_rotvec2rotmat( const double rv[AA_RESTRICT 3],
                                 double R[AA_RESTRICT 9] );

/* AA_API void aa_tf_tfv2tfq( const double vrv[AA_RESTRICT 6],  */
/*                            double x[AA_RESTRICT 3], double quat[AA_RESTRICT 4] ); */
/* AA_API void aa_tf_tfq2tfv( const double x[AA_RESTRICT 3], const double quat[AA_RESTRICT 4],  */
/*                            double vrv[AA_RESTRICT 6] ); */

/* Convert ZYX Euler Angles to Rotation Matrix */
/* AA_API void aa_tf_eulerzyx2rotmat( const double e[AA_RESTRICT 3], */
/*                                    double R[AA_RESTRICT 9] ); */

/** Convert Rotation Matrix to ZYX Euler Angles */
AA_API void aa_tf_rotmat2eulerzyx( const double R[AA_RESTRICT 9],
                                   double e[AA_RESTRICT 3] );


/* AA_API void aa_tf_eulerzyx2quat( const double e[AA_RESTRICT 3], */
/*                                  double q[AA_RESTRICT 4] ); */


#define AA_TF_DEF_EULER(letters)                                        \
    AA_API void                                                         \
    aa_tf_euler ## letters ## 2rotmat( double e1, double e2, double e3, \
                                       double R[AA_RESTRICT 9] );       \
    AA_API void                                                         \
    aa_tf_euler ## letters ## 2quat( double e1, double e2, double e3,   \
                                    double q[AA_RESTRICT 4] );

AA_TF_DEF_EULER( xyz )
AA_TF_DEF_EULER( xzy )

AA_TF_DEF_EULER( yxz )
AA_TF_DEF_EULER( yzx )

AA_TF_DEF_EULER( zyx )
AA_TF_DEF_EULER( zxy )

AA_TF_DEF_EULER( xyx )
AA_TF_DEF_EULER( xzx )

AA_TF_DEF_EULER( yxy )
AA_TF_DEF_EULER( yzy )

AA_TF_DEF_EULER( zxz )
AA_TF_DEF_EULER( zyz )

/** Angle about x axis */
AA_API void aa_tf_xangle2rotmat( double theta_x, double R[AA_RESTRICT 9] );
/** Angle about y axis */
AA_API void aa_tf_yangle2rotmat( double theta_y, double R[AA_RESTRICT 9] );
/** Angle about z axis */
AA_API void aa_tf_zangle2rotmat( double theta_z, double R[AA_RESTRICT 9] );



/* Dual Quaternions */

/** Dual quaternion addition */
AA_API void aa_tf_duqu_add( const double d1[AA_RESTRICT 8], const double d2[AA_RESTRICT 8],
                            double d3[AA_RESTRICT 8] );

/** Dual quaternion scalar multiplication */
AA_API void aa_tf_duqu_smul( const double d1[AA_RESTRICT 8], const double d2[AA_RESTRICT 8],
                             double d3[AA_RESTRICT 8] );

/** Dual quaternion multiplication */
AA_API void aa_tf_duqu_mul( const double d1[AA_RESTRICT 8], const double d2[AA_RESTRICT 8],
                            double d3[AA_RESTRICT 8] );

/** Dual quaternion conjugate */
AA_API void aa_tf_duqu_conj( const double d[AA_RESTRICT 8], double dconj[AA_RESTRICT 8] );

/** Dual quaternion norm */
AA_API void aa_tf_duqu_norm( const double d[AA_RESTRICT 8], double *nreal, double *ndual );

/** Dual quaternion normalization */
AA_API void aa_tf_duqu_normalize( double d[AA_RESTRICT 8] );


/** Dual quaternion transformation */
AA_API void aa_tf_duqu( const double d[AA_RESTRICT 8], const double p0[AA_RESTRICT 3],
                        double p1[AA_RESTRICT 3]  );


/** Dual quaternion normalization */
AA_API void aa_tf_duqu_trans( const double d[AA_RESTRICT 8], double v[AA_RESTRICT 3] );

/** Dual quaternion normalization */
AA_API void aa_tf_duqu2ftmat( const double d[AA_RESTRICT 8], double T[AA_RESTRICT 12] );

/** Dual quaternion normalization */
AA_API void aa_tf_tfmat2duqu( const double T[AA_RESTRICT 12], double d[AA_RESTRICT 8] ) ;

/** Dual quaternion normalization */
AA_API void aa_tf_qv2duqu( const double q[AA_RESTRICT 4], const double v[AA_RESTRICT 3],
                           double d[AA_RESTRICT 8] ) ;

/** Dual quaternion derivative from velocity */
AA_API void aa_tf_duqu_vel2diff( const double d[AA_RESTRICT 8], const double dx[AA_RESTRICT 6],
                                 double dd[AA_RESTRICT 8] ) ;


/** Dual quaternion derivative to spatial velocity */
AA_API void aa_tf_duqu_diff2vel( const double d[AA_RESTRICT 8], const double dd[AA_RESTRICT 8],
                                 double dx[AA_RESTRICT 6] ) ;


/** Dual quaternion integration */
AA_API void aa_tf_duqu_svel( const double d0[AA_RESTRICT 8], const double dd[AA_RESTRICT 8],
                             double dt, double d1[AA_RESTRICT 6] ) ;
AA_API void aa_tf_duqu_sdiff( const double d0[AA_RESTRICT 8], const double dd[AA_RESTRICT 8],
                              double dt, double d1[AA_RESTRICT 6] ) ;

#endif //AMINO_TF_H
