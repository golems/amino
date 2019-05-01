/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2010-2014, Georgia Tech Research Corporation
 * Copyright (c) 2015, Rice University
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
 *   * Neither the name of copyright holder the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

#ifndef AMINO_TF_H
#define AMINO_TF_H

#include "mat.h"

/**
 * @file tf.h
 *
 * Low-level operations for SE(3) orientations and transformations.
 *
 * \sa tf.hpp provides a higher-level object-oriented interface which
 * may ease conversion between different representations.
 */

#include <float.h>

#ifdef __cplusplus
extern "C" {
#endif

/**********/
/* Series */
/**********/

/**
 * Define an inline function for a series using Horner's rule.
 *
 * @param name pre-mangled name for the function
 * @param a0 zero-order coefficient
 * @param a1 first-order coefficient
 * @param a2 second-order coefficient
 */
#define AA_TF_DEF_SERIES(name, a0, a1, a2)                  \
    static inline double                                    \
    aa_tf_ ## name ##_series2(double theta2)                \
    {                                                       \
        return aa_horner3( theta2, a0, a1, a2 );            \
    }                                                       \
    static inline double                                    \
    aa_tf_ ## name ## _series(double theta)                 \
    {                                                       \
        return aa_tf_ ## name ## _series2(theta*theta);     \
    }                                                       \

AA_TF_DEF_SERIES( sinc, 1., -1./6, 1./120 )
AA_TF_DEF_SERIES( cos, 1., -1./2, 1./24 )
AA_TF_DEF_SERIES( invsinc, 1., 1./6, 7./360 )

/**
 * Compute sinc(theta), cos(theta), given the square of theta.
 *
 * @param theta2 theta*theta
 * @param sc output for sinc(theta)
 * @param c output for cos(theta)
 */
static inline void
aa_tf_sinccos2( double theta2, double *sc, double *c )
{
    if( theta2 < sqrt(DBL_EPSILON) ) {
        *sc = aa_tf_sinc_series2(theta2);
        *c = aa_tf_cos_series2(theta2);
    } else {
        double theta = sqrt(theta2);
        double s = sin(theta);
        *c = cos(theta);
        *sc = s / theta;
    }
}

/**
 * Compute sinc(theta), cos(theta)
 */
static inline void
aa_tf_sinccos( double theta, double *sc, double *c )
{
    if( theta*theta < sqrt(DBL_EPSILON) ) {
        *sc = aa_tf_sinc_series(theta);
        *c = aa_tf_cos_series(theta);
    } else {
        double s = sin(theta);
        *c = cos(theta);
        *sc = s / theta;
    }
}

/**
 * Compute sinc(theta)
 */
static inline double
aa_tf_sinc( double theta )
{
    if( fabs(theta) < sqrt(sqrt(DBL_EPSILON)) ) {
        return aa_tf_sinc_series(theta);
    } else {
        return sin(theta)/theta;
    }
}

/*********/
/* Types */
/*********/

/// Index of vector x element
#define AA_TF_X 0
/// Index of vector y element
#define AA_TF_Y 1
/// Index of vector z element
#define AA_TF_Z 2

/**
 * Memory layout for a vector of length 3.
 */
typedef struct aa_tf_vec3 {
    union {
        struct {
            double x;   ///< x component
            double y;   ///< y component
            double z;   ///< z component
        };
        double data[3]; ///< data array
    };
} aa_tf_vec3_t;

/**
 * Memory layout for a rotation matrix. column major
 *
 * The matrix is in column-major order.
 */
typedef struct aa_tf_rotmat {
    union {
        struct {
            struct aa_tf_vec3 col0;  ///< column 0
            struct aa_tf_vec3 col1;  ///< column 1
            struct aa_tf_vec3 col2;  ///< column 2
        };
        double data[9]; ///< data array
    };
} aa_tf_rotmat_t;

/**
 * Axis-Angle rotation
 */
typedef struct aa_tf_axang {
    union {
        struct {
            union {
                struct aa_tf_vec3 axis;
                double v[3];
            };
            double angle;
        };
        double data[4];
    };
} aa_tf_axang_t;

/// Index of quaternion vector part
#define AA_TF_QUAT_V 0
/// Index of quaternion vector part
#define AA_TF_QUAT_XYZ AA_TF_QUAT_V
/// Index of quaternion vector x
#define AA_TF_QUAT_X (AA_TF_QUAT_V + AA_TF_X)
/// Index of quaternion vector y
#define AA_TF_QUAT_Y (AA_TF_QUAT_V + AA_TF_Y)
/// Index of quaternion vector z
#define AA_TF_QUAT_Z (AA_TF_QUAT_V + AA_TF_Z)
/// Index of quaternion scalar part
#define AA_TF_QUAT_W 3

/**
 * Memory layout for a quaternion, x,y,z,w order
 */
typedef struct aa_tf_quat {
    union {
        struct {
            double x; ///< x component
            double y; ///< y component
            double z; ///< z component
            double w; ///< w component
        };
        struct {
            union {
                struct aa_tf_vec3 vec; ///< vector part
                double v[3];           ///< vector part
            };
            double scalar; ///< scalar part
        };
        double data[4]; ///< data array
    };
} aa_tf_quat_t;

/**
 * Memory layout for an Euler Angle in ZYX (yaw-pitch-roll) format.
 */
typedef struct aa_tf_eulerzyx {
    union {
        struct {
            double y; ///< yaw
            double p; ///< pitch
            double r; ///< roll
        };
        double data[3]; ///< data array
    };
} aa_tf_eulerzyx_t;


/**
 * Memory layout for a transformation matrix.
 *
 * It is a column-major matrix, but the bottom row is omitted because
 * this row is the same for all SE(3) transformation matrices.
 *
 * The first 9 elements are a column major rotation matrix.
 * The last 3 elements are the origin vector.
 *
 *
 */
typedef struct aa_tf_tfmat {
    union {
        struct {
            double R[9];      ///< the rotation matrix part
            aa_tf_vec3_t v;   ///< the origin vector part
        };
        double data[12]; ///< data array
    };
} aa_tf_tfmat_t;

/**
 * Index of rotation matrix part of transformation matrix.
 */
#define AA_TF_TFMAT_R 0

/**
 * Index of origin part of transformation matrix.
 */
#define AA_TF_TFMAT_V 9

/**
 * Index of origin x component of transformation matrix.
 */
#define AA_TF_TFMAT_X 9

/**
 * Index of origin y component of transformation matrix.
 */
#define AA_TF_TFMAT_Y 10

/**
 * Index of origin z component of transformation matrix.
 */
#define AA_TF_TFMAT_Z 11

/**
 * Memory layout for a Transformation as rotation quaternion and translation vector.
 */
typedef struct aa_tf_qv {
    union {
        struct {
            aa_tf_quat_t r;
            aa_tf_vec3_t v;
        };
        double data[7]; ///< data array
    };
} aa_tf_qv_t;



/**
 * Memory layout for a dual quaternion .
 */
typedef struct aa_tf_duqu {
    union {
        struct {
            aa_tf_quat_t real;
            aa_tf_quat_t dual;
        };
        double data[8]; ///< data array
    };
} aa_tf_duqu_t;

/**
 * Memory layout for an SE(3) velocity
 */
struct aa_tf_dx {
    union {
        struct {
            double dv[3];      ///< translational velocity
            double omega[3];   ///< rotational velocity
        };
        double data[6]; ///< data array
    };
};

/// Index of spatial velocity translational part
#define AA_TF_DX_V 0
/// Index of spatial velocity translational part
#define AA_TF_DX_W 3

/** Transform and spatial velocity */
struct aa_tf_qv_dx {
    union {
        struct {
            struct aa_tf_qv tf; ///< transform
            struct aa_tf_dx dx; ///< velocity
        };
        double data[13];
    };
};

/// Index of quaternion-translation quaternion part
#define AA_TF_QUTR_Q  0
/// Index of quaternion-translation quaternion x
#define AA_TF_QUTR_QX (AA_TF_QUTR_Q + AA_TF_QUAT_X)
/// Index of quaternion-translation quaternion y
#define AA_TF_QUTR_QY (AA_TF_QUTR_Q + AA_TF_QUAT_Y)
/// Index of quaternion-translation quaternion z
#define AA_TF_QUTR_QZ (AA_TF_QUTR_Q + AA_TF_QUAT_Z)
/// Index of quaternion-translation quaternion w
#define AA_TF_QUTR_QW (AA_TF_QUTR_Q + AA_TF_QUAT_W)

/// Index of quaternion-translation translation part
#define AA_TF_QUTR_T  4
/// Index of quaternion-translation translation x
#define AA_TF_QUTR_TX (AA_TF_QUTR_T + AA_TF_X)
/// Index of quaternion-translation translation y
#define AA_TF_QUTR_TY (AA_TF_QUTR_T + AA_TF_Y)
/// Index of quaternion-translation translation z
#define AA_TF_QUTR_TZ (AA_TF_QUTR_T + AA_TF_Z)

/// index of dual quaternion real part
#define AA_TF_DUQU_REAL 0
/// index of dual quaternion dual part
#define AA_TF_DUQU_DUAL 4

/// index of dual quaternion real w
#define AA_TF_DUQU_REAL_W    (AA_TF_DUQU_REAL + AA_TF_QUAT_W)
/// index of dual quaternion real xyz
#define AA_TF_DUQU_REAL_XYZ  (AA_TF_DUQU_REAL + AA_TF_QUAT_XYZ)
/// index of dual quaternion real x
#define AA_TF_DUQU_REAL_X  (AA_TF_DUQU_REAL + AA_TF_QUAT_X)
/// index of dual quaternion real y
#define AA_TF_DUQU_REAL_Y  (AA_TF_DUQU_REAL + AA_TF_QUAT_Y)
/// index of dual quaternion real z
#define AA_TF_DUQU_REAL_Z  (AA_TF_DUQU_REAL + AA_TF_QUAT_Z)

/// index of dual quaternion dual w
#define AA_TF_DUQU_DUAL_W    (AA_TF_DUQU_DUAL + AA_TF_QUAT_W)
/// index of dual quaternion dual xyz
#define AA_TF_DUQU_DUAL_XYZ  (AA_TF_DUQU_DUAL + AA_TF_QUAT_XYZ)
/// index of dual quaternion dual x
#define AA_TF_DUQU_DUAL_X  (AA_TF_DUQU_DUAL + AA_TF_QUAT_X)
/// index of dual quaternion dual y
#define AA_TF_DUQU_DUAL_Y  (AA_TF_DUQU_DUAL + AA_TF_QUAT_Y)
/// index of dual quaternion dual z
#define AA_TF_DUQU_DUAL_Z  (AA_TF_DUQU_DUAL + AA_TF_QUAT_Z)

/// a small number
#define AA_TF_EPSILON .0001



/**
 * Static initializer for an identity transformation matrix.
 */
#define AA_TF_IDENT_INITIALIZER {1,0,0, 0,1,0, 0,0,1, 0,0,0}

/**
 * Static initializer for an identity rotation matrix.
 */
#define AA_TF_ROTMAT_IDENT_INITIALIZER {1,0,0, 0,1,0, 0,0,1}

/**
 * Static initializer for an identity quaternion.
 */
#define AA_TF_QUAT_IDENT_INITIALIZER {0,0,0,1}

/**
 * Static initializer for an identity dual quaternion.
 */
#define AA_TF_DUQU_IDENT_INITIALIZER {0,0,0,1, 0,0,0,0}

/**
 * Static initializer for an identity quaternion-translation.
 */
#define AA_TF_QUTR_IDENT_INITIALIZER {0,0,0,1, 0,0,0}

/**
 * Static initializer for an identity axis-angle.
 */
#define AA_TF_AXANG_IDENT_INITIALIZER {1,0,0,0}

/**
 * Static initializer for an identity rotation-vector.
 */
#define AA_TF_ROTVEC_IDENT_INITIALIZER {0,0,0}

/**
 * Static initializer for an identity vector-3.
 */
#define AA_TF_VEC_IDENT_INITIALIZER {0,0,0}

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

/**
 * Identity transformation matrix array.
 */
static const double aa_tf_ident[12] = AA_TF_IDENT_INITIALIZER;

/**
 * Identity transformation matrix array.
 */
static const double aa_tf_tfmat_ident[12] = AA_TF_IDENT_INITIALIZER;

/**
 * Identity rotation matrix array.
 */
static const double aa_tf_rotmat_ident[9] = AA_TF_ROTMAT_IDENT_INITIALIZER;

/**
 * Identity quaternion array.
 */
static const double aa_tf_quat_ident[4] = AA_TF_QUAT_IDENT_INITIALIZER;

/**
 * Identity dual quaternion array.
 */
static const double aa_tf_duqu_ident[8] = AA_TF_DUQU_IDENT_INITIALIZER;

/**
 * Identity quaternion-translation array.
 */
static const double aa_tf_qutr_ident[7] = AA_TF_QUTR_IDENT_INITIALIZER;

/**
 * Identity axis-angle array.
 */
static const double aa_tf_axang_ident[4] = AA_TF_AXANG_IDENT_INITIALIZER;

/**
 * Identity rotation vector array.
 */
static const double aa_tf_rotvec_ident[3] = AA_TF_ROTVEC_IDENT_INITIALIZER;

/**
 * Identity vector-3 array.
 */
static const double aa_tf_vec_ident[3] = AA_TF_ROTVEC_IDENT_INITIALIZER;

/**
 * An X axis.
 */
static const double aa_tf_vec_x[3] = {1, 0, 0};

/**
 * A Y axis.
 */
static const double aa_tf_vec_y[3] = {0, 1, 0};

/**
 * A Z axis.
 */
static const double aa_tf_vec_z[3] = {0, 0, 1};

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
AA_API void aa_tf_tf_qv( const double quat[AA_RESTRICT 4],
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
/**
 * Invert transform
 *
 * Unit quaternion inverse is equal to the conjuage
 */
AA_API void aa_tf_qv_conj( const double q[AA_RESTRICT 4], const double v[AA_RESTRICT 3],
                           double qc[AA_RESTRICT 4], double vc[AA_RESTRICT 3]  );

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
AA_API void aa_tf_qv_chain( const double q0[AA_RESTRICT 4],
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

/************/
/* Matrices */
/************/

/**
 * Construct a skew-symmetric matrix.
 *
 * @f[
 *     I + a*[u] + b*[u]^2
 * @f]
 *
 * where
 *
 * @f[
 *     [u] = \begin{bmatrix}
 *          0 & -u_2 & u_1 \\
 *          u_2 & 0 & -u_0 \\
 *          -u_1 & u_0 & 0
 *         \end{bmatrix}
 * @f]
 */
AA_API void aa_tf_skewsym_scal2( double a, double b, const double u[3], double R[9] );

/**
 * Construct a skew-symmetric matrix.
 */
AA_API void
aa_tf_skewsym_scal_c( const double u[AA_RESTRICT 3],
                      const double a[AA_RESTRICT 3], const double b[AA_RESTRICT 3],
                      double R[9] );


/* Multiply */

/**
 * Multiple two rotation matrices.
 */
AA_API void aa_tf_rotmat_mul( const double R0[AA_RESTRICT 9],
                              const double R1[AA_RESTRICT 9],
                              double R[AA_RESTRICT 9] );

/**
 * Multiple two transformation matrices.
 */
AA_API void aa_tf_tfmat_mul( const double T0[AA_RESTRICT 12],
                             const double T1[AA_RESTRICT 12],
                             double T[AA_RESTRICT 12] );

/**
 * Multiple two transformation matrices, in split representation.
 */
AA_API void aa_tf_tfmat2_mul( const double R0[AA_RESTRICT 9],
                              const double v0[AA_RESTRICT 3],
                              const double R1[AA_RESTRICT 9],
                              const double v1[AA_RESTRICT 3],
                              double R[AA_RESTRICT 9], double v[AA_RESTRICT 3] );


/* Inverting Multiply */
/**
 * Multiple inverse  of R0 by R1.
 *
 * \f[ R = ({R_0}^{-1}) * (R_1) \f]
 */
AA_API void aa_tf_rotmat_imul( const double R0[AA_RESTRICT 9],
                               const double R1[AA_RESTRICT 9],
                               double R[AA_RESTRICT 9] );

/**
 * Multiple R1 by inverse of R1.
 *
 * \f[ R = ({R_0}) * ({R_1}^{-1}) \f]
 */
AA_API void aa_tf_rotmat_muli( const double R0[AA_RESTRICT 9],
                               const double R1[AA_RESTRICT 9],
                               double R[AA_RESTRICT 9] );

/**
 * Multiple inverse  of T0 by T1.
 *
 * \f[ T = ({T_0}^{-1}) * (T_1) \f]
 */
AA_API void aa_tf_tfmat_imul( const double T0[AA_RESTRICT 12],
                              const double T1[AA_RESTRICT 12],
                              double T[AA_RESTRICT 12] );

/**
 * Multiple T1 by inverse of T1.
 *
 * \f[ T = ({T_0}) * ({T_1}^{-1}) \f]
 */
AA_API void aa_tf_tfmat_muli( const double T0[AA_RESTRICT 12],
                              const double T1[AA_RESTRICT 12],
                              double T[AA_RESTRICT 12] );

/**
 * Multiple inverse of transform (R0,v0) by transform (R1,v1).
 *
 * \f[ (R,v) = \left(\left(R_0,v_0\right)^{-1}\right) * ({R_1},v_1) \f]
 */
AA_API void aa_tf_tfmat2_imul( const double R0[AA_RESTRICT 9],
                               const double v0[AA_RESTRICT 3],
                               const double R1[AA_RESTRICT 9],
                               const double v1[AA_RESTRICT 3],
                               double R[AA_RESTRICT 9], double v[AA_RESTRICT 3] );

/**
 * Multiple (R0,v0) by inverse of transform transform (R1,v1).
 *
 * \f[ (R,v) = ({R_0},v_0) \left(\left(R_1,v_1\right)^{-1}\right) \f]
 */
AA_API void aa_tf_tfmat2_muli( const double R0[AA_RESTRICT 9],
                               const double v0[AA_RESTRICT 3],
                               const double R1[AA_RESTRICT 9],
                               const double v1[AA_RESTRICT 3],
                               double R[AA_RESTRICT 9], double v[AA_RESTRICT 3] );


/**
 * Orthonormalize the rotation matrix.
 */
AA_API void
aa_tf_rotmat_normalize( double R[AA_RESTRICT 9] );

/* Transform */

/**
 * Rotate a point using a rotation matrix.
 */
AA_API void aa_tf_rotmat_rot( const double R[AA_RESTRICT 9],
                              const double p0[AA_RESTRICT 3],
                              double p1[AA_RESTRICT 3] );

/**
 * Transform a point using a transformation matrix.
 */
AA_API void aa_tf_tfmat_tf( const double T[AA_RESTRICT 12],
                            const double p0[AA_RESTRICT 3],
                            double p1[AA_RESTRICT 3] );

/**
 * Transform a point using a split transformation matrix.
 */
AA_API void aa_tf_tfmat2_tf( const double R[AA_RESTRICT 9],
                             const double v[AA_RESTRICT 3],
                             const double p0[AA_RESTRICT 3],
                             double p1[AA_RESTRICT 4] );

/**
 * Invert a rotation in place
 */
AA_API void aa_tf_rotmat_inv1( double R[AA_RESTRICT 9] );

/**
 * Invert a rotation
 */
AA_API void aa_tf_rotmat_inv2( const double R[AA_RESTRICT 9],
                               double Ri[AA_RESTRICT 9] );

/**
 * Invert a transform in place
 */
AA_API void aa_tf_tfmat_inv1( double T[AA_RESTRICT 12] );

/**
 * Invert a transform
 */
AA_API void aa_tf_tfmat_inv2( const double T[AA_RESTRICT 12],
                              double Ti[AA_RESTRICT 12] );


/**
 * Orthonormalize the transformation matrix.
 */
AA_API void
aa_tf_tfmat_normalize( double T[AA_RESTRICT 12] );

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

/// Rotation Matrix exponential from axis angle
AA_API void aa_tf_rotmat_exp_aa( const double aa[AA_RESTRICT 4], double R[AA_RESTRICT 9] );

/// Rotation Matrix exponential from rotation vector
AA_API void aa_tf_rotmat_expv( const double rv[AA_RESTRICT 3], double R[AA_RESTRICT 9] );

/// Rotation Matrix logarithm
AA_API void aa_tf_rotmat_lnv( const double R[AA_RESTRICT 9], double v[AA_RESTRICT 3] );

/// Transformation Matrix exponential
AA_API void aa_tf_tfmat_expv( const double v[AA_RESTRICT 6], double T[AA_RESTRICT 12] );

/// Transformation Matrix logarithm
AA_API void aa_tf_tfmat_lnv( const double T[AA_RESTRICT 12], double v[AA_RESTRICT 6] );

/// Velocity to rotation matrix derivative
AA_API void aa_tf_rotmat_vel2diff( const double R[AA_RESTRICT 9],
                                   const double w[AA_RESTRICT 3], double dR[AA_RESTRICT 9] );

/// Rotation matrix derivative to velocity
AA_API void aa_tf_rotmat_diff2vel( const double R[AA_RESTRICT 9],
                                   const double dR[AA_RESTRICT 9], double w[AA_RESTRICT 3] );

/// Integrate rotational velocity
AA_API void aa_tf_rotmat_svel( const double R0[AA_RESTRICT 9],
                               const double w[AA_RESTRICT 3], double dt,
                               double R1[AA_RESTRICT 9] );



/// Integrate rotational velocity
AA_API void aa_tf_tfmat_svel( const double T0[AA_RESTRICT 12],
                              const double w[AA_RESTRICT 3], double dt,
                              double T1[AA_RESTRICT 12] );

/// Vararg multiply two rotation matrices
AA_API void aa_tf_v9mul( double R[AA_RESTRICT 9],
                         const double R1[AA_RESTRICT 9], const double R2[AA_RESTRICT 9], ... );


/**
 * Construct rotation matrix from an x and y axis of the child frame.
 */
AA_API void
aa_tf_rotmat_xy( const double x_axis[AA_RESTRICT 3],
                 const double y_axis[AA_RESTRICT 3],
                 double R[AA_RESTRICT 9] );
/**
 * Construct rotation matrix from a y and z axis of the child frame.
 */
AA_API void
aa_tf_rotmat_yz( const double y_axis[AA_RESTRICT 3],
                 const double z_axis[AA_RESTRICT 3],
                 double R[AA_RESTRICT 9] );

/**
 * Construct rotation matrix from a z and x axis of the child frame.
 */
AA_API void
aa_tf_rotmat_zx( const double z_axis[AA_RESTRICT 3],
                 const double x_axis[AA_RESTRICT 3],
                 double R[AA_RESTRICT 9] );


/***********/
/* Vectors */
/***********/

/**
 * Macro to compute a dot product of length 3 vectors.
 *
 * May evaluate its arguments multiple times.
 */
#define AA_TF_DOTX(a,b) \
    ((a)[0]*(b)[0] + (a)[1]*(b)[1] + (a)[2]*(b)[2])

/**
 * Inlined vector dot product
 */
static inline double
AA_TF_VDOT( const double a[AA_RESTRICT 3], const double b[AA_RESTRICT 3] )
{
    return AA_TF_DOTX(a,b);
}

/**
 * Inlined vector dot product
 */
static inline float
AA_TF_VDOTF( const float a[AA_RESTRICT 3], const float b[AA_RESTRICT 3] )
{
    return AA_TF_DOTX(a,b);
}


/**
 * Vector dot product.
 */
AA_API double
aa_tf_vdot( const double a[AA_RESTRICT 3], const double b[AA_RESTRICT 3] );

/**
 * Vector dot product.
 */
AA_API float
aa_tf_vdotf( const float a[AA_RESTRICT 3], const float b[AA_RESTRICT 3] );

/**
 * Macro to compute the cross product of length 3 vectors.
 *
 * May evaluate its arguments multiple times.
 */
#define AA_TF_CROSSX( a, b, c ) \
    (c)[0] =  (a)[1]*(b)[2] - (a)[2]*(b)[1]; \
    (c)[1] =  (a)[2]*(b)[0] - (a)[0]*(b)[2]; \
    (c)[2] =  (a)[0]*(b)[1] - (a)[1]*(b)[0]; \


/**
 * Inlined Vector cross product
 */
static inline void
AA_TF_CROSS( const double a[AA_RESTRICT 3], const double b[AA_RESTRICT 3], double c[AA_RESTRICT 3] )
{
    AA_TF_CROSSX(a,b,c);
}

/**
 * Inlined Vector cross product
 */
static inline void
AA_TF_CROSSF( const float a[AA_RESTRICT 3], const float b[AA_RESTRICT 3], float c[AA_RESTRICT 3] )
{
    AA_TF_CROSSX(a,b,c);
}

/**
 * Vector cross product
 */
AA_API void aa_tf_cross( const double a[AA_RESTRICT 3], const double b[AA_RESTRICT 3],
                         double c[AA_RESTRICT 3] ) ;

/**
 * Construct matrix for left cross product
 * v*a = M*a
 */
AA_API void aa_tf_cross_mat_l( const double v[3], struct aa_dmat *M );

/**
 * Construct matrix for right cross product
 * a*v = M*a
 */
AA_API void aa_tf_cross_mat_r( const double v[3], struct aa_dmat *M );



/**
 * Vector cross product
 */
AA_API void aa_tf_crossf( const float a[AA_RESTRICT 3], const float b[AA_RESTRICT 3],
                          float c[AA_RESTRICT 3] ) ;

/**
 * Normalize Vector.
 */
AA_API void aa_tf_vnormalize( double v[AA_RESTRICT 3] );


/**
 * Normalize Vector.
 */
AA_API void aa_tf_vnormalizef( float v[AA_RESTRICT 3] );

/**
 * Sum-square-differences of two vectors
 */
double
aa_tf_vssd( const double a[AA_RESTRICT 3], const double b[AA_RESTRICT 3] );

/**
 * Norm of vector
 */
double
aa_tf_vnorm( const double a[AA_RESTRICT 3] );

/***************/
/* Quaternions */
/***************/

/**
 * Inlined quaternion dot product.
 */
static inline double
AA_TF_QDOT( const double a[AA_RESTRICT 4], const double b[AA_RESTRICT 4] )
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
}

/**
 * Sum-square-differences of two quaternions
 */
double
aa_tf_qssd( const double a[AA_RESTRICT 4], const double b[AA_RESTRICT 4] );

/**
 * Quaternion dot product.
 */
double
aa_tf_qdot( const double a[AA_RESTRICT 4], const double b[AA_RESTRICT 4] );

/** Normalize Quaternion.
 * \f[ \bf{q} \leftarrow \frac{\bf q}{\Arrowvert {\bf q} \Arrowvert} \f]
 */
AA_API void aa_tf_qnormalize( double q[AA_RESTRICT 4] );

/**
 * Scale a quaternion.
 * @f[ \bf{q} \leftarrow \alpha {\bf q} @f]
 */
AA_API void
aa_tf_qscal( double q[AA_RESTRICT 4], double alpha );


/** Return norm of the quaternion */
AA_API double aa_tf_qnorm( const double q[AA_RESTRICT 4] );

/** Minimize angle represented by the quaternion.  This puts the
 *  quaternion in the right-hand side of the complex plane.  Its w
 *  value will be positive.
 */
AA_API void aa_tf_qminimize( double q[AA_RESTRICT 4] );

/** Minimize angle represented by the quaternion.  This puts the
 *  quaternion in the right-hand side of the complex plane.  Its w
 *  value will be positive.
 */
AA_API void aa_tf_qminimize2( const double q[AA_RESTRICT 4], double qmin[AA_RESTRICT 4] );

/** Normailize quaternion.
 */
AA_API void aa_tf_qnormalize2( const double q[AA_RESTRICT 4], double qnorm[AA_RESTRICT 4] );

/** Quaternion conjugate */
AA_API void aa_tf_qconj( const double q[AA_RESTRICT 4],
                         double r[AA_RESTRICT 4] );


/** Quaternion exponential */
AA_API void aa_tf_qexp( const double q[AA_RESTRICT 4],
                        double r[AA_RESTRICT 4] );


/** Quaternion natural log */
AA_API void aa_tf_qln( const double q[AA_RESTRICT 4],
                       double r[AA_RESTRICT 4] );

/** Derivative of the Unit Quaternion Logarithm.
 *
 * \param q A unit quaternion
 * \param dq The derivative of q
 * \param dln The derivative of ln(dq)
 */
AA_API void aa_tf_qduln( const double q[AA_RESTRICT 4],
                         const double dq[AA_RESTRICT 4],
                         double dln[AA_RESTRICT 3] );

/**
 * Derivative of the Unit Quaternion Logarithm, computed via Jacobian.
 *
 * \param q A unit quaternion
 * \param dq The derivative of q
 * \param dln The derivative of ln(dq)
 */
AA_API void aa_tf_qdulnj( const double q[AA_RESTRICT 4],
                         const double dq[AA_RESTRICT 4],
                         double dln[AA_RESTRICT 3] );

/** Derivative of the Pure Quaternion Exponential.
 *
 * \param e A pure quaternion
 * \param de The derivative of e
 * \param dq The derivative of exp(e)
 */
AA_API void aa_tf_qdpexp( const double e[AA_RESTRICT 3],
                          const double de[AA_RESTRICT 3],
                          double dq[AA_RESTRICT 4] );

/**
 * Derivative of the Pure Quaternion Exponential, computed via Jacobian.
 *
 * \param e A pure quaternion
 * \param de The derivative of e
 * \param dq The derivative of exp(e)
 */
AA_API void aa_tf_qdpexpj( const double e[AA_RESTRICT 3],
                          const double de[AA_RESTRICT 3],
                          double dq[AA_RESTRICT 4] );

/** Return the angle of the quaternion.
 *
 * Note that this result is half the 3D angle.
 */
double aa_tf_qangle( const double q[AA_RESTRICT 4] );

/**
 * Relative quaternion angles
 */
double aa_tf_qangle_rel( const double *q, const double *p );

/** Return the angle between unit quaterniosn in 4D space.
 */
double aa_tf_quhypangle2
( const double q[AA_RESTRICT 4], const double p[AA_RESTRICT 4] );

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

/** Quaternion multiplication and normalize. */
AA_API void aa_tf_qmulnorm( const double a[AA_RESTRICT 4],
                            const double b[AA_RESTRICT 4],
                            double c[AA_RESTRICT 4] );

/** Quaternion multiplication. */
AA_API void aa_tf_qmul_qv( const double q[AA_RESTRICT 4],
                           const double v[AA_RESTRICT 3],
                           double c[AA_RESTRICT 4] );

/** Quaternion multiplication. */
AA_API void aa_tf_qmul_vq( const double v[AA_RESTRICT 3],
                           const double q[AA_RESTRICT 4],
                           double c[AA_RESTRICT 4] );

/** Quaternion conjugate a and multiply by b. */
AA_API void aa_tf_qcmul( const double a[AA_RESTRICT 4],
                         const double b[AA_RESTRICT 4],
                         double c[AA_RESTRICT 4] );

/** Quaternion multiply a by conjugate b. */
AA_API void aa_tf_qmulc( const double a[AA_RESTRICT 4],
                         const double b[AA_RESTRICT 4],
                         double c[AA_RESTRICT 4] );


/**
 * Quaternion point rotation, in place.
 */
AA_API void
aa_tf_qrot1( const double q[AA_RESTRICT 4], double v[AA_RESTRICT 3] );

/** Quaternion point rotation. */
AA_API void aa_tf_qrot( const double q[AA_RESTRICT 4],
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

/** Quaternaion time derivate to angular velocity */
AA_API void aa_tf_qdiff2vel( const double q[AA_RESTRICT 4],
                             const double dq_dt[AA_RESTRICT 4],
                             double v[AA_RESTRICT 3] );

/** Angular velocity to quaternion time derivative */
AA_API void aa_tf_qvel2diff( const double q[AA_RESTRICT 4],
                             const double v[AA_RESTRICT 3],
                             double dq_dt[AA_RESTRICT 4] );


/** Convert rotation vector derivative to rotational velocity */
AA_API void aa_tf_rotvec_diff2vel( const double v[3], const double dv[3],
                                   double w[3] );

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


/** Integrate unit quaternion from quaternion derivative.
 *
 * \param q0 Initial rotation quaternion
 * \param dq Quaternion derivative
 * \param dt Time step
 * \param q1 Final rotation quaternion
 **/
AA_API void aa_tf_qsdiff( const double q0[AA_RESTRICT 4],
                          const double dq[AA_RESTRICT 4],
                          double dt,
                          double q1[AA_RESTRICT 4] );


AA_API void aa_tf_qsacc_rk( const double q0[AA_RESTRICT 4],
                            const double v[AA_RESTRICT 3],
                            const double a[AA_RESTRICT 3],
                            double dt,
                            double q1[AA_RESTRICT 4] );

AA_API void aa_tf_qsacc( const double q0[AA_RESTRICT 4],
                         const double v[AA_RESTRICT 3],
                         const double a[AA_RESTRICT 3],
                         double dt,
                         double q1[AA_RESTRICT 4] );


/** Unit quaternion for angle about x axis */
AA_API void aa_tf_xangle2quat( double theta_x, double q[AA_RESTRICT 4] );
/** Unit quaternion for angle about y axis */
AA_API void aa_tf_yangle2quat( double theta_y, double q[AA_RESTRICT 4] );
/** Unit quaternion for angle about z axis */
AA_API void aa_tf_zangle2quat( double theta_z, double q[AA_RESTRICT 4] );

/**
 *  Construct matrix for Davenport's q-method
 */
AA_API void aa_tf_quat_davenport_matrix
( size_t n, const double *w, const double *q, size_t ldqq, double *M );

/**
 *  Weighted average quaternion using Davenport's q-method
 *
 * @param n number of quaternions
 * @param w weights
 * @param Q array of quaternions
 * @param ldq leading dimension of Q
 * @param y average quaternion
 */
AA_API void aa_tf_quat_davenport
( size_t n, const double *w, const double *Q, size_t ldq, double *y );



/** Construct matrix for left quaternion multiply
 * q*p = M*p
 */
AA_API void aa_tf_qmatrix_l( const double *q, double *M, size_t ldm );

/** Construct matrix for right quaternion multiply
 * p*q = M*p
 */
AA_API void aa_tf_qmatrix_r( const double *q, double *M, size_t ldm );


/** Generate random unit quaternion */
void aa_tf_qurand( double q[4] );

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
/**
 * Rotate a point
 */
AA_API void aa_tf_axang_rot( const double axang[AA_RESTRICT 4],
                             const double p[AA_RESTRICT 3],
                             double q[AA_RESTRICT 3] );

/**************/
/* Quat-Trans */
/**************/

/// quaternion-translation conjugate
void aa_tf_qv_conj( const double q[4], const double v[3],
                    double qc[4], double vc[3] );


/**
 * Offset of quaternion part in quaternion-translation representation.
 */
#define AA_TF_QUTR_Q 0

/**
 * Offset of translation part in quaternion-translation
 * representation.
 */
#define AA_TF_QUTR_V 4

/// quaternion-translation to dual quaternion
void aa_tf_qutr2duqu( const double e[7], double s[8] );

/// quaternion-translation to transformation matrix
void aa_tf_qutr2tfmat( const double e[7], double T[12] );

/// transformation matrix to  quaternion-translation
void aa_tf_tfmat2qutr( const double T[12], double e[7] );

/// dual quaternion to quaternion-translation
void aa_tf_duqu2qutr( const double s[8], double e[7] );

/// quaternion-translation multiply
void aa_tf_qutr_mul( const double a[7], const double b[7], double c[7] ) ;

/**
 * Transform a point,
 */
AA_API void
aa_tf_qutr_tf( const double E[AA_RESTRICT 7], const double p0[AA_RESTRICT 3],
               double p1[AA_RESTRICT 3] );

/// quaternion-translation multiply and normalize
void aa_tf_qutr_mulnorm( const double a[7], const double b[7], double c[7] ) ;

/// quaternion-translation conjugate
void aa_tf_qutr_conj( const double a[7], double c[7] ) ;

/// quaternion-translation conjugate multiply
void aa_tf_qutr_mulc( const double a[7], const double b[7], double c[7] ) ;

/// quaternion-translation conjugate multiply
void aa_tf_qutr_cmul( const double a[7], const double b[7], double c[7] ) ;

/**
 * Quaternion-vector exponential.
 */
AA_API void aa_tf_qv_expv
( const double w[3],  const double dv[3],
  double q[4], double v[3] );

/**
 * Quaternion-vector logarithm.
 */
AA_API void aa_tf_qv_lnv
( const double q[4], const double v[3],
  double w[3],  double dv[3] );

/**
 * Quaternion-vector exponential.
 */
AA_API void aa_tf_qutr_expv
( const double w[6], double e[7] );

/**
 * Quaternion-vector logarithm.
 */
AA_API void aa_tf_qutr_lnv
( const double e[7], double w[6] );

/** Quaternion-translation twist to velocity */
void aa_tf_qutr_twist2vel
( const double e[7], const double w[6], double dx[6] );

/** Quaternion-translation derivative to spatial velocity */
AA_API void aa_tf_qutr_diff2vel
( const double e[7], const double de[7], double dx[6] );

/** Quaternion-translation spatial velocity to derivative */
AA_API void aa_tf_qutr_vel2diff
( const double e[7], const double dx[6], double de[7] );



/** Quaternion-vector velocity to twist */
AA_API void
aa_tf_qv_vel2twist( const double q[AA_RESTRICT 4], const double v[AA_RESTRICT 3],
                    const double w[AA_RESTRICT 3], const double dv[AA_RESTRICT 3],
                    double tw[AA_RESTRICT 3], double tv[AA_RESTRICT 3] );

/** Quaternion-vector twist to velocity */
AA_API void
aa_tf_qv_twist2vel( const double q[AA_RESTRICT 4], const double v[AA_RESTRICT 3],
                    const double tw[AA_RESTRICT 3], const double tv[AA_RESTRICT 3],
                    double w[AA_RESTRICT 3], double dv[AA_RESTRICT 3] );

/** Integrate a quaternion-translation */
void aa_tf_qutr_svel
( const double e0[7], const double dx[6], double dt, double e1[7] );

/** Integrate a quaternion-translation */
void aa_tf_qutr_sdiff
( const double e0[7], const double de[7], double dt, double e1[7] );

/** Weighted average transform */
void aa_tf_qutr_wavg
( size_t n, const double *w, const double *EE, size_t ldee, double *a );

/** Generate random transform */
void aa_tf_qutr_rand( double E[7] );

/***************/
/* Conversions */
/***************/

/** Quaternion to axis-angle. */
AA_API void aa_tf_quat2axang( const double q[AA_RESTRICT 4],
                              double axang[AA_RESTRICT 4] );


/** axis-angle to quaternion. */
AA_API void aa_tf_axang2quat( const double axang[AA_RESTRICT 4],
                              double q[AA_RESTRICT 4] );

/** axis-angle to quaternion. */
AA_API void aa_tf_axang2quat2( const double axis[AA_RESTRICT 3],
                               double angle,
                               double q[AA_RESTRICT 4] );


/**
 * Convert rotation between two vectors to a quaternion
 */
AA_API void
aa_tf_vecs2quat( const double u[AA_RESTRICT 3],
                 const double v[AA_RESTRICT 3],
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

/// convert axis angle to rotation matrix
AA_API void aa_tf_axang2rotmat2( const double axis[AA_RESTRICT 3],
                                 double angle,
                                 double R[AA_RESTRICT 9] );

/// convert rotatoin vector to rotation matrix
AA_API void aa_tf_rotvec2rotmat( const double rv[AA_RESTRICT 3],
                                 double R[AA_RESTRICT 9] );


/** Convert orientation unit quaternion and translation vector to transformation matrix. */
AA_API void aa_tf_qv2tfmat( const double q[AA_RESTRICT 4], const double v[AA_RESTRICT 3],
                           double T[AA_RESTRICT 12] ) ;

/* AA_API void aa_tf_tfv2tfq( const double vrv[AA_RESTRICT 6],  */
/*                            double x[AA_RESTRICT 3], double quat[AA_RESTRICT 4] ); */
/* AA_API void aa_tf_tfq2tfv( const double x[AA_RESTRICT 3], const double quat[AA_RESTRICT 4],  */
/*                            double vrv[AA_RESTRICT 6] ); */

/** Convert Rotation Matrix to ZYX Euler Angles */
AA_API void aa_tf_rotmat2eulerzyx( const double R[AA_RESTRICT 9],
                                   double e[AA_RESTRICT 3] );


/** Convert quaternion to ZYX Euler Angles */
AA_API void aa_tf_quat2eulerzyx( const double q[AA_RESTRICT 4],
                                 double e[AA_RESTRICT 3] );

/**
 * Create declarations for Euler angle conversion functions.
 */
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

/** Angle about x axis */
AA_API void aa_tf_xangle2axang( double theta_x, double axang[AA_RESTRICT 4] );
/** Angle about y axis */
AA_API void aa_tf_yangle2axang( double theta_y, double axang[AA_RESTRICT 4] );
/** Angle about z axis */
AA_API void aa_tf_zangle2axang( double theta_z, double axang[AA_RESTRICT 4] );

/** Normalize the axis */
AA_API void aa_tf_axang_normalize( double axang[AA_RESTRICT 4] );

/**
 * Find the camera frame, looking in negative z direction.
 *
 *
 */
AA_API void
aa_tf_rotmat_mzlook( const double eye[AA_RESTRICT 3],
                     const double target[AA_RESTRICT 3],
                     const double up[AA_RESTRICT 3],
                     double R[AA_RESTRICT 9] );


/**
 * Find the camera frame, looking in negative z direction
 */
AA_API void
aa_tf_tfmat_mzlook( const double eye[AA_RESTRICT 3],
                    const double target[AA_RESTRICT 3],
                    const double up[AA_RESTRICT 3],
                    double T[AA_RESTRICT 12] );

/**
 * Find the camera frame, looking in negative z direction
 */
AA_API void
aa_tf_qmzlook( const double eye[AA_RESTRICT 3],
               const double target[AA_RESTRICT 3],
               const double up[AA_RESTRICT 3],
               double q[AA_RESTRICT 4] );

/**
 * Find the camera frame, looking in negative z direction
 */
AA_API void
aa_tf_qv_mzlook( const double eye[AA_RESTRICT 3],
                 const double target[AA_RESTRICT 3],
                 const double up[AA_RESTRICT 3],
                 double q[AA_RESTRICT 4],
                 double v[AA_RESTRICT 3] );

/**
 * Find the camera frame, looking in  negative z direction
 */
AA_API void
aa_tf_qutr_mzlook( const double eye[AA_RESTRICT 3],
                   const double target[AA_RESTRICT 3],
                   const double up[AA_RESTRICT 3],
                   double T[AA_RESTRICT 12] );


/* Dual Quaternions */

/** Dual quaternion addition */
AA_API void aa_tf_duqu_add( const double d1[AA_RESTRICT 8], const double d2[AA_RESTRICT 8],
                            double d3[AA_RESTRICT 8] );

/** Dual quaternion subtraction */
AA_API void aa_tf_duqu_sub( const double d1[AA_RESTRICT 8], const double d2[AA_RESTRICT 8],
                            double d3[AA_RESTRICT 8] );

/** Dual quaternion scalar multiplication */
AA_API void aa_tf_duqu_smul( double alpha, const double x[AA_RESTRICT 8],
                             double y[AA_RESTRICT 8] );

/** Dual quaternion multiplication */
AA_API void aa_tf_duqu_mul( const double d1[AA_RESTRICT 8], const double d2[AA_RESTRICT 8],
                            double d3[AA_RESTRICT 8] );

/** Construct matrix for left dual quaternion multiply
 * q*p = M*p
 */
AA_API void aa_tf_duqu_matrix_l( const double *q, double *M, size_t ldm );

/** Construct matrix for right dual quaternion multiply
 * p*q = M*p
 */
AA_API void aa_tf_duqu_matrix_r( const double *q, double *M, size_t ldm );

/** Construct matrix for left dual quaternion multiply
 * q*p = M*p
 */
AA_API void aa_tf_duqu_mat_l( const double *q, struct aa_dmat *M );

/** Construct matrix for right dual quaternion multiply
 * p*q = M*p
 */
AA_API void aa_tf_duqu_mat_r( const double *q, struct aa_dmat *M );

/** Dual quaternion multiply conjugate of d1 by d2 */
AA_API void aa_tf_duqu_cmul( const double d1[AA_RESTRICT 8], const double d2[AA_RESTRICT 8],
                             double d3[AA_RESTRICT 8] );

/** Dual quaternion multiply d1 by conjugate of d2 */
AA_API void aa_tf_duqu_mulc( const double d1[AA_RESTRICT 8], const double d2[AA_RESTRICT 8],
                             double d3[AA_RESTRICT 8] );

/** Dual quaternion conjugate */
AA_API void aa_tf_duqu_conj( const double d[AA_RESTRICT 8], double dconj[AA_RESTRICT 8] );

/** Dual quaternion exponential */
AA_API void aa_tf_duqu_exp( const double d[AA_RESTRICT 8], double e[AA_RESTRICT 8] );

/** Dual quaternion natural logarithm */
AA_API void aa_tf_duqu_ln( const double d[AA_RESTRICT 8], double e[AA_RESTRICT 8] );

/** Dual quaternion norm */
AA_API void aa_tf_duqu_norm( const double d[AA_RESTRICT 8], double *nreal, double *ndual );

/** Dual quaternion normalization */
AA_API void aa_tf_duqu_normalize( double d[AA_RESTRICT 8] );

/** Dual quaternion angle minimization */
AA_API void aa_tf_duqu_minimize( double d[AA_RESTRICT 8] );

/** Dual quaternion transformation */
AA_API void aa_tf_tf_duqu( const double d[AA_RESTRICT 8], const double p0[AA_RESTRICT 3],
                        double p1[AA_RESTRICT 3]  );

/** Dual quaternion transformation */
AA_API void aa_tf_duqu_tf( const double d[AA_RESTRICT 8], const double p0[AA_RESTRICT 3],
                        double p1[AA_RESTRICT 3]  );

/** Extract dual quaternion translation vector */
AA_API void aa_tf_duqu_trans( const double d[AA_RESTRICT 8], double v[AA_RESTRICT 3] );

/** Convert dual quaternion to transformation matrix */
AA_API void aa_tf_duqu2tfmat( const double d[AA_RESTRICT 8], double T[AA_RESTRICT 12] );

/** Convert transformation matrix to dual quaternion */
AA_API void aa_tf_tfmat2duqu( const double T[AA_RESTRICT 12], double d[AA_RESTRICT 8] ) ;

/** Convert orientation unit quaternion and translation vector to dual quaternion. */
AA_API void aa_tf_qv2duqu( const double q[AA_RESTRICT 4], const double v[AA_RESTRICT 3],
                           double d[AA_RESTRICT 8] ) ;

/** Pure translation dual quaternion. */
AA_API void aa_tf_xyz2duqu( double x, double y, double z,
                            double d[AA_RESTRICT 8] ) ;

/** Convert x angle and translation to dual quaternion. */
AA_API void aa_tf_xxyz2duqu( double theta, double x, double y, double z,
                             double d[AA_RESTRICT 8] );

/** Convert y angle and translation to dual quaternion. */
AA_API void aa_tf_yxyz2duqu( double theta, double x, double y, double z,
                             double d[AA_RESTRICT 8] );

/** Convert z angle and translation to dual quaternion. */
AA_API void aa_tf_zxyz2duqu( double theta, double x, double y, double z,
                             double d[AA_RESTRICT 8] );

/** Convert dual quaternion to orientation unit quaternion and translation vector. */
AA_API void aa_tf_duqu2qv( const double d[AA_RESTRICT 8],
                           double q[AA_RESTRICT 4], double v[AA_RESTRICT 3] ) ;

/// transformation matrix to  quaternion-translation
void aa_tf_tfmat2av( const double T[12], double q[AA_RESTRICT 4], double v[AA_RESTRICT 3] ) ;

/** Dual quaternion twist from velocity */
AA_API void aa_tf_duqu_vel2twist( const double d[AA_RESTRICT 8], const double dx[AA_RESTRICT 6],
                                  double t[AA_RESTRICT 8] ) ;

/** Dual quaternion twist to velocity */
AA_API void aa_tf_duqu_twist2vel( const double d[AA_RESTRICT 8], const double t[AA_RESTRICT 8],
                                  double dx[AA_RESTRICT 6] ) ;

/** Dual quaternion twist to derivative */
AA_API void aa_tf_duqu_twist2diff( const double d[AA_RESTRICT 8], const double t[AA_RESTRICT 8],
                                  double dd[AA_RESTRICT 8] ) ;

/** Dual quaternion derivative from velocity */
AA_API void aa_tf_duqu_vel2diff( const double d[AA_RESTRICT 8], const double dx[AA_RESTRICT 6],
                                 double dd[AA_RESTRICT 8] ) ;

/** Dual quaternion derivative to spatial velocity */
AA_API void aa_tf_duqu_diff2vel( const double d[AA_RESTRICT 8], const double dd[AA_RESTRICT 8],
                                 double dx[AA_RESTRICT 6] ) ;

/** Convert dual quaternion derivative to dual quaternion twist */

AA_API void aa_tf_duqu_diff2twist( const double d[AA_RESTRICT 8], const double dd[AA_RESTRICT 8],
                                   double twist[AA_RESTRICT 8] ) ;

/** Dual quaternion twist integration.
 * \param d0 initial position, dual quaternion
 * \param twist dual quaternion twist
 * \param dt time step
 * \param d1 final position, dual quaternion
 */
AA_API void aa_tf_duqu_stwist( const double d0[AA_RESTRICT 8], const double twist[AA_RESTRICT 8],
                               double dt, double d1[AA_RESTRICT 6] ) ;

/** Dual quaternion velocity integration.
 * \param d0 initial position, dual quaternion
 * \param dx spatial velocity
 * \param dt time step
 * \param d1 final position, dual quaternion
 */
AA_API void aa_tf_duqu_svel( const double d0[AA_RESTRICT 8], const double dx[AA_RESTRICT 6],
                             double dt, double d1[AA_RESTRICT 6] ) ;

/** Dual quaternion derivative integration.
 * \param d0 initial position, dual quaternion
 * \param dd dual quaternion derivative
 * \param dt time step
 * \param d1 final position, dual quaternion
 */
AA_API void aa_tf_duqu_sdiff( const double d0[AA_RESTRICT 8], const double dd[AA_RESTRICT 8],
                              double dt, double d1[AA_RESTRICT 6] ) ;


/**
 * Convert a velocity Jacobian to a dual quaternion derivative Jacobian.
 *
 * The velocity Jacobian Jvel relates velocities to configurations:
 * \f[
 *   \begin{bmatrix}\omega \\ \dot v\end{bmatrix}
 *   =
 *   \mathbf{J}_{\rm vel} \dot{\phi}
 * \f]
 *
 * The derivative Jacobian Js relates the dual quaternion derivative to configurations:
 * \f[
 *   \dot{S}
 *   =
 *   \mathbf{J}_S \dot{\phi}
 * \f]
 *
 * @param The pose as a dual quaternion
 * @param The velocity Jacobian
 * @param The dual quaternion derivative Jacobian
 */
AA_API void
aa_tf_duqu_jac_vel2diff(const double S[8], const struct aa_dmat *Jvel,
                        struct aa_dmat *Js );


/**
 * Compute the Jacobian of the quaternion logarithm.
 *
 * @param[in]  q  The quaternion
 * @param[out] Js Storage for the Jacobian, 4x4 matrix
 */
AA_API void
aa_tf_qln_jac(const double q[4], struct aa_dmat *Js );

/** Convert a pure dual quaternion to conventional dual quaternion.
*/
AA_API void
aa_tf_duqu2pure( const double S[AA_RESTRICT 8],
                 double v[AA_RESTRICT 6] );

/** Convert a conventional dual quaternion to pure dual quaternion.
*/
AA_API void
aa_tf_pure2duqu( const double v[AA_RESTRICT 6],
                 double S[AA_RESTRICT 8]);




/* DH Parameter Conversions
 *
 * Conventions:
 *  - Proximal: Craig and Lynch & Park
 *  - Distal: Murray, Li, & Sastry
 */


AA_API void
aa_tf_dhprox2tfmat( double alpha, double a, double d, double phi,
                    double T[AA_RESTRICT 12]);
AA_API void
aa_tf_dhprox2duqu( double alpha, double a, double d, double phi,
                   double S[AA_RESTRICT 8]);
AA_API void
aa_tf_dhprox2qutr( double alpha, double a, double d, double phi,
                   double E[AA_RESTRICT 7]);
AA_API void
aa_tf_dhprox2qv( double alpha, double a, double d, double phi,
                 double q[AA_RESTRICT 4], double v[3]);

AA_API void
aa_tf_dhdist2tfmat( double alpha, double a, double d, double phi,
                    double T[AA_RESTRICT 12]);
AA_API void
aa_tf_dhdist2duqu( double alpha, double a, double d, double phi,
                   double S[AA_RESTRICT 8]);
AA_API void
aa_tf_dhdist2qutr( double alpha, double a, double d, double phi,
                   double E[AA_RESTRICT 7]);
AA_API void
aa_tf_dhdist2qv( double alpha, double a, double d, double phi,
                 double q[AA_RESTRICT 4], double v[3]);

/* Misc */

AA_API void
aa_tf_relx_mean( size_t n, const double *R,
                 const double *X, size_t ldx,
                 const double *Y, size_t ldy,
                 double rel[3]);

AA_API void
aa_tf_relx_median( size_t n, const double *R,
                   const double *X, size_t ldx,
                   const double *Y, size_t ldy,
                   double rel[3]);

/**
 * Vector projection of a onto b.
 *
 * \f[
 *   \vec{c}
 *    = \mathrm{proj}_\vec{b}(\vec{a})
 *    = \frac{\vec{a} \cdot \vec{b}}{\vec{b} \cdot \vec{b}} \vec{b}
 * \f]
 */
AA_API
void aa_tf_proj( const double a[AA_RESTRICT 3],
                 const double b[AA_RESTRICT 3],
                 double c[AA_RESTRICT 3] );

/**
 * Orthogonal projection of a onto b.
 *
 * \f[
 *   \vec{c}
 *    = \vec{a} - \mathrm{proj}_\vec{b}(\vec{a})
 * \f]
 */
AA_API
void aa_tf_proj_orth( const double a[AA_RESTRICT 3],
                      const double b[AA_RESTRICT 3],
                      double c[AA_RESTRICT 3] );


#ifdef __cplusplus
}
#endif

#endif //AMINO_TF_H
