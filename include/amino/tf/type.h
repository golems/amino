/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2022, Colorado School of Mines
 * All rights reserved.
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

#ifndef AMINO_TF_TYPE_H
#define AMINO_TF_TYPE_H

/**
 * @file type.h
 *
 * Types for transforms.
 *
 *
 * All types passed by reference are contiguous sequences of double floats and
 * may safely be passed with a double* pointer to the first element.  The
 * structs in this file specify the memory layouts.
 *
 * Complex numbers and 2D vectors are represented with C99 complex numbers and
 * are passed and returned by value.
 */

/*--------------*/
/* Planar Types */
/*--------------*/

/** Typedef for C99 complex numbers. */
typedef double _Complex aa_tf_cmplx;

/** Represent 2D vectors as complex numbers. */
typedef aa_tf_cmplx aa_tf_vec2;


/**
 * Memory layout for a planar rotation matrix.
 *
 * Column major order.
 */
typedef struct aa_tf_rotmatp {
    union {
        struct {
            aa_tf_vec2 col0; ///< column 0
            aa_tf_vec2 col1; ///< column 1
        };
        double data[4]; ///< data array
    };
} aa_tf_rotmatp;

/**
 * Memory layout for planar transform as complex number and translation vector.
 */
typedef struct aa_tf_cv {
    union {
        double data[4]; ///< data array
        struct {
            aa_tf_cmplx c;  ///< rotation unit complex
            aa_tf_vec2 v;   ///< translation vector
        };
    };
} aa_tf_cv;

/**
 * Memory layout for a planar transformation matrix.
 *
 * It is a column-major matrix, but the bottom row is omitted because
 * this row is the same for all SE(2) transformation matrices.
 *
 * The first 4 elements are a column major rotation matrix.
 * The last 2 elements are the origin vector.
 */
typedef struct aa_tf_tfmatp {
    union {
        struct {
            aa_tf_rotmatp R;  ///< the rotation matrix part
            aa_tf_vec2 v;     ///< the origin vector part
        };
        double data[6]; ///< data array
    };
} aa_tf_tfmatp;

/*---------------*/
/* Spatial Types */
/*---------------*/

/** Index of vector x element */
#define AA_TF_X 0
/** Index of vector y element */
#define AA_TF_Y 1
/** Index of vector z element */
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
 * Memory layout for axis-Angle rotation in x,y,z,angle order.
 *
 * The first three elements are the axis x,y,z coordinates.  The final element
 * is the angle.
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

/** Index of quaternion vector part */
#define AA_TF_QUAT_V 0
/** Index of quaternion vector part */
#define AA_TF_QUAT_XYZ AA_TF_QUAT_V
/** Index of quaternion vector x */
#define AA_TF_QUAT_X (AA_TF_QUAT_V + AA_TF_X)
/** Index of quaternion vector y */
#define AA_TF_QUAT_Y (AA_TF_QUAT_V + AA_TF_Y)
/** Index of quaternion vector z */
#define AA_TF_QUAT_Z (AA_TF_QUAT_V + AA_TF_Z)
/** Index of quaternion scalar part */
#define AA_TF_QUAT_W 3

/**
 * Memory layout for a quaternion, x,y,z,w order.
 *
 * The first three elements are the vector (x,y,z).  The last element is the
 * scalar (w).
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
            aa_tf_quat_t r;  ///< rotation unit quaternion
            aa_tf_vec3_t v;  ///< translation vector
        };
        double data[7]; ///< data array
    };
} aa_tf_qv_t;

/**
 * Memory layout for a dual quaternion.
 */
typedef struct aa_tf_duqu {
    union {
        struct {
            aa_tf_quat_t real;  ///< real part
            aa_tf_quat_t dual;  ///< dual part
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

/** Index of spatial velocity translational part */
#define AA_TF_DX_V 0
/** Index of spatial velocity rotational part */
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

/** Index of quaternion-translation quaternion part */
#define AA_TF_QUTR_Q  0
/** Index of quaternion-translation quaternion x */
#define AA_TF_QUTR_QX (AA_TF_QUTR_Q + AA_TF_QUAT_X)
/** Index of quaternion-translation quaternion y */
#define AA_TF_QUTR_QY (AA_TF_QUTR_Q + AA_TF_QUAT_Y)
/** Index of quaternion-translation quaternion z */
#define AA_TF_QUTR_QZ (AA_TF_QUTR_Q + AA_TF_QUAT_Z)
/** Index of quaternion-translation quaternion w */
#define AA_TF_QUTR_QW (AA_TF_QUTR_Q + AA_TF_QUAT_W)

/** Index of quaternion-translation translation part */
#define AA_TF_QUTR_T  4
/** Index of quaternion-translation translation x */
#define AA_TF_QUTR_TX (AA_TF_QUTR_T + AA_TF_X)
/** Index of quaternion-translation translation y */
#define AA_TF_QUTR_TY (AA_TF_QUTR_T + AA_TF_Y)
/** Index of quaternion-translation translation z */
#define AA_TF_QUTR_TZ (AA_TF_QUTR_T + AA_TF_Z)

/** index of dual quaternion real part */
#define AA_TF_DUQU_REAL 0
/** index of dual quaternion dual part */
#define AA_TF_DUQU_DUAL 4

/** index of dual quaternion real w */
#define AA_TF_DUQU_REAL_W    (AA_TF_DUQU_REAL + AA_TF_QUAT_W)
/** index of dual quaternion real xyz */
#define AA_TF_DUQU_REAL_XYZ  (AA_TF_DUQU_REAL + AA_TF_QUAT_XYZ)
/** index of dual quaternion real x */
#define AA_TF_DUQU_REAL_X  (AA_TF_DUQU_REAL + AA_TF_QUAT_X)
/** index of dual quaternion real y */
#define AA_TF_DUQU_REAL_Y  (AA_TF_DUQU_REAL + AA_TF_QUAT_Y)
/** index of dual quaternion real z */
#define AA_TF_DUQU_REAL_Z  (AA_TF_DUQU_REAL + AA_TF_QUAT_Z)

/** index of dual quaternion dual w */
#define AA_TF_DUQU_DUAL_W    (AA_TF_DUQU_DUAL + AA_TF_QUAT_W)
/** index of dual quaternion dual xyz */
#define AA_TF_DUQU_DUAL_XYZ  (AA_TF_DUQU_DUAL + AA_TF_QUAT_XYZ)
/** index of dual quaternion dual x */
#define AA_TF_DUQU_DUAL_X  (AA_TF_DUQU_DUAL + AA_TF_QUAT_X)
/** index of dual quaternion dual y */
#define AA_TF_DUQU_DUAL_Y  (AA_TF_DUQU_DUAL + AA_TF_QUAT_Y)
/** index of dual quaternion dual z */
#define AA_TF_DUQU_DUAL_Z  (AA_TF_DUQU_DUAL + AA_TF_QUAT_Z)


#endif //AMINO_TF_TYPE_H
