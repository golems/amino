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

#ifndef AMINO_TF_PLANAR_H
#define AMINO_TF_PLANAR_H

/**
 * @file planar.h
 *
 * Low-level operations for SE(2), planar orientations and transformations.
 *
 */

/** Construct a complex number */
#define AA_TF_CMPLX(x, y) ((x) + _Complex_I * (y))

/** Extract the real part of a complex number */
#define AA_TF_CMPLX_REAL(c) (creal(c))

/** Extract the imaginary part of a complex number */
#define AA_TF_CMPLX_IMAG(c) (cimag(c))

/** Construct a 2D vector */
#define AA_TF_VEC2(x, y) AA_TF_CMPLX(x, y)

/** Extract the x-coordinate of a 2D vector */
#define AA_TF_VEC2_X(v) AA_TF_CMPLX_REAL(v)

/** Extract the y-coordinate of a 2D vector */
#define AA_TF_VEC2_Y(v) AA_TF_CMPLX_IMAG(v)

/** Load a 2D vector from memory */
#define AA_TF_VEC2_LD(ptr) AA_TF_VEC2((ptr)[0], (ptr)[1])

/** Store a 2D vector in memory */
#define AA_TF_VEC2_ST(v, ptr)       \
    do {                            \
        (ptr)[0] = AA_TF_VEC2_X(v); \
        (ptr)[1] = AA_TF_VEC2_Y(v); \
    } while (0)

/** Initializer for planar rotation matrix identity element */
#define AA_TF_ROTMATP_IDENT_INITIALIZER {1,0, 0,1};


/** Initializer for complex number and translation identity element */
#define AA_TF_COTR_IDENT_INITIALIZER {1,0, 0,0 };

/** Initializer for planar transformation matrix identity element */
#define AA_TF_TFMATP_IDENT_INITIALIZER {1,0, 0,1, 0,0};

/** Constant for complex number identity element */
#define AA_TF_CMPLX_IDENT AA_TF_CMPLX(1, 0)


/** Exponential of a pure (zero real part) complex number.
 *
 * @f[
 *  e^{\theta \imath} = \cos \theta + \imath \sin \theta
 * @f]
 */
AA_API aa_tf_cmplx
aa_tf_cpexp(double theta);

/** Logarithm of a unit complex number
 *  @f[
 *  \ln\left(a + b\imath\right) = \operatorname{atan2}\left(b,a\right)\;,
 *  \quad \text{where } \sqrt{a^2+b^2} = 1
 *  @f]
 */
AA_API double
aa_tf_culn(const aa_tf_cmplx c);

/** Exponential of a pure (zero real part) complex number as a rotation matrix.
 */
AA_API void
aa_tf_rotmatp_pexp(double angle, double R[AA_RESTRICT 4]);

/**
 * Logarithm of a planar rotation matrix.
 */
AA_API double
aa_tf_rotmatp_uln(const double R[4]);

/**
 * Apply a planar rotation via complex number.
 */
AA_API aa_tf_vec2
aa_tf_crot(const aa_tf_cmplx c, const aa_tf_vec2 v);

/**
 * Apply a planar rotation via rotation matrix.
 */
AA_API aa_tf_vec2
aa_tf_rotmatp_rot(const double R[AA_RESTRICT 4], const aa_tf_vec2 v);

/**
 * Conjugate a complex number, giving the inverse rotation.
 */
AA_API aa_tf_cmplx
aa_tf_cconj(const aa_tf_cmplx c);

/**
 * Invert a planar rotation matrix.
 */
AA_API void aa_tf_rotmatp_inv2(const double R[AA_RESTRICT 4],
                               double Ri[AA_RESTRICT 4]);

/**
 * Multiply complex numbers.
 */
AA_API aa_tf_cmplx
aa_tf_cmul(const aa_tf_cmplx c1, const aa_tf_cmplx c2);

/**
 * Multiply planar rotation matrices.
 *
 */
AA_API void aa_tf_rotmatp_mul(const double R1[AA_RESTRICT 4],
                              const double R2[AA_RESTRICT 4],
                              double R12[AA_RESTRICT 4]);

/** Convert a complex number to a rotation angle. */
AA_API double
aa_tf_cmplx2angle(const aa_tf_cmplx c);

/** Convert a unit complex number to a rotation matrix. */
AA_API void
aa_tf_cmplx2rotmatp(const aa_tf_cmplx c, double R[AA_RESTRICT 4]);

/** Convert a rotation angle to a complex number. */
AA_API aa_tf_cmplx
aa_tf_angle2cmplx(double angle);

/** Convert a rotation angle to a planar rotation matrix. */
AA_API void
aa_tf_angle2rotmatp(double angle, double R[AA_RESTRICT 4]);

/** Convert a planar rotation matrix to a complex number. */
AA_API aa_tf_cmplx
aa_tf_rotmatp2cmplx(const double R[AA_RESTRICT 4]);

/** Convert a planar rotation matrix to a rotation angle. */
AA_API double
aa_tf_rotmatp2angle(const double R[AA_RESTRICT 4]);

/* ---------------------- */
/* --- Transformation --- */
/* ---------------------- */

/**
 * Apply a planar transform represented with a complex number and translation vector.
 *
 * @param c The rotation part of the transform
 * @param v The translation part of the transform
 * @param p The point to transform
 */
AA_API aa_tf_vec2
aa_tf_cv_tf(const aa_tf_cmplx c, const aa_tf_vec2 v, const aa_tf_vec2 p);

/**
 * Apply a planar transform represented matrix.
 */
AA_API aa_tf_vec2
aa_tf_tfmatp_tf(const double T[AA_RESTRICT 6], const aa_tf_vec2 p);

/**
 * Convert a complex number and translation vector to a planar transformation matrix.
 */
AA_API void
aa_tf_cv2tfmatp(const aa_tf_cmplx c, const aa_tf_vec2 v, double T[AA_RESTRICT 6]);

/**
 * Convert a planar transformation matrix to a complex number and translation
 * vector.
 */
AA_API void
aa_tf_tfmatp2cv(const double T[AA_RESTRICT 6], aa_tf_cmplx *c, aa_tf_vec2 *v);

/**
 * Multiply (chain) two complex number and translation vector planar transforms.
 */
AA_API void
aa_tf_cv_mul(const aa_tf_cmplx c1, const aa_tf_vec2 v1,
             const aa_tf_cmplx c2, const aa_tf_vec2 v2,
             aa_tf_cmplx *c12, aa_tf_vec2 *v12);

/**
 * Multiply (chain) two complex number and translation vector planar transforms.
 */
AA_API void
aa_tf_tfmatp_mul(const double T1[AA_RESTRICT 6], const double T2[AA_RESTRICT 6],
                 double T12[AA_RESTRICT 6]);

/**
 * Invert a complex number and translation vector planar transform.
 */
AA_API void
aa_tf_cv_inv(const aa_tf_cmplx c, const aa_tf_vec2 v,
             aa_tf_cmplx *ci, aa_tf_vec2 *vi);

/**
 * Invert a planar transformation matrix.
 */
AA_API void
aa_tf_tfmatp_inv2(const double T[AA_RESTRICT 6], double Ti[AA_RESTRICT 6]);

#endif //AMINO_TFP_H
