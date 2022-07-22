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

#include <complex.h>

#include "amino.h"
#include "amino_internal.h"



/* static void s_print_cmplx(FILE *f, aa_tf_cmplx c) { */
/*     fprintf(f, "%f\t + %fi\n", AA_TF_CMPLX_REAL(c), AA_TF_CMPLX_IMAG(c)); */
/* } */

AA_API aa_tf_vec2
aa_tf_crot(aa_tf_cmplx cc, const aa_tf_vec2 v)
{
    return v*cc;
}

AA_API aa_tf_vec2
aa_tf_rotmatp_rot(const double R[AA_RESTRICT 4], const aa_tf_vec2 v)
{
    double x = AA_TF_VEC2_X(v);
    double y = AA_TF_VEC2_Y(v);
    double xp = R[0] * x + R[2] * y;
    double yp = R[1] * x + R[3] * y;
    aa_tf_vec2 vp = AA_TF_VEC2(xp, yp);

    return vp;
}
AA_API aa_tf_cmplx
aa_tf_cconj(const aa_tf_cmplx c)
{
    return AA_TF_CMPLX(AA_TF_CMPLX_REAL(c), -AA_TF_CMPLX_IMAG(c));
}

AA_API void aa_tf_rotmatp_inv2(const double R[AA_RESTRICT 4],
                               double Ri[AA_RESTRICT 4])
{
    Ri[0] = R[0];
    Ri[1] = R[2];
    Ri[2] = R[1];
    Ri[3] = R[3];
}

AA_API aa_tf_cmplx
aa_tf_cmul(const aa_tf_cmplx c1, const aa_tf_cmplx c2)
{
    return c1*c2;
}

AA_API void
aa_tf_rotmatp_mul(const double R1[AA_RESTRICT 4],
                  const double R2[AA_RESTRICT 4],
                  double R12[AA_RESTRICT 4])
{
    R12[0] = R1[0] * R2[0] + R1[2] * R2[1];
    R12[1] = R1[1] * R2[0] + R1[3] * R2[1];

    R12[2] = R1[0] * R2[2] + R1[2] * R2[3];
    R12[3] = R1[1] * R2[2] + R1[3] * R2[3];
}

AA_API aa_tf_cmplx
aa_tf_cpexp(double angle)
{
    return AA_TF_CMPLX(cos(angle), sin(angle));
}

AA_API double
aa_tf_culn(aa_tf_cmplx c)
{
    return atan2(cimag(c), creal(c));
}

AA_API double
aa_tf_cmplx2angle(const aa_tf_cmplx c)
{
    return aa_tf_culn(c);
}

AA_API void
aa_tf_cmplx2rotmatp(const aa_tf_cmplx cc, double R[AA_RESTRICT 4])
{
    double c = AA_TF_CMPLX_REAL(cc), s = AA_TF_CMPLX_IMAG(cc);
    R[0] = c;
    R[1] = s;
    R[2] = -s;
    R[3] = c;
}

AA_API aa_tf_cmplx
aa_tf_angle2cmplx(double angle)
{
    return aa_tf_cpexp(angle);
}

AA_API void
aa_tf_angle2rotmatp(double angle, double R[AA_RESTRICT 4])
{
    aa_tf_cmplx2rotmatp(aa_tf_angle2cmplx(angle), R);
}

AA_API aa_tf_cmplx
aa_tf_rotmatp2cmplx(const double R[AA_RESTRICT 4])
{
    double c = (R[0] + R[3])/2;
    double s = (R[1] - R[2])/2;
    return AA_TF_CMPLX(c, s);
}

AA_API double
aa_tf_rotmatp2angle(const double R[AA_RESTRICT 4])
{
    return aa_tf_cmplx2angle(aa_tf_rotmatp2cmplx(R));
}

AA_API void
aa_tf_rotmatp_pexp(double angle, double R[AA_RESTRICT 4])
{
    aa_tf_angle2rotmatp(angle, R);
}

AA_API double
aa_tf_rotmatp_uln(const double R[4])
{
    return aa_tf_rotmatp2angle(R);
}

/* ---------------------- */
/* --- Transformation --- */
/* ---------------------- */

AA_API aa_tf_vec2
aa_tf_cv_tf(const aa_tf_cmplx z, const aa_tf_vec2 v, const aa_tf_vec2 p)
{
    /* return v + z*p; */

    /* Expand operations to enable FMAs */
    double px = AA_TF_VEC2_X(p), py = AA_TF_VEC2_Y(p);
    double c = AA_TF_CMPLX_REAL(z), s = AA_TF_CMPLX_IMAG(z);

    double x = AA_TF_VEC2_X(v) + c*px - s*py;
    double y = AA_TF_VEC2_Y(v) + s*px + c*py;

    return AA_TF_VEC2(x, y);
}

AA_API aa_tf_vec2
aa_tf_tfmatp_tf(const double T[AA_RESTRICT 6], const aa_tf_vec2 p)
{
    double px = AA_TF_VEC2_X(p), py = AA_TF_VEC2_Y(p);

    /*
     * FMA: 4
     */
    double x = T[4] + T[0]*px + T[2]*py;
    double y = T[5] + T[1]*px + T[3]*py;

    return AA_TF_VEC2(x, y);
}

AA_API void
aa_tf_cv2tfmatp(const aa_tf_cmplx c, const aa_tf_vec2 v, double T[AA_RESTRICT 6])
{
    aa_tf_cmplx2rotmatp(c, T);
    AA_TF_VEC2_ST(v, T+4);
}

AA_API void
aa_tf_tfmatp2cv(const double T[AA_RESTRICT 6], aa_tf_cmplx *c, aa_tf_vec2 *v)
{
    *c = aa_tf_rotmatp2cmplx(T);
    *v = AA_TF_VEC2_LD(T+4);
}

AA_API void
aa_tf_cv_mul(const aa_tf_cmplx c1, const aa_tf_vec2 v1,
             const aa_tf_cmplx c2, const aa_tf_vec2 v2,
             aa_tf_cmplx *c12, aa_tf_vec2 *v12)
{
    // c12 = c1*c2
    // v12 = v1 + c1*v2
    *c12 = aa_tf_cmul(c1, c2);
    *v12 = aa_tf_cv_tf(c1, v1, v2);
}

AA_API void
aa_tf_tfmatp_mul(const double T1[AA_RESTRICT 6], const double T2[AA_RESTRICT 6],
                 double T12[AA_RESTRICT 6])
{
    aa_tf_rotmatp_mul(T1, T2, T12);
    aa_tf_vec2 T2v = AA_TF_VEC2_LD(T2 + 4);
    aa_tf_vec2 T12v = aa_tf_tfmatp_tf(T1, T2v);
    AA_TF_VEC2_ST(T12v, T12+4);
}

AA_API void
aa_tf_cv_inv(const aa_tf_cmplx c, const aa_tf_vec2 v,
             aa_tf_cmplx *ci, aa_tf_vec2 *vi)
{
    *ci = aa_tf_cconj(c);
    *vi = -aa_tf_crot(*ci, v);
}

AA_API void
aa_tf_tfmatp_inv2(const double T[AA_RESTRICT 6], double Ti[AA_RESTRICT 6])
{
    aa_tf_rotmatp_inv2(T, Ti);
    aa_tf_vec2 vv = AA_TF_VEC2(-T[4], -T[5]);
    AA_TF_VEC2_ST(aa_tf_rotmatp_rot(Ti, vv), Ti + 4);
}
